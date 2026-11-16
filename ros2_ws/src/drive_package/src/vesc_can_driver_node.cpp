#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>

class VescCanDriverNode : public rclcpp::Node
{
public:
    VescCanDriverNode() : Node("vesc_can_driver_node"), can_socket_(-1)
    {
        // declare and retrieve VESC driver parameters
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<std::string>("control_mode", "erpm"); // "duty", "erpm", "current"
        this->declare_parameter<double>("max_erpm", 7000.0);
        this->declare_parameter<double>("min_erpm", 100.0); // minimum ERPM threshold for smooth startup
        this->declare_parameter<double>("max_current", 14.0); // max current in amps
        this->declare_parameter<double>("max_duty", 0.95); // max duty cycle limit (0.0 to 1.0)
        this->declare_parameter<double>("ramp_rate", 1.0); // max velocity change per second
        this->declare_parameter<double>("publish_rate", 50.0); // CAN transmit loop frequency in Hz
        // Deadman: a wheel whose command is older than this (seconds) is stopped at once, without ramping.
        // The dashboard publishes at 10 Hz and joy_node repeats at 20 Hz, so 0.5 s is several missed messages.
        // Without it, a dropped link or a crashed publisher left the rover driving its last command. <= 0 disables.
        this->declare_parameter<double>("command_timeout", 0.5);
        // Measured wheel speed: each VESC's CAN STATUS frame (packet 9: ERPM, current, duty) is read back and
        // published on /wheel_states (sensor_msgs/JointState; velocity = wheel rad/s, effort = motor current A).
        // The VESCs must have "CAN Status Message" STATUS 1 enabled in VESC Tool (e.g. 50 Hz).
        // A wheel with no status frame for status_timeout seconds is left out of the message.
        this->declare_parameter<double>("status_timeout", 0.5);

        // CAN IDs for the 4 wheels
        this->declare_parameter<int>("fl_can_id", 53);
        this->declare_parameter<int>("fr_can_id", 44);
        this->declare_parameter<int>("rl_can_id", 48);
        this->declare_parameter<int>("rr_can_id", 7);

        // Physical wheel & motor parameters for speed (m/s) -> ERPM calculation
        this->declare_parameter<double>("wheel_radius", 0.10);  // meters (0.10m = 10cm radius)
        this->declare_parameter<int>("pole_pairs", 2);          // BLDC motor pole pairs (e.g. 7 for 14-pole motor)
        this->declare_parameter<double>("gear_ratio", 50.0);     // Gearbox reduction ratio (1.0 = direct drive)
        this->declare_parameter<bool>("use_metric_speed", true); // true = velocity in m/s, false = normalized -1..1

        can_interface_ = this->get_parameter("can_interface").as_string();
        control_mode_ = this->get_parameter("control_mode").as_string();
        max_erpm_ = this->get_parameter("max_erpm").as_double();
        min_erpm_ = this->get_parameter("min_erpm").as_double();
        max_current_ = this->get_parameter("max_current").as_double();
        max_duty_ = this->get_parameter("max_duty").as_double();
        ramp_rate_ = this->get_parameter("ramp_rate").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        command_timeout_ = this->get_parameter("command_timeout").as_double();
        status_timeout_ = this->get_parameter("status_timeout").as_double();

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        pole_pairs_ = this->get_parameter("pole_pairs").as_int();
        gear_ratio_ = this->get_parameter("gear_ratio").as_double();
        use_metric_speed_ = this->get_parameter("use_metric_speed").as_bool();

        fl_can_id_ = this->get_parameter("fl_can_id").as_int();
        fr_can_id_ = this->get_parameter("fr_can_id").as_int();
        rl_can_id_ = this->get_parameter("rl_can_id").as_int();
        rr_can_id_ = this->get_parameter("rr_can_id").as_int();

        // initialize SocketCAN interface
        if (initCanSocket(can_interface_))
        {
            RCLCPP_INFO(this->get_logger(), "Successfully initialized CAN socket on %s", can_interface_.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "CAN socket initialization failed. Running in simulation mode.");
        }

        // subscriptions to independent motor command channels
        rclcpp::QoS sub_qos(10);
        sub_qos.reliable();
        sub_qos.durability_volatile();

        fl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/front_left", sub_qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                RCLCPP_DEBUG(this->get_logger(), "Received front_left: %.3f", msg->data);   // was INFO on every message
                target_fl_ = std::max(-1.0f, std::min(1.0f, msg->data));
                last_fl_ = steady_.now();
            });

        fr_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/front_right", sub_qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                target_fr_ = std::max(-1.0f, std::min(1.0f, msg->data));
                last_fr_ = steady_.now();
            });

        rl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/rear_left", sub_qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                target_rl_ = std::max(-1.0f, std::min(1.0f, msg->data));
                last_rl_ = steady_.now();
            });

        rr_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/rear_right", sub_qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                target_rr_ = std::max(-1.0f, std::min(1.0f, msg->data));
                last_rr_ = steady_.now();
            });

        wheel_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/wheel_states", 10);

        // periodic CAN transmit & ramping loop
        double timer_sec = 1.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_sec),
            std::bind(&VescCanDriverNode::updateLoop, this));

        RCLCPP_INFO(this->get_logger(), "VESC CAN Driver Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "Interface: %s | Mode: %s | Max Duty: %.2f | Ramp Rate: %.1f/s | Command timeout: %.2f s",
                    can_interface_.c_str(), control_mode_.c_str(), max_duty_, ramp_rate_, command_timeout_);
    }

    ~VescCanDriverNode()
    {
        if (can_socket_ != -1)
        {
            close(can_socket_);
            RCLCPP_INFO(this->get_logger(), "Closed CAN socket.");
        }
    }

private:
    bool initCanSocket(const std::string &interface)
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed for CAN: %s", std::strerror(errno));
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get CAN interface index for %s: %s", interface.c_str(), std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Binding CAN socket failed: %s", std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        // make socket non-blocking to prevent locking up the ROS 2 executor when the TX queue is full
        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        return true;
    }

    void sendVescCanFrame(int can_id, uint8_t comm_id, int32_t val)
    {
        if (can_socket_ == -1 || can_id < 0)
        {
            return;
        }

        struct can_frame frame;
        uint32_t raw_id = (static_cast<uint32_t>(comm_id) << 8) | (static_cast<uint32_t>(can_id) & 0xFF);
        
        frame.can_id = raw_id | CAN_EFF_FLAG;
        frame.can_dlc = 4;

        frame.data[0] = (val >> 24) & 0xFF;
        frame.data[1] = (val >> 16) & 0xFF;
        frame.data[2] = (val >> 8) & 0xFF;
        frame.data[3] = val & 0xFF;

        ssize_t bytes_written = write(can_socket_, &frame, sizeof(struct can_frame));
        if (bytes_written < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK) 
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                     "Failed to write to CAN socket: %s", std::strerror(errno));
            }
        }
    }

    // Drain the (non-blocking) socket and keep the latest STATUS frame of each wheel.
    // STATUS (CAN_PACKET_STATUS = 9), extended ID = (9 << 8) | vesc_id, 8 bytes big-endian:
    //   int32 ERPM, int16 current x10 (A), int16 duty x1000
    void readStatusFrames()
    {
        if (can_socket_ == -1) return;
        struct can_frame frame;
        const int ids[4] = {fl_can_id_, fr_can_id_, rl_can_id_, rr_can_id_};
        for (int n = 0; n < 200; ++n)   // bounded: 4 VESCs at 50 Hz is ~4 frames per 20 ms loop
        {
            ssize_t got = read(can_socket_, &frame, sizeof(frame));
            if (got < static_cast<ssize_t>(sizeof(frame))) break;   // EAGAIN (nothing left) or short read
            if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8) continue;
            uint32_t id = frame.can_id & CAN_EFF_MASK;
            if (((id >> 8) & 0xFF) != 9) continue;
            int vesc_id = static_cast<int>(id & 0xFF);
            for (int w = 0; w < 4; ++w)
            {
                if (ids[w] != vesc_id) continue;
                const uint8_t *d = frame.data;
                int32_t erpm = static_cast<int32_t>((uint32_t(d[0]) << 24) | (uint32_t(d[1]) << 16) |
                                                    (uint32_t(d[2]) << 8) | uint32_t(d[3]));
                int16_t current_x10 = static_cast<int16_t>((uint16_t(d[4]) << 8) | uint16_t(d[5]));
                status_erpm_[w] = erpm;
                status_current_[w] = current_x10 / 10.0;
                status_time_[w] = steady_.now();
                status_seen_[w] = true;
            }
        }
    }

    void publishWheelStates()
    {
        static const char *names[4] = {"front_left_wheel_joint", "front_right_wheel_joint",
                                       "rear_left_wheel_joint", "rear_right_wheel_joint"};
        // wheel angular speed (rad/s) = ERPM / (pole pairs * gear ratio) * 2 pi / 60
        const double rad_s_per_erpm = 2.0 * M_PI / 60.0 / (static_cast<double>(pole_pairs_) * gear_ratio_);
        const rclcpp::Time now = steady_.now();
        sensor_msgs::msg::JointState js;
        for (int w = 0; w < 4; ++w)
        {
            if (!status_seen_[w] || (now - status_time_[w]).seconds() > status_timeout_) continue;
            js.name.push_back(names[w]);
            js.velocity.push_back(status_erpm_[w] * rad_s_per_erpm);
            js.effort.push_back(status_current_[w]);
        }
        if (js.name.empty()) return;   // no VESC is reporting: publish nothing rather than zeros
        js.header.stamp = this->get_clock()->now();
        wheel_state_pub_->publish(js);
    }

    void updateLoop()
    {
        readStatusFrames();
        publishWheelStates();

        double dt = 1.0 / publish_rate_;
        double max_change = ramp_rate_ * dt;

        // deadman: stop any wheel whose command is stale (no ramp, so the rover stops at once)
        if (command_timeout_ > 0.0)
        {
            const rclcpp::Time now = steady_.now();
            bool stopped = false;
            stopped |= enforceTimeout(now, last_fl_, target_fl_, current_fl_);
            stopped |= enforceTimeout(now, last_fr_, target_fr_, current_fr_);
            stopped |= enforceTimeout(now, last_rl_, target_rl_, current_rl_);
            stopped |= enforceTimeout(now, last_rr_, target_rr_, current_rr_);
            if (stopped)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "No motor command for %.2f s: wheels stopped (deadman)", command_timeout_);
            }
        }

        // apply ramping to each motor
        current_fl_ = rampValue(current_fl_, target_fl_, max_change);
        current_fr_ = rampValue(current_fr_, target_fr_, max_change);
        current_rl_ = rampValue(current_rl_, target_rl_, max_change);
        current_rr_ = rampValue(current_rr_, target_rr_, max_change);

        // transmit updated setpoints to active CAN IDs
        processAndSend("Front Left", current_fl_, fl_can_id_);
        processAndSend("Front Right", current_fr_, fr_can_id_);
        processAndSend("Rear Left", current_rl_, rl_can_id_);
        processAndSend("Rear Right", current_rr_, rr_can_id_);
    }

    // Zeroes a wheel whose last command is older than command_timeout_. Returns true only when the wheel
    // was still commanded to move, so the warning fires on a real stop and not every cycle while idle.
    bool enforceTimeout(const rclcpp::Time &now, const rclcpp::Time &last, float &target, float &current)
    {
        if ((now - last).seconds() <= command_timeout_) return false;
        bool wasMoving = (target != 0.0f || current != 0.0f);
        target = 0.0f;
        current = 0.0f;
        return wasMoving;
    }

    float rampValue(float current, float target, double max_change)
    {
        if (current < target)
        {
            return std::min(target, static_cast<float>(current + max_change));
        }
        else if (current > target)
        {
            return std::max(target, static_cast<float>(current - max_change));
        }
        return current;
    }

    void processAndSend(const std::string &wheel_name, float velocity, int can_id)
    {
        if (can_id < 0) return;

        uint8_t comm_id = 0; 
        int32_t send_val = 0;

        if (control_mode_ == "erpm")
        {
            int32_t target_erpm = 0;
            if (use_metric_speed_)
            {
                // ERPM = velocity_m_s * (60 / (2 * pi * wheel_radius)) * pole_pairs * gear_ratio
                double erpm_per_m_s = (60.0 / (2.0 * M_PI * wheel_radius_)) * static_cast<double>(pole_pairs_) * gear_ratio_;
                target_erpm = static_cast<int32_t>(velocity * erpm_per_m_s);
            }
            else
            {
                target_erpm = static_cast<int32_t>(velocity * max_erpm_);
            }

            // When idle/stopped, send SET_CURRENT = 0 to relax motor and avoid 0-RPM PID jitter
            if (std::abs(velocity) < 0.01f || std::abs(target_erpm) < 10)
            {
                comm_id = 1; // CAN_PACKET_SET_CURRENT
                send_val = 0;
            }
            else
            {
                comm_id = 3; // CAN_PACKET_SET_RPM
                // Enforce minimum ERPM threshold for stable commutation on startup
                if (std::abs(target_erpm) < min_erpm_)
                {
                    send_val = (target_erpm >= 0) ? static_cast<int32_t>(min_erpm_) 
                                                  : -static_cast<int32_t>(min_erpm_);
                }
                else
                {
                    send_val = target_erpm;
                }
            }
        }
        else if (control_mode_ == "current")
        {
            comm_id = 1; // CAN_PACKET_SET_CURRENT
            send_val = static_cast<int32_t>(velocity * max_current_ * 1000.0);
        }
        else // default to duty cycle
        {
            comm_id = 0; // CAN_PACKET_SET_DUTY
            float scaled_velocity = velocity * static_cast<float>(max_duty_);
            send_val = static_cast<int32_t>(scaled_velocity * 100000.0f);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] CAN ID=%d | cmd=%.3f | scaled=%.3f | send_val=%d",
                wheel_name.c_str(), can_id, velocity, scaled_velocity, send_val);
        }

        sendVescCanFrame(can_id, comm_id, send_val);
    }

    // parameters
    std::string can_interface_;
    std::string control_mode_;
    double max_erpm_;
    double min_erpm_;
    double max_current_;
    double max_duty_;
    double ramp_rate_;
    double publish_rate_;
    double command_timeout_;
    double status_timeout_;

    // latest VESC STATUS per wheel (fl, fr, rl, rr)
    std::array<int32_t, 4> status_erpm_{};
    std::array<double, 4> status_current_{};
    std::array<rclcpp::Time, 4> status_time_{rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Time(0, 0, RCL_STEADY_TIME),
                                             rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Time(0, 0, RCL_STEADY_TIME)};
    std::array<bool, 4> status_seen_{};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_state_pub_;

    // time each wheel last received a command, on a steady clock (unaffected by sim time or clock jumps);
    // they start at the clock's epoch, so the wheels stay stopped until the first command arrives
    rclcpp::Clock steady_{RCL_STEADY_TIME};
    rclcpp::Time last_fl_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time last_fr_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time last_rl_{0, 0, RCL_STEADY_TIME};
    rclcpp::Time last_rr_{0, 0, RCL_STEADY_TIME};

    double wheel_radius_;
    int pole_pairs_;
    double gear_ratio_;
    bool use_metric_speed_;

    int fl_can_id_;
    int fr_can_id_;
    int rl_can_id_;
    int rr_can_id_;

    // target inputs from topics
    float target_fl_ = 0.0f;
    float target_fr_ = 0.0f;
    float target_rl_ = 0.0f;
    float target_rr_ = 0.0f;

    // current ramped outputs sent to CAN
    float current_fl_ = 0.0f;
    float current_fr_ = 0.0f;
    float current_rl_ = 0.0f;
    float current_rr_ = 0.0f;

    int can_socket_;

    // subscriptions and timer
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fr_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rr_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VescCanDriverNode>());
    rclcpp::shutdown();
    return 0;
}
