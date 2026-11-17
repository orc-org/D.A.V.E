#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "drive_package/drive_math.hpp"
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
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<std::string>("control_mode", "erpm");
        this->declare_parameter<double>("max_erpm", 7000.0);
        this->declare_parameter<double>("min_erpm", 100.0);
        this->declare_parameter<double>("max_current", 14.0);
        this->declare_parameter<double>("max_duty", 0.95);
        this->declare_parameter<double>("brake_current", 5.0);
        this->declare_parameter<bool>("brake_on_zero", true);
        this->declare_parameter<bool>("enable_control_mode_lock", true);
        this->declare_parameter<double>("ramp_rate", 1.0);
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<double>("command_timeout", 0.5);
        this->declare_parameter<double>("status_timeout", 0.5);

        this->declare_parameter<int>("fl_can_id", 53);
        this->declare_parameter<int>("fr_can_id", 44);
        this->declare_parameter<int>("rl_can_id", 48);
        this->declare_parameter<int>("rr_can_id", 7);

        this->declare_parameter<double>("wheel_radius", 0.10);
        this->declare_parameter<int>("pole_pairs", 2);
        this->declare_parameter<double>("gear_ratio", 50.0);
        this->declare_parameter<bool>("use_metric_speed", true);

        can_interface_ = this->get_parameter("can_interface").as_string();
        control_mode_ = this->get_parameter("control_mode").as_string();
        max_erpm_ = this->get_parameter("max_erpm").as_double();
        min_erpm_ = this->get_parameter("min_erpm").as_double();
        max_current_ = this->get_parameter("max_current").as_double();
        max_duty_ = this->get_parameter("max_duty").as_double();
        brake_current_ = this->get_parameter("brake_current").as_double();
        brake_on_zero_ = this->get_parameter("brake_on_zero").as_bool();
        enable_control_mode_lock_ = this->get_parameter("enable_control_mode_lock").as_bool();
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

        if (initCanSocket(can_interface_)) {
            RCLCPP_INFO(this->get_logger(), "Successfully initialized CAN socket on %s", can_interface_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "CAN socket initialization failed. Running in simulation mode.");
        }

        rclcpp::QoS sub_qos(10);
        sub_qos.reliable();
        sub_qos.durability_volatile();

        auto make_wheel_cb = [this](float &target_ref, rclcpp::Time &stamp_ref) {
            return [this, &target_ref, &stamp_ref](const std_msgs::msg::Float32::SharedPtr msg) {
                if (!std::isfinite(msg->data)) {
                    target_ref = 0.0f;
                    return;
                }
                if (enable_control_mode_lock_ && robot_control_mode_ != "drive") {
                    target_ref = 0.0f;
                    return;
                }
                target_ref = std::clamp(msg->data, -1.0f, 1.0f);
                stamp_ref = steady_.now();
            };
        };

        fl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/front_left", sub_qos, make_wheel_cb(target_fl_, last_fl_));
        fr_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/front_right", sub_qos, make_wheel_cb(target_fr_, last_fr_));
        rl_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/rear_left", sub_qos, make_wheel_cb(target_rl_, last_rl_));
        rr_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/rear_right", sub_qos, make_wheel_cb(target_rr_, last_rr_));

        if (enable_control_mode_lock_) {
            mode_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/control_mode", 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    robot_control_mode_ = msg->data;
                    if (robot_control_mode_ != "drive") {
                        zeroAllTargetsAndCurrents();
                    }
                });
        }

        wheel_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/wheel_states", 10);

        double timer_sec = 1.0 / std::max(1.0, publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_sec),
            std::bind(&VescCanDriverNode::updateLoop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "VESC CAN Driver Initialized: iface=%s mode=%s wheel_radius=%.3f max_erpm=%.0f brake=%.1fA timeout=%.2fs",
            can_interface_.c_str(), control_mode_.c_str(), wheel_radius_,
            max_erpm_, brake_current_, command_timeout_);
    }

    ~VescCanDriverNode()
    {
        if (can_socket_ != -1) {
            close(can_socket_);
            RCLCPP_INFO(this->get_logger(), "Closed CAN socket.");
        }
    }

private:
    void zeroAllTargetsAndCurrents()
    {
        target_fl_ = target_fr_ = target_rl_ = target_rr_ = 0.0f;
        current_fl_ = current_fr_ = current_rl_ = current_rr_ = 0.0f;
    }

    bool initCanSocket(const std::string &interface)
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed for CAN: %s", std::strerror(errno));
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get CAN interface index for %s: %s", interface.c_str(), std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Binding CAN socket failed: %s", std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    void sendVescCanFrame(int can_id, uint8_t comm_id, int32_t val)
    {
        if (can_socket_ == -1 || can_id < 0) {
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
        if (bytes_written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Failed to write to CAN socket: %s", std::strerror(errno));
        }
    }

    void readStatusFrames()
    {
        if (can_socket_ == -1) return;
        struct can_frame frame;
        const int ids[4] = {fl_can_id_, fr_can_id_, rl_can_id_, rr_can_id_};
        for (int n = 0; n < 200; ++n) {
            ssize_t got = read(can_socket_, &frame, sizeof(frame));
            if (got < static_cast<ssize_t>(sizeof(frame))) break;
            if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8) continue;
            uint32_t id = frame.can_id & CAN_EFF_MASK;
            if (((id >> 8) & 0xFF) != 9) continue;
            int vesc_id = static_cast<int>(id & 0xFF);
            for (int w = 0; w < 4; ++w) {
                if (ids[w] != vesc_id) continue;
                const uint8_t *d = frame.data;
                int32_t erpm = static_cast<int32_t>(
                    (uint32_t(d[0]) << 24) | (uint32_t(d[1]) << 16) |
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
        static const char *names[4] = {
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint"};
        const double rad_s_per_erpm =
            2.0 * M_PI / 60.0 / (static_cast<double>(pole_pairs_) * gear_ratio_);
        const rclcpp::Time now = steady_.now();
        sensor_msgs::msg::JointState js;
        for (int w = 0; w < 4; ++w) {
            if (!status_seen_[w] || (now - status_time_[w]).seconds() > status_timeout_) continue;
            js.name.push_back(names[w]);
            js.velocity.push_back(status_erpm_[w] * rad_s_per_erpm);
            js.effort.push_back(status_current_[w]);
        }
        if (js.name.empty()) return;
        js.header.stamp = this->get_clock()->now();
        wheel_state_pub_->publish(js);
    }

    void updateLoop()
    {
        readStatusFrames();
        publishWheelStates();

        const double dt = 1.0 / std::max(1.0, publish_rate_);
        const double max_change = ramp_rate_ * dt;
        const bool mode_locked = enable_control_mode_lock_ && robot_control_mode_ != "drive";

        bool deadman_fl = mode_locked;
        bool deadman_fr = mode_locked;
        bool deadman_rl = mode_locked;
        bool deadman_rr = mode_locked;

        if (command_timeout_ > 0.0) {
            const rclcpp::Time now = steady_.now();
            bool stopped = false;
            stopped |= enforceTimeout(now, last_fl_, target_fl_, current_fl_, deadman_fl);
            stopped |= enforceTimeout(now, last_fr_, target_fr_, current_fr_, deadman_fr);
            stopped |= enforceTimeout(now, last_rl_, target_rl_, current_rl_, deadman_rl);
            stopped |= enforceTimeout(now, last_rr_, target_rr_, current_rr_, deadman_rr);
            if (stopped) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "No motor command for %.2f s: braking/stopping wheels (deadman)", command_timeout_);
            }
        }

        if (mode_locked) {
            zeroAllTargetsAndCurrents();
        } else {
            current_fl_ = rampValue(current_fl_, target_fl_, max_change);
            current_fr_ = rampValue(current_fr_, target_fr_, max_change);
            current_rl_ = rampValue(current_rl_, target_rl_, max_change);
            current_rr_ = rampValue(current_rr_, target_rr_, max_change);
        }

        processAndSend("Front Left", current_fl_, fl_can_id_, deadman_fl);
        processAndSend("Front Right", current_fr_, fr_can_id_, deadman_fr);
        processAndSend("Rear Left", current_rl_, rl_can_id_, deadman_rl);
        processAndSend("Rear Right", current_rr_, rr_can_id_, deadman_rr);
    }

    bool enforceTimeout(
        const rclcpp::Time &now, const rclcpp::Time &last,
        float &target, float &current, bool &timed_out)
    {
        if ((now - last).seconds() <= command_timeout_) return false;
        timed_out = true;
        bool wasMoving = (target != 0.0f || current != 0.0f);
        target = 0.0f;
        current = 0.0f;
        return wasMoving;
    }

    float rampValue(float current, float target, double max_change)
    {
        if (current < target) {
            return std::min(target, static_cast<float>(current + max_change));
        } else if (current > target) {
            return std::max(target, static_cast<float>(current - max_change));
        }
        return current;
    }

    void processAndSend(const std::string &wheel_name, float velocity, int can_id, bool force_brake)
    {
        if (can_id < 0) return;

        const auto cmd = drive_package::computeVescCanCommand(
            control_mode_.c_str(), velocity, force_brake,
            use_metric_speed_, wheel_radius_, pole_pairs_, gear_ratio_,
            min_erpm_, max_erpm_, max_current_, max_duty_,
            brake_current_, brake_on_zero_);

        if (control_mode_ != "erpm" && control_mode_ != "current" && !force_brake) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[%s] CAN ID=%d | cmd=%.3f | send_val=%d",
                wheel_name.c_str(), can_id, velocity, cmd.value);
        }
        sendVescCanFrame(can_id, cmd.comm_id, cmd.value);
    }

    std::string can_interface_;
    std::string control_mode_;
    std::string robot_control_mode_ = "drive";
    double max_erpm_;
    double min_erpm_;
    double max_current_;
    double max_duty_;
    double brake_current_;
    bool brake_on_zero_;
    bool enable_control_mode_lock_;
    double ramp_rate_;
    double publish_rate_;
    double command_timeout_;
    double status_timeout_;

    std::array<int32_t, 4> status_erpm_{};
    std::array<double, 4> status_current_{};
    std::array<rclcpp::Time, 4> status_time_{
        rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Time(0, 0, RCL_STEADY_TIME),
        rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Time(0, 0, RCL_STEADY_TIME)};
    std::array<bool, 4> status_seen_{};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_state_pub_;

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

    float target_fl_ = 0.0f;
    float target_fr_ = 0.0f;
    float target_rl_ = 0.0f;
    float target_rr_ = 0.0f;

    float current_fl_ = 0.0f;
    float current_fr_ = 0.0f;
    float current_rl_ = 0.0f;
    float current_rr_ = 0.0f;

    int can_socket_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fr_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rr_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VescCanDriverNode>());
    rclcpp::shutdown();
    return 0;
}
