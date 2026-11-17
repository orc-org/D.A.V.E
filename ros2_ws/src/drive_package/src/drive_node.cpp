#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "drive_package/drive_math.hpp"
#include <algorithm>
#include <string>

class JoyDrive : public rclcpp::Node
{
public:
    JoyDrive() : Node("drive_node")
    {
        this->declare_parameter<bool>("right_trigger_forward", true);
        this->declare_parameter<bool>("cmd_vel_is_si", true);
        this->declare_parameter<double>("wheel_separation", 0.60);
        this->declare_parameter<double>("max_wheel_speed", 1.0);
        this->declare_parameter<double>("joy_deadzone", 0.05);

        right_trigger_forward_ = this->get_parameter("right_trigger_forward").as_bool();
        cmd_vel_is_si_ = this->get_parameter("cmd_vel_is_si").as_bool();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();
        joy_deadzone_ = static_cast<float>(this->get_parameter("joy_deadzone").as_double());

        rclcpp::QoS sub_qos(10);
        sub_qos.reliable();
        sub_qos.durability_volatile();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", sub_qos,
            std::bind(&JoyDrive::joyCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", sub_qos,
            std::bind(&JoyDrive::cmdVelCallback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/control_mode", 10,
            std::bind(&JoyDrive::modeCallback, this, std::placeholders::_1));

        sensitivity_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sensitivity", 10,
            std::bind(&JoyDrive::sensitivityCallback, this, std::placeholders::_1));

        rclcpp::QoS pub_qos(10);
        pub_qos.reliable();
        pub_qos.durability_volatile();

        front_left_pub_  = this->create_publisher<std_msgs::msg::Float32>("/motor/front_left", pub_qos);
        front_right_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/front_right", pub_qos);
        rear_left_pub_   = this->create_publisher<std_msgs::msg::Float32>("/motor/rear_left", pub_qos);
        rear_right_pub_  = this->create_publisher<std_msgs::msg::Float32>("/motor/rear_right", pub_qos);

        RCLCPP_INFO(
            this->get_logger(),
            "Drive node running (cmd_vel_is_si=%s, right_trigger_forward=%s, wheel_sep=%.2f m, max_wheel=%.2f m/s)",
            cmd_vel_is_si_ ? "true" : "false",
            right_trigger_forward_ ? "true" : "false",
            wheel_separation_,
            max_wheel_speed_);
    }

private:
    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Drive mode changed to: %s", current_mode_.c_str());
        if (current_mode_ != "drive") {
            publishWheelPair({0.0f, 0.0f});
        }
    }

    void sensitivityCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "ultra") {
            sensitivity_ = 0.25f;
        } else if (msg->data == "fine") {
            sensitivity_ = 0.5f;
        } else if (msg->data == "coarse") {
            sensitivity_ = 1.0f;
        } else {
            RCLCPP_WARN(this->get_logger(), "Ignoring unknown /sensitivity preset: '%s'", msg->data.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sensitivity changed to %s (%.2f)", msg->data.c_str(), sensitivity_);
    }

    void publishWheelPair(const drive_package::WheelPair &wheels)
    {
        std_msgs::msg::Float32 flmsg, frmsg, rlmsg, rrmsg;
        flmsg.data = wheels.left;
        rlmsg.data = wheels.left;
        frmsg.data = wheels.right;
        rrmsg.data = wheels.right;

        front_left_pub_->publish(flmsg);
        front_right_pub_->publish(frmsg);
        rear_left_pub_->publish(rlmsg);
        rear_right_pub_->publish(rrmsg);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (current_mode_ != "drive") {
            return;
        }
        if (msg->axes.size() < 6) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Ignoring /joy message with %zu axes (expected >= 6); stopping wheels.",
                msg->axes.size());
            publishWheelPair({0.0f, 0.0f});
            return;
        }

        const float steering = msg->axes[0];
        const float lt_raw = msg->axes[4];
        const float rt_raw = msg->axes[5];
        RCLCPP_DEBUG(
            this->get_logger(),
            "Controller Inputs: LT=%.3f RT=%.3f LStickX=%.3f",
            lt_raw, rt_raw, steering);

        const auto wheels = drive_package::mixJoystickCommand(
            lt_raw, rt_raw, steering, sensitivity_, joy_deadzone_, right_trigger_forward_);
        publishWheelPair(wheels);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Motor Power Levels: L=%d%% | R=%d%%",
            static_cast<int>(wheels.left * 100.0f),
            static_cast<int>(wheels.right * 100.0f));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_mode_ != "drive") {
            return;
        }

        const auto wheels = drive_package::mixTwistCommand(
            msg->linear.x, msg->angular.z, sensitivity_,
            cmd_vel_is_si_, wheel_separation_, max_wheel_speed_);
        publishWheelPair(wheels);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "cmd_vel -> Left Wheels: %.2f | Right Wheels: %.2f", wheels.left, wheels.right);
    }

    bool right_trigger_forward_ = true;
    bool cmd_vel_is_si_ = true;
    double wheel_separation_ = 0.60;
    double max_wheel_speed_ = 1.0;
    float joy_deadzone_ = 0.05f;
    std::string current_mode_ = "drive";
    float sensitivity_ = 1.0f;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensitivity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rear_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rear_right_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyDrive>());
    rclcpp::shutdown();
    return 0;
}
