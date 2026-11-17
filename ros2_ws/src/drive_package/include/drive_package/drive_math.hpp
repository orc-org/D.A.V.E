#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace drive_package
{

struct WheelPair
{
    float left = 0.0f;
    float right = 0.0f;
};

struct VescCanCommand
{
    uint8_t comm_id = 1;  // 0=SET_DUTY, 1=SET_CURRENT, 2=SET_CURRENT_BRAKE, 3=SET_RPM
    int32_t value = 0;
};

inline bool isFinitePair(double a, double b)
{
    return std::isfinite(a) && std::isfinite(b);
}

/** Mix normalized joystick trigger throttle and stick steering into [-1, 1] wheel commands. */
inline WheelPair mixJoystickCommand(
    float lt_raw, float rt_raw, float steering_raw,
    float sensitivity, float deadzone, bool right_trigger_forward)
{
    if (!std::isfinite(lt_raw) || !std::isfinite(rt_raw) || !std::isfinite(steering_raw)) {
        return {0.0f, 0.0f};
    }

    const float lt_norm = std::clamp((1.0f - lt_raw) * 0.5f, 0.0f, 1.0f);
    const float rt_norm = std::clamp((1.0f - rt_raw) * 0.5f, 0.0f, 1.0f);
    float throttle = right_trigger_forward ? (rt_norm - lt_norm) : (lt_norm - rt_norm);
    if (std::abs(throttle) < deadzone) {
        throttle = 0.0f;
    }

    float steering = std::clamp(steering_raw, -1.0f, 1.0f);
    float left = 0.0f;
    float right = 0.0f;
    if (throttle != 0.0f) {
        // Scale differential term by |throttle| so left stick (+steering) always turns CCW
        // in both forward and reverse travel instead of inverting yaw sign while backing up.
        const float diff = std::abs(throttle) * steering;
        left = throttle - diff;
        right = throttle + diff;
    } else {
        if (std::abs(steering) < deadzone) {
            steering = 0.0f;
        }
        left = -steering;
        right = steering;
    }

    left = std::clamp(left * sensitivity, -1.0f, 1.0f);
    right = std::clamp(right * sensitivity, -1.0f, 1.0f);
    return {left, right};
}

/** Mix Twist (linear.x, angular.z) into normalized [-1, 1] wheel commands. */
inline WheelPair mixTwistCommand(
    double linear_x, double angular_z, float sensitivity,
    bool cmd_vel_is_si, double wheel_separation, double max_wheel_speed)
{
    if (!isFinitePair(linear_x, angular_z) || max_wheel_speed <= 0.0) {
        return {0.0f, 0.0f};
    }

    double left = 0.0;
    double right = 0.0;
    if (cmd_vel_is_si) {
        const double half_track = 0.5 * std::max(0.0, wheel_separation);
        const double v_left = linear_x - angular_z * half_track;
        const double v_right = linear_x + angular_z * half_track;
        left = v_left / max_wheel_speed;
        right = v_right / max_wheel_speed;
    } else {
        left = linear_x - angular_z;
        right = linear_x + angular_z;
    }

    return {
        std::clamp(static_cast<float>(left) * sensitivity, -1.0f, 1.0f),
        std::clamp(static_cast<float>(right) * sensitivity, -1.0f, 1.0f)
    };
}

/** Encode a wheel velocity command (normalized or m/s) into a VESC CAN packet with braking and ERPM clamps. */
inline VescCanCommand computeVescCanCommand(
    const char *control_mode, float velocity, bool force_brake,
    bool use_metric_speed, double wheel_radius, int pole_pairs, double gear_ratio,
    double min_erpm, double max_erpm, double max_current, double max_duty,
    double brake_current, bool brake_on_zero)
{
    if (!std::isfinite(velocity)) {
        velocity = 0.0f;
    }

    const int32_t brake_ma = static_cast<int32_t>(std::max(0.0, brake_current) * 1000.0);
    if (force_brake && brake_on_zero && brake_ma > 0) {
        return {2, brake_ma};  // CAN_PACKET_SET_CURRENT_BRAKE
    }
    if (force_brake) {
        return {1, 0};         // CAN_PACKET_SET_CURRENT = 0
    }

    const std::string_view mode(control_mode ? control_mode : "duty");
    if (mode == "erpm") {
        double raw_erpm = 0.0;
        if (use_metric_speed && wheel_radius > 0.0) {
            const double erpm_per_m_s =
                (60.0 / (2.0 * M_PI * wheel_radius)) * static_cast<double>(pole_pairs) * gear_ratio;
            raw_erpm = static_cast<double>(velocity) * erpm_per_m_s;
        } else {
            raw_erpm = static_cast<double>(velocity) * max_erpm;
        }

        const int32_t erpm_cap = static_cast<int32_t>(std::max(0.0, max_erpm));
        int32_t target_erpm = static_cast<int32_t>(
            std::clamp(raw_erpm, -static_cast<double>(erpm_cap), static_cast<double>(erpm_cap)));

        if (std::abs(velocity) < 0.01f || std::abs(target_erpm) < 10) {
            if (brake_on_zero && brake_ma > 0) {
                return {2, brake_ma};  // CAN_PACKET_SET_CURRENT_BRAKE prevents downhill coasting
            }
            return {1, 0};
        }

        if (std::abs(target_erpm) < min_erpm && erpm_cap >= static_cast<int32_t>(min_erpm)) {
            target_erpm = (target_erpm >= 0) ? static_cast<int32_t>(min_erpm)
                                             : -static_cast<int32_t>(min_erpm);
        }
        return {3, target_erpm};  // CAN_PACKET_SET_RPM
    }

    if (mode == "current") {
        return {1, static_cast<int32_t>(velocity * max_current * 1000.0)};
    }

    const float scaled_duty = velocity * static_cast<float>(max_duty);
    return {0, static_cast<int32_t>(scaled_duty * 100000.0f)};
}

}  // namespace drive_package
