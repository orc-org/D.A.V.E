#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "drive_package/drive_math.hpp"

TEST(DriveMathTest, JoystickHandlesShortAxesSafeFallbackAndNaN)
{
    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    auto res = drive_package::mixJoystickCommand(nan_val, 1.0f, 0.0f, 1.0f, 0.05f, true);
    EXPECT_FLOAT_EQ(res.left, 0.0f);
    EXPECT_FLOAT_EQ(res.right, 0.0f);

    res = drive_package::mixJoystickCommand(1.0f, 1.0f, nan_val, 1.0f, 0.05f, true);
    EXPECT_FLOAT_EQ(res.left, 0.0f);
    EXPECT_FLOAT_EQ(res.right, 0.0f);
}

TEST(DriveMathTest, JoystickRightTriggerForwardPolarityAndReverseYawConsistency)
{
    // Triggers unpressed (+1.0), stick neutral -> zero command
    auto neutral = drive_package::mixJoystickCommand(1.0f, 1.0f, 0.0f, 1.0f, 0.05f, true);
    EXPECT_FLOAT_EQ(neutral.left, 0.0f);
    EXPECT_FLOAT_EQ(neutral.right, 0.0f);

    // R2 fully pressed (-1.0) -> forward
    auto fwd = drive_package::mixJoystickCommand(1.0f, -1.0f, 0.0f, 1.0f, 0.05f, true);
    EXPECT_NEAR(fwd.left, 1.0f, 1e-5f);
    EXPECT_NEAR(fwd.right, 1.0f, 1e-5f);

    // L2 fully pressed (-1.0) -> reverse
    auto rev = drive_package::mixJoystickCommand(-1.0f, 1.0f, 0.0f, 1.0f, 0.05f, true);
    EXPECT_NEAR(rev.left, -1.0f, 1e-5f);
    EXPECT_NEAR(rev.right, -1.0f, 1e-5f);

    // Legacy polarity option (right_trigger_forward=false)
    auto legacy_fwd = drive_package::mixJoystickCommand(-1.0f, 1.0f, 0.0f, 1.0f, 0.05f, false);
    EXPECT_NEAR(legacy_fwd.left, 1.0f, 1e-5f);
    EXPECT_NEAR(legacy_fwd.right, 1.0f, 1e-5f);

    // Left stick (+0.5) turns CCW (right > left) consistently both forward and in reverse
    auto fwd_left = drive_package::mixJoystickCommand(1.0f, -0.5f, 0.5f, 1.0f, 0.05f, true);
    EXPECT_GT(fwd_left.right, fwd_left.left);
    auto rev_left = drive_package::mixJoystickCommand(-0.5f, 1.0f, 0.5f, 1.0f, 0.05f, true);
    EXPECT_GT(rev_left.right, rev_left.left);

    // Zero-point turn
    auto spin_ccw = drive_package::mixJoystickCommand(1.0f, 1.0f, 0.8f, 0.5f, 0.05f, true);
    EXPECT_NEAR(spin_ccw.left, -0.4f, 1e-5f);
    EXPECT_NEAR(spin_ccw.right, 0.4f, 1e-5f);
}

TEST(DriveMathTest, TwistMixingUsesSiKinematicsAndClamps)
{
    // v=0.5 m/s, omega=1.0 rad/s, W=0.60 m => v_L = 0.2, v_R = 0.8
    auto w = drive_package::mixTwistCommand(0.5, 1.0, 1.0f, true, 0.60, 1.0);
    EXPECT_NEAR(w.left, 0.2f, 1e-5f);
    EXPECT_NEAR(w.right, 0.8f, 1e-5f);

    // Saturation preserves [-1, 1]
    auto sat = drive_package::mixTwistCommand(2.0, 0.0, 1.0f, true, 0.60, 1.0);
    EXPECT_FLOAT_EQ(sat.left, 1.0f);
    EXPECT_FLOAT_EQ(sat.right, 1.0f);

    // NaN safety
    const double nan_val = std::numeric_limits<double>::quiet_NaN();
    auto bad = drive_package::mixTwistCommand(nan_val, 0.5, 1.0f, true, 0.60, 1.0);
    EXPECT_FLOAT_EQ(bad.left, 0.0f);
    EXPECT_FLOAT_EQ(bad.right, 0.0f);
}

TEST(DriveMathTest, VescCanCommandHandlesBrakeClampAndErpmConversion)
{
    // Force brake uses CAN_PACKET_SET_CURRENT_BRAKE (comm_id=2) with mA
    auto brk = drive_package::computeVescCanCommand(
        "erpm", 0.5f, true, true, 0.15, 7, 1.0, 400.0, 15000.0, 40.0, 0.9, 5.0, true);
    EXPECT_EQ(brk.comm_id, 2);
    EXPECT_EQ(brk.value, 5000);

    // Coast when brake_on_zero is false
    auto coast = drive_package::computeVescCanCommand(
        "erpm", 0.0f, false, true, 0.15, 7, 1.0, 400.0, 15000.0, 40.0, 0.9, 5.0, false);
    EXPECT_EQ(coast.comm_id, 1);
    EXPECT_EQ(coast.value, 0);

    // Metric conversion: 1 m/s on r=0.15m -> 60/(2*pi*0.15) * 7 = ~445.63 ERPM, clamped by max_erpm
    auto metric = drive_package::computeVescCanCommand(
        "erpm", 1.0f, false, true, 0.15, 7, 1.0, 400.0, 15000.0, 40.0, 0.9, 5.0, true);
    EXPECT_EQ(metric.comm_id, 3);
    EXPECT_NEAR(metric.value, 445, 2);

    // High speed clamps to max_erpm instead of overflowing
    auto clamped = drive_package::computeVescCanCommand(
        "erpm", 50.0f, false, true, 0.15, 7, 10.0, 400.0, 15000.0, 40.0, 0.9, 5.0, true);
    EXPECT_EQ(clamped.comm_id, 3);
    EXPECT_EQ(clamped.value, 15000);

    // Negative clamp
    auto rev_clamped = drive_package::computeVescCanCommand(
        "erpm", -50.0f, false, true, 0.15, 7, 10.0, 400.0, 15000.0, 40.0, 0.9, 5.0, true);
    EXPECT_EQ(rev_clamped.comm_id, 3);
    EXPECT_EQ(rev_clamped.value, -15000);

    // NaN falls back cleanly to safe stop/brake
    auto nan_cmd = drive_package::computeVescCanCommand(
        "erpm", std::numeric_limits<float>::quiet_NaN(), false, true, 0.15, 7, 1.0,
        400.0, 15000.0, 40.0, 0.9, 5.0, true);
    EXPECT_EQ(nan_cmd.comm_id, 2);
}

TEST(DriveMathTest, JoystickAppliesSteeringDeadzoneWhileDriving)
{
    // R2 half-pressed (-0.0 -> 0.5 throttle), small steering inside 0.05 deadzone -> straight
    auto straight = drive_package::mixJoystickCommand(1.0f, 0.0f, 0.03f, 1.0f, 0.05f, true);
    EXPECT_FLOAT_EQ(straight.left, straight.right);
    EXPECT_NEAR(straight.left, 0.5f, 1e-5f);
}

TEST(DriveMathTest, VescCurrentAndDutyModesHonorBrakeOnZeroAndClamp)
{
    // Near-zero current/duty commands brake when brake_on_zero=true, coast when false
    auto cur_brk = drive_package::computeVescCanCommand(
        "current", 0.005f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, 0.95, 5.0, true);
    EXPECT_EQ(cur_brk.comm_id, 2);
    EXPECT_EQ(cur_brk.value, 5000);

    auto cur_coast = drive_package::computeVescCanCommand(
        "current", 0.0f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, 0.95, 5.0, false);
    EXPECT_EQ(cur_coast.comm_id, 1);
    EXPECT_EQ(cur_coast.value, 0);

    auto duty_coast = drive_package::computeVescCanCommand(
        "duty", 0.0f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, 0.95, 5.0, false);
    EXPECT_EQ(duty_coast.comm_id, 0);
    EXPECT_EQ(duty_coast.value, 0);

    // Commands beyond [-1, 1] are clamped to [-1, 1] before scaling max_current / max_duty
    auto cur_clamped = drive_package::computeVescCanCommand(
        "current", 2.5f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, 0.95, 5.0, true);
    EXPECT_EQ(cur_clamped.comm_id, 1);
    EXPECT_EQ(cur_clamped.value, 14000);

    auto duty_clamped = drive_package::computeVescCanCommand(
        "duty", -2.5f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, 0.95, 5.0, true);
    EXPECT_EQ(duty_clamped.comm_id, 0);
    EXPECT_EQ(duty_clamped.value, -95000);

    // NaN max_erpm / max_current / max_duty falls back cleanly without UB/overflow
    const double nan_d = std::numeric_limits<double>::quiet_NaN();
    auto nan_max_erpm = drive_package::computeVescCanCommand(
        "erpm", 0.5f, false, true, 0.15, 2, 50.0, 100.0, nan_d, 14.0, 0.95, 5.0, true);
    EXPECT_EQ(nan_max_erpm.comm_id, 2);
    EXPECT_EQ(nan_max_erpm.value, 5000);

    auto nan_max_cur = drive_package::computeVescCanCommand(
        "current", 0.5f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, nan_d, 0.95, 5.0, true);
    EXPECT_EQ(nan_max_cur.comm_id, 1);
    EXPECT_EQ(nan_max_cur.value, 0);

    auto nan_max_duty = drive_package::computeVescCanCommand(
        "duty", 0.5f, false, false, 0.15, 2, 50.0, 100.0, 7000.0, 14.0, nan_d, 5.0, true);
    EXPECT_EQ(nan_max_duty.comm_id, 0);
    EXPECT_EQ(nan_max_duty.value, 0);
}
