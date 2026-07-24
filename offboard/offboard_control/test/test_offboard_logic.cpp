// Unit tests for the node-independent logic of offboard_control (prompt 102):
// TransformPX4 frame math, SpeedController interpolation/arrival, the central
// setpoint validator and the diagnostics string mappings. No ROS graph needed.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "offboard_control/offboard_control.hpp"

namespace {
constexpr double kTol = 1e-9;
constexpr double kYawTol = 1e-6;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
}  // namespace

// --- TransformPX4 ---

TEST(TransformPX4, NormalizeYaw) {
    EXPECT_NEAR(TransformPX4::normalizeYaw(0.3), 0.3, kTol);
    EXPECT_NEAR(TransformPX4::normalizeYaw(2.5 * M_PI), 0.5 * M_PI, kTol);
    EXPECT_NEAR(TransformPX4::normalizeYaw(-2.5 * M_PI), -0.5 * M_PI, kTol);
    EXPECT_NEAR(std::abs(TransformPX4::normalizeYaw(3.0 * M_PI)), M_PI, kTol);
}

TEST(TransformPX4, GetDistance) {
    EXPECT_NEAR(TransformPX4::getDistance(Pose(0, 0, 0, 0), Pose(3, 4, 0, 1.0)), 5.0, kTol);
    EXPECT_NEAR(TransformPX4::getDistance(Pose(1, 1, 1, 0), Pose(1, 1, 1, 2.0)), 0.0, kTol);
}

TEST(TransformPX4, InterpolateYawMidpointAndWrap) {
    EXPECT_NEAR(TransformPX4::interpolateYaw(0.0, M_PI_2, 0.5), M_PI_4, kYawTol);
    EXPECT_NEAR(TransformPX4::interpolateYaw(0.0, M_PI_2, 0.0), 0.0, kYawTol);
    EXPECT_NEAR(TransformPX4::interpolateYaw(0.0, M_PI_2, 1.0), M_PI_2, kYawTol);
    // Wrap across ±pi: from 3.0 to -3.0 the short way is +0.283 rad.
    EXPECT_NEAR(TransformPX4::interpolateYaw(3.0, -3.0, 1.0), -3.0, kYawTol);
}

TEST(TransformPX4, EnuNedYawKnownPairs) {
    // NED yaw = pi/2 - ENU yaw (east=0 ENU is +90 deg NED).
    EXPECT_NEAR(TransformPX4::enuToNedYaw(0.0), M_PI_2, kYawTol);
    EXPECT_NEAR(TransformPX4::enuToNedYaw(M_PI_2), 0.0, kYawTol);
    EXPECT_NEAR(TransformPX4::nedToEnuYaw(M_PI_2), 0.0, kYawTol);
    // Roundtrip.
    const double yaw = 0.7;
    EXPECT_NEAR(TransformPX4::nedToEnuYaw(TransformPX4::enuToNedYaw(yaw)), yaw, kYawTol);
}

TEST(TransformPX4, EnuNedPoseRoundtrip) {
    const Pose enu(1.0, 2.0, 3.0, 0.4);
    const Pose ned = TransformPX4::enuToNedPose(enu);
    // ENU (x, y, z) -> NED (y, x, -z).
    EXPECT_NEAR(ned.x, 2.0, kTol);
    EXPECT_NEAR(ned.y, 1.0, kTol);
    EXPECT_NEAR(ned.z, -3.0, kTol);
    const Pose back = TransformPX4::nedToEnuPose(ned);
    EXPECT_NEAR(back.x, enu.x, kTol);
    EXPECT_NEAR(back.y, enu.y, kTol);
    EXPECT_NEAR(back.z, enu.z, kTol);
    EXPECT_NEAR(back.yaw, enu.yaw, kYawTol);
}

TEST(TransformPX4, EnuNedVelSwapsAxes) {
    CurrentPose enu_vel{};
    enu_vel.vx = 1.0;
    enu_vel.vy = 2.0;
    enu_vel.vz = 3.0;
    const CurrentPose ned_vel = TransformPX4::enuToNedVel(enu_vel);
    EXPECT_NEAR(ned_vel.vx, 2.0, kTol);
    EXPECT_NEAR(ned_vel.vy, 1.0, kTol);
    EXPECT_NEAR(ned_vel.vz, -3.0, kTol);
    const CurrentPose back = TransformPX4::nedToEnuVel(ned_vel);
    EXPECT_NEAR(back.vx, enu_vel.vx, kTol);
    EXPECT_NEAR(back.vy, enu_vel.vy, kTol);
    EXPECT_NEAR(back.vz, enu_vel.vz, kTol);
}

// --- SpeedController (map-frame target: no TF lookups happen) ---

class SpeedControllerTest : public ::testing::Test {
protected:
    SpeedControllerTest()
        : clock_(std::make_shared<rclcpp::Clock>()),
          buffer_(clock_),
          logger_(rclcpp::get_logger("test_speed_controller")) {}

    std::shared_ptr<rclcpp::Clock> clock_;
    tf2_ros::Buffer buffer_;
    rclcpp::Logger logger_;
    static constexpr long long kStartUs = 1000000;
};

TEST_F(SpeedControllerTest, LinearInterpolationTowardsTarget) {
    // Target ENU (2, 0, 0) in map == NED (0, 2, 0); 1 m/s -> 2 s of motion.
    SpeedController ctrl(logger_, Pose(0, 0, 0, 0), Pose(2.0, 0.0, 0.0, 0.0),
                         "map", "map", 1.0, kStartUs, buffer_, 0.1);
    ASSERT_FALSE(ctrl.hasTransformationError());

    const Pose at_start = ctrl.getCurrentSetpoint_NED(kStartUs);
    EXPECT_NEAR(at_start.x, 0.0, kTol);
    EXPECT_NEAR(at_start.y, 0.0, kTol);
    // Yaw is applied immediately (no angular ramp): ENU 0 -> NED pi/2.
    EXPECT_NEAR(at_start.yaw, M_PI_2, kYawTol);

    const Pose at_half = ctrl.getCurrentSetpoint_NED(kStartUs + 1000000);
    EXPECT_NEAR(at_half.x, 0.0, kTol);
    EXPECT_NEAR(at_half.y, 1.0, 1e-6);

    const Pose beyond = ctrl.getCurrentSetpoint_NED(kStartUs + 5000000);
    EXPECT_NEAR(beyond.y, 2.0, 1e-6);

    EXPECT_NEAR(ctrl.targetPoseNED().y, 2.0, 1e-6);
    EXPECT_NEAR(ctrl.targetSpeed(), 1.0, kTol);
}

TEST_F(SpeedControllerTest, IsEndByPositionAndYawTolerance) {
    SpeedController ctrl(logger_, Pose(0, 0, 0, 0), Pose(2.0, 0.0, 0.0, 0.0),
                         "map", "map", 1.0, kStartUs, buffer_, 0.1);
    ASSERT_FALSE(ctrl.hasTransformationError());
    // At target with matching yaw -> done.
    EXPECT_TRUE(ctrl.isEnd(Pose(0.0, 2.0, 0.0, M_PI_2)));
    // At target but wrong yaw -> not done.
    EXPECT_FALSE(ctrl.isEnd(Pose(0.0, 2.0, 0.0, 0.0)));
    // Away from target -> not done.
    EXPECT_FALSE(ctrl.isEnd(Pose(0.0, 0.0, 0.0, M_PI_2)));
    // Within the explicit tolerances -> done.
    EXPECT_TRUE(ctrl.isEnd(Pose(0.0, 1.5, 0.0, M_PI_2), 0.6, 0.1));
}

TEST_F(SpeedControllerTest, IsFinishedByNominalTime) {
    SpeedController ctrl(logger_, Pose(0, 0, 0, 0), Pose(2.0, 0.0, 0.0, 0.0),
                         "map", "map", 1.0, kStartUs, buffer_, 0.1);
    ASSERT_FALSE(ctrl.hasTransformationError());
    EXPECT_FALSE(ctrl.isFinished(kStartUs + 1900000));
    EXPECT_TRUE(ctrl.isFinished(kStartUs + 2100000));
}

// --- Central setpoint validation (prompt 99) ---

TEST(ValidateSetpoint, NavigateHistoricalChecks) {
    std::string reason;
    EXPECT_TRUE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, 0.5, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, kInf, 2, 3, 0.5, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, kInf, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, kNan, 2, 3, 0.5, 0.0, reason));  // x,y together
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, -0.5, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, kNan, kNan, kNan, 0.5, 0.0, reason));
    // Partial NaN target (keep current x/y) is legal.
    EXPECT_TRUE(validate_setpoint_numeric(NAVIGATE, kNan, kNan, 3, 0.5, 0.0, reason));
}

TEST(ValidateSetpoint, SpeedLimit) {
    std::string reason;
    EXPECT_TRUE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, 0.5, 1.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, 1.5, 1.0, reason));
    EXPECT_TRUE(validate_setpoint_numeric(NAVIGATE, 1, 2, 3, 5.0, 0.0, reason));  // 0 = off
}

TEST(ValidateSetpoint, PositionRejectsNonFinite) {
    std::string reason;
    EXPECT_TRUE(validate_setpoint_numeric(POSITION, 1, 2, 3, 0.0, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(POSITION, kNan, 2, 3, 0.0, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(POSITION, 1, kInf, 3, 0.0, 0.0, reason));
}

TEST(ValidateSetpoint, VelocityRejectsNonFiniteAndOverspeed) {
    std::string reason;
    EXPECT_TRUE(validate_setpoint_numeric(VELOCITY, 0.5, 0.5, 0.0, 0.0, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(VELOCITY, kNan, 0.5, 0.0, 0.0, 0.0, reason));
    EXPECT_FALSE(validate_setpoint_numeric(VELOCITY, 3.0, 4.0, 0.0, 0.0, 4.0, reason));  // |v|=5 > 4
    EXPECT_TRUE(validate_setpoint_numeric(VELOCITY, 3.0, 4.0, 0.0, 0.0, 6.0, reason));
}

// --- Diagnostics string mappings (prompt 101) ---

TEST(Mappings, SetpointTypeToString) {
    EXPECT_STREQ(setpoint_type_to_string(NONE), "NONE");
    EXPECT_STREQ(setpoint_type_to_string(NAVIGATE), "NAVIGATE");
    EXPECT_STREQ(setpoint_type_to_string(POSITION), "POSITION");
    EXPECT_STREQ(setpoint_type_to_string(EXCITE), "EXCITE");
}

TEST(Mappings, NavStateToString) {
    EXPECT_STREQ(nav_state_to_string(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD),
                 "OFFBOARD");
    EXPECT_STREQ(nav_state_to_string(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL),
                 "POSCTL");
    EXPECT_STREQ(nav_state_to_string(255), "UNKNOWN");
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
