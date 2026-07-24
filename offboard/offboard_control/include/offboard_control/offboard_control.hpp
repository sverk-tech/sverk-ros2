#ifndef OFFBOARD_CONTROL_HPP_
#define OFFBOARD_CONTROL_HPP_
#include "rclcpp/rclcpp.hpp"
#include "offboard_interfaces/srv/navigate.hpp"
#include "offboard_interfaces/srv/get_telemetry.hpp"
#include "offboard_interfaces/srv/set_altitude.hpp"
#include "offboard_interfaces/srv/set_yaw.hpp"
#include "offboard_interfaces/srv/set_yaw_rate.hpp"
#include "offboard_interfaces/srv/set_position.hpp"
#include "offboard_interfaces/srv/set_trajectory.hpp"
#include "offboard_interfaces/srv/set_velocity.hpp"
#include "offboard_interfaces/srv/set_attitude.hpp"
#include "offboard_interfaces/srv/set_rates.hpp"
#include "offboard_interfaces/srv/flip.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "offboard_interfaces/action/run_excitation.hpp"
#include "offboard_interfaces/action/receive_telemetry.hpp"
#include "offboard_interfaces/action/navigate_to.hpp"
#include "offboard_interfaces/srv/stop_excitation.hpp"
#include "offboard_interfaces/srv/emergency_stop.hpp"
#include "offboard_interfaces/srv/return_to_launch.hpp"
#include "offboard_interfaces/msg/excitation_sample.hpp"
#include "offboard_interfaces/msg/telemetry.hpp"
#include "offboard_interfaces/msg/offboard_status.hpp"
#include "offboard_interfaces/msg/trajectory_correction.hpp"
#include "offboard_control/excitation_signals.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <map>
#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>


using namespace std::chrono_literals;
using namespace px4_ros_com::frame_transforms;

class QoSConfigurator {
public:
    static rclcpp::QoS sensor_data() {
        return rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
    }
    static rclcpp::QoS system_status() {
        return rclcpp::QoS(rclcpp::KeepLast(5))
            .best_effort()
            .transient_local();
    }
    static rclcpp::QoS commands() {
        return rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
    }
};

struct CurrentPose {
    double x, y, z;
    double vx, vy, vz;
    double heading;
    uint64_t timestamp;
};

struct Pose {
    double x, y, z;
    double yaw;
    Pose(double x = 0.0, double y = 0.0, double z = 0.0, double yaw = 0.0)
        : x(x), y(y), z(z), yaw(yaw) {}
    Pose(CurrentPose pose)
        : x(pose.x), y(pose.y), z(pose.z), yaw(pose.heading) {}
};

enum SetpointType {
    NONE,
    NAVIGATE,
    NAVIGATE_GLOBAL,
    POSITION,
    VELOCITY,
    TRAJECTORY,
    ATTITUDE,
    RATES,
    FLIP,
    ALTITUDE,
    YAW,
    YAW_RATE,
    EXCITE,
};

// Human-readable name of a SetpointType (diagnostics topic, logs, tests).
inline const char * setpoint_type_to_string(SetpointType type) {
    switch (type) {
        case NONE:            return "NONE";
        case NAVIGATE:        return "NAVIGATE";
        case NAVIGATE_GLOBAL: return "NAVIGATE_GLOBAL";
        case POSITION:        return "POSITION";
        case VELOCITY:        return "VELOCITY";
        case TRAJECTORY:      return "TRAJECTORY";
        case ATTITUDE:        return "ATTITUDE";
        case RATES:           return "RATES";
        case FLIP:            return "FLIP";
        case ALTITUDE:        return "ALTITUDE";
        case YAW:             return "YAW";
        case YAW_RATE:        return "YAW_RATE";
        case EXCITE:          return "EXCITE";
    }
    return "UNKNOWN";
}

// PX4 nav_state to the mode string used by GetTelemetry/Telemetry/OffboardStatus.
inline const char * nav_state_to_string(uint8_t nav_state) {
    switch (nav_state) {
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:     return "OFFBOARD";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:    return "AUTO.LAND";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:  return "AUTO.LOITER";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "AUTO.MISSION";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:     return "AUTO.RTL";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "AUTO.TAKEOFF";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:       return "MANUAL";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:         return "ACRO";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:       return "ALTCTL";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:       return "POSCTL";
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:         return "STABILIZE";
        default:                                                          return "UNKNOWN";
    }
}

// Pure numeric part of the central setpoint validation (prompt 99) — a free
// function so it is unit-testable without a node instance (prompt 102). The
// node-side validate_setpoint() wraps it with the configured max_speed.
inline bool validate_setpoint_numeric(SetpointType kind, double x, double y, double z,
                                      double speed, double max_speed, std::string & reason)
{
    if (kind == NAVIGATE) {
        // Historical navigate checks, absorbed verbatim (partial NaN targets are
        // legal there: NaN = keep the current coordinate).
        if (std::isinf(x) || std::isinf(y) || std::isinf(z)) {
            reason = "x, y, z arguments cannot be Inf";
            return false;
        }
        if (std::isinf(speed)) {
            reason = "speed argument cannot be Inf";
            return false;
        }
        if (std::isfinite(x) != std::isfinite(y)) {
            reason = "x and y can be set only together";
            return false;
        }
        if (speed < 0) {
            reason = "Navigate speed must be positive";
            return false;
        }
        if (std::isnan(x) && std::isnan(y) && std::isnan(z)) {
            reason = "At least one of x, y, z must be finite";
            return false;
        }
        if (max_speed > 0.0 && speed > max_speed) {
            reason = "speed exceeds max_speed (" + std::to_string(max_speed) + " m/s)";
            return false;
        }
    } else if (kind == POSITION) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            reason = "x, y, z must be finite";
            return false;
        }
    } else if (kind == VELOCITY) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            reason = "vx, vy, vz must be finite";
            return false;
        }
        if (max_speed > 0.0 && std::sqrt(x * x + y * y + z * z) > max_speed) {
            reason = "velocity magnitude exceeds max_speed (" + std::to_string(max_speed) + " m/s)";
            return false;
        }
    }
    return true;
}

enum class FlipAxis { ROLL, PITCH, YAW };

enum class FlipPhase { CLIMB, ROTATE, POSITION_HOLD };

// Excitation maneuver phases ( / excitation context §4).
enum class ExcitePhase { SETTLE_IN, INJECT, SETTLE_OUT, REVERT };

class TransformPX4 {
public:
    using QuaternionArray = std::array<float, 4>;

    static double normalizeYaw(double yaw) {
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        return yaw;
    }

    static double interpolateYaw(double start_yaw, double end_yaw, double ratio) {
        double diff = std::atan2(std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));
        double interpolated_diff = diff * ratio;
        double result_yaw = start_yaw + interpolated_diff;
        return std::atan2(std::sin(result_yaw), std::cos(result_yaw));
    }

    static double getDistance(const Pose& from, const Pose& to) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double dz = to.z - from.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static Pose enuToNedPose(const Pose& enu_pose) {
        Eigen::Vector3d enu_vec(enu_pose.x, enu_pose.y, enu_pose.z);
        Eigen::Vector3d ned_vec = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_vec);
        return Pose(ned_vec.x(), ned_vec.y(), ned_vec.z(), enuToNedYaw(enu_pose.yaw));
    }

    static Pose nedToEnuPose(const Pose& ned_pose) {
        Eigen::Vector3d ned_vec(ned_pose.x, ned_pose.y, ned_pose.z);
        Eigen::Vector3d enu_vec = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_vec);
        return Pose(enu_vec.x(), enu_vec.y(), enu_vec.z(), nedToEnuYaw(ned_pose.yaw));
    }

    static double nedToEnuYaw(const double& ned_yaw) {
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(0.0, 0.0, ned_yaw);
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        return px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_enu);
    }

    static QuaternionArray nedToEnuQuan(const QuaternionArray& ned_quan) {
        // Array to Eigen quaternion
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(ned_quan);
        
        // NED to ENU quaternion
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        
        // Back to array
        QuaternionArray enu_quan;
        px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(q_enu, enu_quan);
        
        return enu_quan;
    }

    static QuaternionArray enuToNedQuan(const QuaternionArray& enu_quan) {
        // Array to Eigen quaternion
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(enu_quan);
        
        // ENU to NED quaternion
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        
        // Back to array
        QuaternionArray ned_quan;
        px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(q_ned, ned_quan);
        
        return ned_quan;
    }

    static double enuToNedYaw(const double& enu_yaw) {
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(0.0, 0.0, enu_yaw);
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        return px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned);
    }

    static CurrentPose enuToNedVel(const CurrentPose& enu_vel) {
        CurrentPose ned_vel;
        ned_vel.timestamp = enu_vel.timestamp;
        ned_vel.heading = enu_vel.heading;
        ned_vel.x = enu_vel.x;
        ned_vel.y = enu_vel.y;
        ned_vel.z = enu_vel.z;
        ned_vel.vx = enu_vel.vy;
        ned_vel.vy = enu_vel.vx;
        ned_vel.vz = -enu_vel.vz;
        return ned_vel;
    }

    static CurrentPose nedToEnuVel(const CurrentPose& ned_vel) {
        CurrentPose enu_vel;
        enu_vel.timestamp = ned_vel.timestamp;
        enu_vel.heading = ned_vel.heading;
        enu_vel.x = ned_vel.x;
        enu_vel.y = ned_vel.y;
        enu_vel.z = ned_vel.z;
        enu_vel.vx = ned_vel.vy;
        enu_vel.vy = ned_vel.vx;
        enu_vel.vz = -ned_vel.vz;
        return enu_vel;
    }
};

class SpeedController {
public:
    SpeedController(
        rclcpp::Logger logger,
        const Pose& start_pose_ned,
        const Pose& target_pose_any_frame,
        const std::string& frame_id,
        const std::string& map_frame_id,
        double speed,
        long long start_timestamp_micros,
        tf2_ros::Buffer& tf_buffer,
        double transform_timeout,
        std::optional<builtin_interfaces::msg::Time> tf_lookup_time_override = std::nullopt)
        : logger_(logger),
          start_pose_ned_(start_pose_ned),
          target_speed_(speed),
          start_timestamp_micros_(start_timestamp_micros),
          tf_buffer_(&tf_buffer),
          transform_timeout_(transform_timeout),
          current_frame_id_(frame_id),
          map_frame_id_(map_frame_id),
          tf_lookup_time_override_(tf_lookup_time_override)
    {
        // Transform target pose into map frame
        Pose target_pose_in_map_frame;
        if (!transform_to_map_frame(target_pose_any_frame, frame_id, target_pose_in_map_frame)) {
            RCLCPP_ERROR(logger_, "Failed to transform target pose from frame %s to map frame", frame_id.c_str());
            transformation_error_ = true;
            return;
        }
        // ENU to NED for internal math
        target_pose_ned_ = TransformPX4::enuToNedPose(target_pose_in_map_frame);
        distance_ = TransformPX4::getDistance(start_pose_ned_, target_pose_ned_);
        double signed_yaw_diff = std::atan2(std::sin(target_pose_ned_.yaw - start_pose_ned_.yaw),
                                           std::cos(target_pose_ned_.yaw - start_pose_ned_.yaw));
        yaw_difference_ = std::abs(signed_yaw_diff);
        if (distance_ == 0.0) {
            total_pos_time_micros_ = 0;
            RCLCPP_INFO(logger_, "SpeedController: Start and target positions are the same. Position time is 0 microseconds.");
        } else if (speed <= 0.0) {
            total_pos_time_micros_ = LLONG_MAX;
            RCLCPP_ERROR(logger_, "SpeedController: Target speed must be positive. Got %f.", speed);
        } else {
            double total_pos_time_seconds = distance_ / speed;
            total_pos_time_micros_ = static_cast<long long>(total_pos_time_seconds * 1e6);
        }
        // Navigate: yaw applied immediately (no angular ramp).
        total_yaw_time_micros_ = 0;
    }

    bool transform_to_map_frame(
        const Pose& source_pose,
        const std::string& source_frame,
        Pose& target_pose) 
    {
        if (source_frame == map_frame_id_) {
            target_pose = source_pose;
            return true;
        }
        geometry_msgs::msg::PoseStamped source_pose_msg;
        source_pose_msg.header.frame_id = source_frame;
        source_pose_msg.header.stamp = tf_lookup_time_override_ ? *tf_lookup_time_override_ : this->now();
        source_pose_msg.pose.position.x = source_pose.x;
        source_pose_msg.pose.position.y = source_pose.y;
        source_pose_msg.pose.position.z = source_pose.z;
        tf2::Quaternion q;
        q.setRPY(0, 0, source_pose.yaw);
        source_pose_msg.pose.orientation = tf2::toMsg(q);
        try {
            geometry_msgs::msg::PoseStamped target_pose_msg = 
                tf_buffer_->transform(source_pose_msg, map_frame_id_, 
                                    tf2::durationFromSec(transform_timeout_));
            target_pose.x = target_pose_msg.pose.position.x;
            target_pose.y = target_pose_msg.pose.position.y;
            target_pose.z = target_pose_msg.pose.position.z;
            tf2::Quaternion q_out;
            tf2::fromMsg(target_pose_msg.pose.orientation, q_out);
            target_pose.yaw = tf2::impl::getYaw(q_out);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
            transformation_error_ = true;
            return false;
        }
    }

    px4_msgs::msg::TrajectorySetpoint getCurrentSetpointMSG_NED(long long current_timestamp_micros) const {
        Pose setpoint_NED = getCurrentSetpoint_NED(current_timestamp_micros);
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = current_timestamp_micros;
        msg.position[0] = setpoint_NED.x;
        msg.position[1] = setpoint_NED.y;
        msg.position[2] = setpoint_NED.z;
        msg.yaw = setpoint_NED.yaw;
        msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
        return msg;
    }

    Pose getCurrentSetpoint_NED(long long current_timestamp_micros) const {
        Pose result_pose_ned = start_pose_ned_;
        if (total_pos_time_micros_ == LLONG_MAX) {
            RCLCPP_ERROR(logger_, "SpeedController: Error in setup (pos speed <= 0), returning start pos.");
        } else if (total_pos_time_micros_ > 0) {
            long long elapsed_pos_time_micros = current_timestamp_micros - start_timestamp_micros_;
            double pos_ratio = std::min(static_cast<double>(elapsed_pos_time_micros) / static_cast<double>(total_pos_time_micros_), 1.0);
            result_pose_ned.x = start_pose_ned_.x + (target_pose_ned_.x - start_pose_ned_.x) * pos_ratio;
            result_pose_ned.y = start_pose_ned_.y + (target_pose_ned_.y - start_pose_ned_.y) * pos_ratio;
            result_pose_ned.z = start_pose_ned_.z + (target_pose_ned_.z - start_pose_ned_.z) * pos_ratio;
        } else {
            result_pose_ned.x = target_pose_ned_.x;
            result_pose_ned.y = target_pose_ned_.y;
            result_pose_ned.z = target_pose_ned_.z;
        }
        result_pose_ned.yaw = target_pose_ned_.yaw;
        return result_pose_ned;
    }

    bool isEnd(const Pose& current_drone_pose_ned, double position_tolerance = 0.08, double yaw_tolerance = 0.08) const {
        if (transformation_error_) {
            return true;
        }
        // Complete only at target (position + yaw), not by timer — avoids early exit/overshoot;
        // slow drone may take longer than the nominal motion time.
        double distance_to_target = TransformPX4::getDistance(current_drone_pose_ned, target_pose_ned_);
        bool position_reached = distance_to_target <= position_tolerance;
        double yaw_diff = current_drone_pose_ned.yaw - target_pose_ned_.yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        bool yaw_reached = std::abs(yaw_diff) <= yaw_tolerance;
        return position_reached && yaw_reached;
    }

    bool isFinished(long long current_timestamp_micros) const {
        if (total_pos_time_micros_ == LLONG_MAX || total_yaw_time_micros_ == LLONG_MAX) {
             return true;
        }
        long long max_required_time = std::max(total_pos_time_micros_, total_yaw_time_micros_);
        return (current_timestamp_micros - start_timestamp_micros_) >= max_required_time;
    }

    bool hasTransformationError() const {
        return transformation_error_;
    }

    Pose targetPoseNED() const {
        return target_pose_ned_;
    }

    double targetSpeed() const {
        return target_speed_;
    }

    builtin_interfaces::msg::Time now() const {
        auto now = std::chrono::system_clock::now();
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto epoch = now_ns.time_since_epoch();
        uint64_t nanoseconds = epoch.count();
        builtin_interfaces::msg::Time time_msg;
        time_msg.sec = static_cast<int32_t>(nanoseconds / 1000000000);
        time_msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000);
        return time_msg;
    }

private:
    rclcpp::Logger logger_;
    Pose start_pose_ned_;
    Pose target_pose_ned_;
    double target_speed_;
    long long start_timestamp_micros_;
    double distance_;
    double yaw_difference_;
    long long total_pos_time_micros_;
    long long total_yaw_time_micros_;
    tf2_ros::Buffer* tf_buffer_;
    double transform_timeout_;
    std::string current_frame_id_;
    std::string map_frame_id_;
    std::optional<builtin_interfaces::msg::Time> tf_lookup_time_override_;
    mutable bool transformation_error_ = false;
};

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode(std::string px4_namespace);
    ~OffboardControlNode();
private:
    // --- TF ---
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped current_odom_transform_;
    geometry_msgs::msg::TransformStamped body_transform_;
    geometry_msgs::msg::TransformStamped target_transform_;
    geometry_msgs::msg::TransformStamped setpoint_transform_;
    geometry_msgs::msg::TransformStamped terrain_transform_;
    
    // --- Reference frames ---
    std::map<std::string, std::string> reference_frames_;

    // --- Service servers ---
    rclcpp::Service<offboard_interfaces::srv::Navigate>::SharedPtr navigate_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
    rclcpp::Service<offboard_interfaces::srv::GetTelemetry>::SharedPtr get_telemetry_service_;
    rclcpp::Service<offboard_interfaces::srv::SetAltitude>::SharedPtr set_altitude_service_;
    rclcpp::Service<offboard_interfaces::srv::SetYaw>::SharedPtr set_yaw_service_;
    rclcpp::Service<offboard_interfaces::srv::SetYawRate>::SharedPtr set_yaw_rate_service_;
    rclcpp::Service<offboard_interfaces::srv::SetPosition>::SharedPtr set_position_service_;
    rclcpp::Service<offboard_interfaces::srv::SetTrajectory>::SharedPtr set_trajectory_service_;
    rclcpp::Service<offboard_interfaces::srv::SetVelocity>::SharedPtr set_velocity_service_;
    rclcpp::Service<offboard_interfaces::srv::SetAttitude>::SharedPtr set_attitude_service_;
    rclcpp::Service<offboard_interfaces::srv::SetRates>::SharedPtr set_rates_service_;
    rclcpp::Service<offboard_interfaces::srv::Flip>::SharedPtr flip_service_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
    // Excitation reference r(t) for the analyzer — NOT a /fmu/in topic.
    rclcpp::Publisher<offboard_interfaces::msg::ExcitationSample>::SharedPtr excitation_reference_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    // --- Subscribers ---
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_pos_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subscriber_;
    // PX4 hover-thrust estimate (P0-1): the excitation baseline reads this at target
    // start instead of the excite_hover_thrust constant, so Pattern B enters at the
    // real hover throttle even as the battery sags over a session.
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_subscriber_;
    // Land detector (P0-2): ground-guard for the excitation safety check — a biased
    // EKF once let the drone "excite" a full chirp while sitting on the ground.
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscriber_;
    rclcpp::Subscription<offboard_interfaces::msg::TrajectoryCorrection>::SharedPtr trajectory_correction_subscriber_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    // --- Frame parameters ---
    std::string body_frame_id_;
    std::string map_frame_id_;
    std::string aruco_map_frame_id_;

    // --- Parameters ---
    double transform_timeout_;
    double offboard_timeout_;
    double arming_timeout_;
    double default_speed_;
    bool land_only_in_offboard_;
    bool check_kill_switch_;
    double local_position_timeout_;
    double global_position_timeout_;
    double battery_timeout_;
    double manual_control_timeout_;
    bool is_simulator_;
    // RC interception (prompt 95): in the simulator the RC gate stays off unless
    // enforce_rc_in_sim=true (SITL with a joystick); rc_stick_abort_thresh is the
    // stick deflection that returns control to the pilot during any maneuver.
    bool enforce_rc_in_sim_ = false;
    double rc_stick_abort_thresh_ = 0.05;
    // Freshness watchdog (prompt 98): when true, a stale local position during
    // an offboard position-dependent maneuver parks the drone (auto-hold on the
    // last valid pose). Set false only for debugging.
    bool enforce_freshness_ = true;
    // Geofence / limits (prompt 99). Disabled by default: geofence_enable=false
    // and max_speed=0 keep the previous behavior exactly. Center/altitude are in
    // the map (ENU) frame; the excitation keeps its own start-relative fence.
    bool geofence_enable_ = false;
    double geofence_radius_ = 10.0;
    double geofence_center_x_ = 0.0;
    double geofence_center_y_ = 0.0;
    double alt_floor_ = 0.0;
    double alt_ceil_ = 50.0;
    double max_speed_ = 0.0;  // 0 = no speed limit
    // Atomic: mutated by the excitation execute() thread (raises the stream to
    // excite_stream_hz on the window, restores after) and read on the timer thread.
    std::atomic<double> control_rate_{30.0};

    // --- State ---
    // Guards the telemetry snapshot below (current_vehicle_status_, battery,
    // global/local position, manual control, angular velocity, attitude quat and
    // their timestamps). Subscribers take it briefly to write; readers on other
    // callback groups (services, excitation) copy into locals under the lock and
    // work on the copy. TF publishing / transforms happen OUTSIDE the lock.
    // LOCK ORDER: telemetry_mutex_ before cmd_mutex_ when both are needed; never
    // hold either together with excite_mutex_.
    std::mutex telemetry_mutex_;
    px4_msgs::msg::VehicleStatus current_vehicle_status_;
    uint64_t current_vehicle_status_timestamp_;
    float battery_voltage;
    float battery_percentage;
    uint64_t current_battery_timestamp_;
    double lat_{0.0}, lon_{0.0}, alt_{0.0};
    uint64_t current_global_position_timestamp_;
    px4_msgs::msg::ManualControlSetpoint current_manual_control_;
    uint64_t current_manual_control_timestamp_;

    // Guards the commanded-setpoint block: setpoint_type_, position/velocity/
    // attitude/rates targets, overrides, flip_* setup, last_sended_pose and the
    // global_speed_controller_ pointer. Writers (service callbacks) store the
    // target fields first and setpoint_type_ LAST under one lock, so the timer
    // either sees the old type with the old fields or the new type with the new
    // ones. The timer copies everything into a CmdSnapshot at the top of
    // publish_setpoint() and publishes from the copy (no TF/publish under lock).
    // LOCK ORDER: never hold cmd_mutex_ and excite_mutex_ together — the EXCITE
    // path keeps its own synchronization (excite_mutex_ + atomics) untouched.
    std::mutex cmd_mutex_;

    SetpointType setpoint_type_ = SetpointType::NONE;
    CurrentPose current_local_position_;
    px4_msgs::msg::TrajectorySetpoint last_sended_pose;
    // Last safe-fallback reason (park_to_hold/RC abort/watchdog) — for ~/status.
    std::string last_abort_reason_;

    // Overrides for NAVIGATE / POSITION: altitude, yaw, yaw_rate
    bool altitude_override_set_ = false;
    double altitude_override_z_ = 0.0;
    std::string altitude_override_frame_id_;
    bool yaw_override_set_ = false;
    double yaw_override_ = 0.0;
    std::string yaw_override_frame_id_;
    bool yaw_rate_override_set_ = false;
    double yaw_rate_override_ = 0.0;

    // Direct setpoint state for POSITION / VELOCITY
    double position_target_x_ = 0.0, position_target_y_ = 0.0, position_target_z_ = 0.0, position_target_yaw_ = 0.0;
    std::string position_target_frame_id_;
    double velocity_target_vx_ = 0.0, velocity_target_vy_ = 0.0, velocity_target_vz_ = 0.0, velocity_target_yaw_ = 0.0;
    std::string velocity_target_frame_id_;

    // Trajectory setpoint state (TRAJECTORY): position / velocity / acceleration
    // are each an all-finite or all-NaN group (NaN = group off). All expressed in
    // trajectory_frame_id_. Written under cmd_mutex_ (target fields first, then
    // setpoint_type_) and copied into the CmdSnapshot on the timer tick.
    std::array<double, 3> trajectory_position_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 3> trajectory_velocity_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 3> trajectory_acceleration_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double trajectory_yaw_ = std::numeric_limits<double>::quiet_NaN();
    double trajectory_yaw_rate_ = std::numeric_limits<double>::quiet_NaN();
    std::string trajectory_frame_id_;

    // High-rate feed-forward correction stream (~/trajectory_correction) layered
    // on top of the base TRAJECTORY target. Each fresh correction group overrides
    // the corresponding service group per tick; a stale stream (older than
    // correction_timeout_) is dropped so the drone falls back to the base target.
    // Guarded by corr_mutex_ (its own lock: written from the high-rate subscriber
    // callback, read once per tick into the CmdSnapshot). Never nested with
    // cmd_mutex_/telemetry_mutex_ — locked separately in snapshot_cmd_state().
    std::mutex corr_mutex_;
    std::array<double, 3> correction_velocity_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 3> correction_acceleration_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double correction_yaw_rate_ = std::numeric_limits<double>::quiet_NaN();
    std::string correction_frame_id_;
    // Node-clock receive time of the last correction (ns; 0 = never). Node-side
    // stamp avoids publisher/node clock-domain mismatch in the freshness check.
    int64_t correction_recv_ns_ = 0;
    double correction_timeout_ = 0.5;  // s; <=0 disables the correction stream

    // Attitude / rates setpoint state
    float attitude_roll_ = 0.0f, attitude_pitch_ = 0.0f, attitude_yaw_ = 0.0f, attitude_thrust_ = 0.5f;
    std::string attitude_frame_id_;
    float rates_roll_ = 0.0f, rates_pitch_ = 0.0f, rates_yaw_ = 0.0f, rates_thrust_ = 0.5f;

    // --- Flip state ---
    FlipPhase flip_phase_ = FlipPhase::CLIMB;
    Pose flip_initial_pose_ned_;
    FlipAxis flip_axis_ = FlipAxis::ROLL;
    float flip_vz_ = 1.0f;
    float flip_climb_duration_ = 0.0f;
    float flip_rate_ = 0.0f;
    float flip_target_angle_ = 0.0f;
    float flip_thrust_ = 0.5f;
    rclcpp::Time flip_climb_start_time_;
    rclcpp::Time flip_rotate_start_time_;
    double flip_angle_done_ = 0.0;

    // --- Excitation (system-ID) state ---
    // Action + emergency-stop service + gyro. NOT read-only: publishes setpoints.
    rclcpp_action::Server<offboard_interfaces::action::RunExcitation>::SharedPtr excitation_action_server_;
    rclcpp::Service<offboard_interfaces::srv::StopExcitation>::SharedPtr stop_excitation_service_;
    rclcpp::CallbackGroup::SharedPtr excite_callback_group_;

    // Config parameters (safe, conservative defaults — see excitation context §4).
    double excite_stream_hz_ = 200.0;        // high-rate setpoint stream on the window
    double excite_settle_in_s_ = 1.5;        // hover-stabilize before injecting
    double excite_settle_out_s_ = 1.5;       // return to trim after injecting
    double excite_max_inject_s_ = 20.0;      // per-axis injection dead-man
    double excite_max_session_s_ = 90.0;     // whole-maneuver dead-man
    bool allow_rate_loop_excitation_ = false; // Pattern B gate (advanced, off by default)
    double excite_geofence_radius_default_ = 2.0; // m from start
    double excite_agl_floor_ = 1.0;          // m, minimum altitude above start
    double excite_agl_ceil_ = 4.0;           // m, maximum altitude above start
    double excite_kp_pos_ = 0.04;            // rad tilt per m horizontal error (small)
    double excite_kp_alt_ = 0.05;            // thrust per m altitude error
    double excite_kd_vel_ = 0.02;            // rad tilt per m/s horizontal velocity (drift damping, P0-4)
    double excite_max_baseline_tilt_ = 0.087; // rad (~5 deg) cap on hover-loop tilt
    double excite_hover_thrust_ = 0.5;       // baseline hover throttle [0..1] (param fallback for P0-1)
    double excite_stick_abort_thresh_ = 0.05; // RC stick soft-abort deflection
    double excite_settle_rate_thresh_ = 0.30;  // rad/s — settle_in exit gate (P2-9)
    double excite_settle_vel_thresh_ = 0.30;   // m/s — settle_in exit gate (P2-9)

    // Runtime state (mutated by timer + action threads).
    ExcitePhase excite_phase_ = ExcitePhase::SETTLE_IN;
    FlipAxis excite_axis_ = FlipAxis::ROLL;
    bool excite_rate_loop_ = false;          // false = Pattern A (attitude)
    std::unique_ptr<excitation::ExcitationSignal> excite_signal_;
    Pose excite_initial_pose_ned_;
    double excite_hold_alt_m_ = 0.0;         // desired altitude above start, m (+up)
    double excite_hover_thrust_used_ = 0.5;  // per-run baseline throttle: PX4 estimate at start else param (P0-1)
    double excite_start_yaw_ned_ = 0.0;
    double excite_max_tilt_rad_ = 0.35;      // hard total-tilt cap from goal
    double excite_geofence_radius_ = 2.0;
    double excite_amplitude_ = 0.0;
    rclcpp::Time excite_session_start_;
    rclcpp::Time excite_phase_start_;
    rclcpp::Time excite_inject_start_;
    double excite_washout_state_ = 0.0;
    double excite_prev_control_rate_ = 30.0; // rate to restore after the window
    std::string excite_signal_str_;          // waveform name for ExcitationSample
    double excite_bl_roll_ = 0.0;            // baseline hover tilt (rad) — for logging
    double excite_bl_pitch_ = 0.0;
    Eigen::Quaterniond current_q_ned_body_{1.0, 0.0, 0.0, 0.0};
    float current_angular_velocity_[3] = {0.0f, 0.0f, 0.0f};
    uint64_t current_angular_velocity_timestamp_ = 0;
    // PX4 hover-thrust estimate (guarded by telemetry_mutex_). valid==false until the
    // estimator converges; the excitation baseline only trusts a fresh, valid sample.
    double current_hover_thrust_ = 0.0;
    bool current_hover_thrust_valid_ = false;
    uint64_t current_hover_thrust_timestamp_ = 0;
    // Land-detector snapshot (guarded by telemetry_mutex_).
    bool current_landed_ = false;
    uint64_t current_landed_timestamp_ = 0;

    // Metrics + feedback snapshot (guarded by excite_mutex_).
    std::mutex excite_mutex_;
    double excite_max_tilt_seen_deg_ = 0.0;
    double excite_max_drift_m_ = 0.0;
    uint32_t excite_samples_injected_ = 0;
    double excite_fb_elapsed_ = 0.0;
    double excite_fb_progress_ = 0.0;
    std::string excite_fb_phase_;
    double excite_fb_ref_ = 0.0;
    double excite_fb_rate_ = 0.0;
    double excite_fb_tilt_ = 0.0;
    double excite_fb_drift_ = 0.0;
    std::string excite_abort_reason_;
    std::string excite_run_tag_;
    double excite_achieved_rate_hz_ = 0.0;

    // Cross-thread flags.
    std::atomic<bool> excite_active_{false};
    // Release/acquire gate: set true (release) by execute() only after excite_signal_
    // and all EXCITE state are built, checked (acquire) at the top of the timer-side
    // phase machine so a weakly-ordered CPU never runs it on half-published state.
    std::atomic<bool> excite_ready_{false};
    std::atomic<bool> excite_finished_{false};
    std::atomic<bool> excite_abort_{false};
    std::atomic<bool> excite_emergency_{false};
    std::atomic<bool> excite_cancel_{false};
    std::atomic<bool> excite_soft_stop_{false};
    std::thread excite_thread_;

    // --- Speed controller ---
    // shared_ptr (not unique_ptr): the timer copies it into its CmdSnapshot under
    // cmd_mutex_ and keeps the controller alive for the tick even if a service
    // swaps/resets the member concurrently.
    std::shared_ptr<SpeedController> global_speed_controller_;

    // Local copy of the commanded state, taken under cmd_mutex_ once per timer
    // tick (see publish_setpoint). FLIP runtime state (flip_phase_ etc.) is not
    // copied: after the service hands it off it is only mutated on the timer.
    struct CmdSnapshot {
        SetpointType type = SetpointType::NONE;
        std::shared_ptr<SpeedController> nav_controller;
        px4_msgs::msg::TrajectorySetpoint last_pose;
        bool altitude_override_set = false;
        double altitude_override_z = 0.0;
        std::string altitude_override_frame_id;
        bool yaw_override_set = false;
        double yaw_override = 0.0;
        std::string yaw_override_frame_id;
        bool yaw_rate_override_set = false;
        double yaw_rate_override = 0.0;
        double position_x = 0.0, position_y = 0.0, position_z = 0.0, position_yaw = 0.0;
        std::string position_frame_id;
        double velocity_vx = 0.0, velocity_vy = 0.0, velocity_vz = 0.0, velocity_yaw = 0.0;
        std::string velocity_frame_id;
        std::array<double, 3> trajectory_position{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
        std::array<double, 3> trajectory_velocity{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
        std::array<double, 3> trajectory_acceleration{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
        double trajectory_yaw = std::numeric_limits<double>::quiet_NaN();
        double trajectory_yaw_rate = std::numeric_limits<double>::quiet_NaN();
        std::string trajectory_frame_id;
        // Feed-forward correction resolved for this tick (fresh == within timeout).
        bool correction_fresh = false;
        std::array<double, 3> correction_velocity{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
        std::array<double, 3> correction_acceleration{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
        double correction_yaw_rate = std::numeric_limits<double>::quiet_NaN();
        std::string correction_frame_id;
        float attitude_roll = 0.0f, attitude_pitch = 0.0f, attitude_yaw = 0.0f, attitude_thrust = 0.5f;
        std::string attitude_frame_id;
        float rates_roll = 0.0f, rates_pitch = 0.0f, rates_yaw = 0.0f, rates_thrust = 0.5f;
    };
    CmdSnapshot snapshot_cmd_state();

    // --- Flags ---
    // busy_ is written from the service group and the excitation handlers
    // concurrently — atomic so the mutual-exclusion check is not a data race.
    std::atomic<bool> busy_{false};
    bool wait_armed_ = false;

    // Short-lock copies of the telemetry snapshot (see telemetry_mutex_).
    CurrentPose snapshot_local_position();
    px4_msgs::msg::VehicleStatus snapshot_vehicle_status(uint64_t * timestamp = nullptr);

    // --- Telemetry streaming (prompt 96) ---
    // Both the ~/telemetry topic and the ReceiveTelemetry action reuse
    // build_telemetry(), the same code path as the get_telemetry service.
    offboard_interfaces::msg::Telemetry build_telemetry(const std::string & requested_frame_id);
    rclcpp::Publisher<offboard_interfaces::msg::Telemetry>::SharedPtr telemetry_publisher_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::CallbackGroup::SharedPtr telemetry_callback_group_;  // own group: never blocks setpoint_timer_
    double telemetry_rate_hz_ = 0.0;  // 0 = topic disabled (default behavior unchanged)
    void publish_telemetry();
    void apply_telemetry_rate(double hz);
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter> & parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    using ReceiveTelemetry = offboard_interfaces::action::ReceiveTelemetry;
    using TelemetryGoalHandle = rclcpp_action::ServerGoalHandle<ReceiveTelemetry>;
    rclcpp_action::Server<ReceiveTelemetry>::SharedPtr receive_telemetry_action_server_;
    std::atomic<bool> telemetry_stream_active_{false};
    std::atomic<bool> telemetry_stream_stop_{false};
    std::thread telemetry_stream_thread_;
    rclcpp_action::GoalResponse handle_telemetry_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ReceiveTelemetry::Goal> goal);
    rclcpp_action::CancelResponse handle_telemetry_cancel(
        const std::shared_ptr<TelemetryGoalHandle> goal_handle);
    void handle_telemetry_accepted(const std::shared_ptr<TelemetryGoalHandle> goal_handle);
    void execute_telemetry_stream(const std::shared_ptr<TelemetryGoalHandle> goal_handle);

    // --- Safe fallback (prompts 95/97/98/99/100 share this) ---
    // Parks the drone: position-hold at the current pose, drops busy_.
    void park_to_hold(const std::string & reason);

    // --- RTL + failsafe policy (prompt 104) ---
    // onboard_failsafe_action selects what the NODE-side critical fallbacks
    // (freshness/geofence watchdogs, RC-loss) do: "hold" (default, park at the
    // current pose), "land" or "rtl". Pilot takeover (kill-switch/stick) always
    // releases the stream to PX4 instead — see failsafe_fallback().
    std::string onboard_failsafe_action_ = "hold";
    // Additive land relaxation: when true, the land service is also allowed
    // from hold-like modes (AUTO.LOITER/POSCTL/ALTCTL), not only OFFBOARD.
    bool land_from_hold_ = false;
    rclcpp::Service<offboard_interfaces::srv::ReturnToLaunch>::SharedPtr return_to_launch_service_;
    void return_to_launch_callback(
        const offboard_interfaces::srv::ReturnToLaunch::Request::SharedPtr request,
        const offboard_interfaces::srv::ReturnToLaunch::Response::SharedPtr response);
    // Critical-rollback dispatcher: routes to park_to_hold / land / RTL
    // according to onboard_failsafe_action.
    void failsafe_fallback(const std::string & reason);

    // --- Universal emergency stop (prompt 100) ---
    // Global abort flag: checked by the CV waits in offboardAndArm so an
    // emergency stop interrupts even an in-flight arm/offboard sequence.
    std::atomic<bool> command_abort_{false};
    rclcpp::Service<offboard_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    void emergency_stop_callback(
        const offboard_interfaces::srv::EmergencyStop::Request::SharedPtr request,
        const offboard_interfaces::srv::EmergencyStop::Response::SharedPtr response);

    // --- Central setpoint validation (prompt 99) ---
    // Numeric checks (NaN/Inf, speed limits) shared by navigate/NavigateTo/
    // set_position/set_velocity. Absorbs the historical navigate checks verbatim.
    bool validate_setpoint(SetpointType kind, double x, double y, double z,
                           double speed, std::string & reason);
    // Optional geofence gate for a position target given in `frame_id` (ENU).
    // No-op (true) while geofence_enable=false.
    bool validate_target_in_geofence(double x, double y, double z,
                                     const std::string & frame_id, std::string & reason);

    // --- NavigateTo action (prompt 97) ---
    // Shared launch logic of the navigate service (thin wrapper) and the
    // NavigateTo action. Throws std::runtime_error on failure; returns the
    // controller driving this maneuver.
    std::shared_ptr<SpeedController> start_navigate(
        float x, float y, float z, float yaw, float speed,
        const std::string & frame_id, bool auto_arm);
    using NavigateTo = offboard_interfaces::action::NavigateTo;
    using NavigateToGoalHandle = rclcpp_action::ServerGoalHandle<NavigateTo>;
    rclcpp_action::Server<NavigateTo>::SharedPtr navigate_to_action_server_;
    rclcpp::CallbackGroup::SharedPtr nav_callback_group_;
    std::atomic<bool> nav_action_active_{false};
    std::atomic<bool> nav_action_stop_{false};
    std::thread nav_action_thread_;
    // --- Diagnostics topic (prompt 101) ---
    rclcpp::Publisher<offboard_interfaces::msg::OffboardStatus>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    void publish_status();

    rclcpp_action::GoalResponse handle_navigate_to_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateTo::Goal> goal);
    rclcpp_action::CancelResponse handle_navigate_to_cancel(
        const std::shared_ptr<NavigateToGoalHandle> goal_handle);
    void handle_navigate_to_accepted(const std::shared_ptr<NavigateToGoalHandle> goal_handle);
    void execute_navigate_to(const std::shared_ptr<NavigateToGoalHandle> goal_handle);

    // Reference yaw at first odometry (so reported/display yaw is 0 at startup).
    // Gated by zero_heading_at_start (default: true in sim, false on the real
    // drone — the historical behavior). The offset applies ONLY to the
    // displayed/telemetry yaw and logs; the control-side NED heading
    // (current_local_position_.heading) always stays raw.
    bool zero_heading_at_start_ = false;
    bool initial_heading_set_ = false;
    double initial_heading_ned_ = 0.0;
    static double normalizeYaw(double yaw);

    // Callback groups for concurrent execution
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    std::shared_ptr<px4_msgs::msg::VehicleStatus> prev_vehicle_status_msg_;

    // --- TF init ---
    void create_reference_frames();
    void init_tf_frames();

    // --- TF helpers ---
    bool wait_for_transform(
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time,
        const rclcpp::Duration& timeout);
    
    std::string get_reference_frame(const std::string& frame_id);
    
    // --- Generic transforms ---
    bool transform_point(
        const geometry_msgs::msg::Point& point,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        geometry_msgs::msg::Point& result);
    
    bool transform_yaw(
        double yaw,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        double& result_yaw);
    
    bool transform_velocity(
        const geometry_msgs::msg::Vector3& velocity,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        geometry_msgs::msg::Vector3& result);

    // --- Publish TF ---
    void clear_navigation_targets();
    void publish_static_transforms();
    void publish_setpoint_frame(const Pose& setpoint_pose_ned, const rclcpp::Time& stamp);
    void publish_terrain_frame(double altitude_above_ground);
    void publish_odometry_transform(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publish_target_frame(
        const Pose& target_pose, 
        const std::string& frame_id,
        const rclcpp::Time& stamp);

    // --- Helpers ---
    void Init_publishers(std::string px4_namespace);
    void Init_subscribers(std::string px4_namespace);
    void Init_services();
    void Init_actions();
    void Init_timer();
    void Init_SpeedController(
        const Pose& start_pose_ned, 
        const Pose& target_pose_any_frame,
        const std::string& frame_id,
        double speed);
    bool validate_frame_id(const std::string& frame_id);
    bool get_altitude_ned_from_frame(double z, const std::string& frame_id, double& out_ned_z);
    bool transform_yaw_to_ned(double yaw, const std::string& frame_id, double& out_ned_yaw);
    /** Time for TF lookups: latest (0) in simulator to avoid extrapolation; now() on real drone. */
    rclcpp::Time get_transform_lookup_time() const;
    /** When source or target is aruco_map (tf_static), use time 0 so static transform is found. */
    rclcpp::Time get_transform_lookup_time(const std::string& source_frame, const std::string& target_frame) const;

    // --- Control ---
    void checkManualControl();
    bool offboardAndArm();
    // Shared RC-interception check (kill-switch, RC-loss timeout, stick
    // deflection) used by the control loop, offboardAndArm and the excitation
    // abort. stick_thresh <= 0 disables the stick check. Returns true when the
    // pilot demands control; `reason` is a short tag for logs.
    bool rc_should_abort(std::string & reason, double stick_thresh);
    // Parks the calling (service/worker) thread on arm_wait_cv_ until the
    // vehicle status satisfies `predicate` or `timeout_s` expires — no polling
    // loop, no setpoint publishing from the waiting thread.
    bool wait_for_vehicle_state(
        const std::function<bool(const px4_msgs::msg::VehicleStatus &)> & predicate,
        double timeout_s);
    // vehicleStatusCallback notifies this CV on every status update.
    std::mutex arm_wait_mutex_;
    std::condition_variable arm_wait_cv_;

    // --- PX4 commands ---
    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void switch_to_offboard_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    bool land();
    void publish_idle_setpoint();

    // --- Service callbacks ---
    void navigate_callback(
        const offboard_interfaces::srv::Navigate::Request::SharedPtr request,
                           const offboard_interfaces::srv::Navigate::Response::SharedPtr response);
    void land_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
                       const std_srvs::srv::Trigger::Response::SharedPtr response);
    void get_telemetry_callback(
        const offboard_interfaces::srv::GetTelemetry::Request::SharedPtr request,
                                const offboard_interfaces::srv::GetTelemetry::Response::SharedPtr response);
    void set_altitude_callback(const offboard_interfaces::srv::SetAltitude::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetAltitude::Response::SharedPtr response);
    void set_yaw_callback(const offboard_interfaces::srv::SetYaw::Request::SharedPtr request,
                          const offboard_interfaces::srv::SetYaw::Response::SharedPtr response);
    void set_yaw_rate_callback(const offboard_interfaces::srv::SetYawRate::Request::SharedPtr request,
                               const offboard_interfaces::srv::SetYawRate::Response::SharedPtr response);
    void set_position_callback(const offboard_interfaces::srv::SetPosition::Request::SharedPtr request,
                               const offboard_interfaces::srv::SetPosition::Response::SharedPtr response);
    void set_trajectory_callback(const offboard_interfaces::srv::SetTrajectory::Request::SharedPtr request,
                                 const offboard_interfaces::srv::SetTrajectory::Response::SharedPtr response);
    void set_velocity_callback(const offboard_interfaces::srv::SetVelocity::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetVelocity::Response::SharedPtr response);
    void set_attitude_callback(const offboard_interfaces::srv::SetAttitude::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetAttitude::Response::SharedPtr response);
    void set_rates_callback(const offboard_interfaces::srv::SetRates::Request::SharedPtr request,
                           const offboard_interfaces::srv::SetRates::Response::SharedPtr response);
    void flip_callback(const offboard_interfaces::srv::Flip::Request::SharedPtr request,
                       const offboard_interfaces::srv::Flip::Response::SharedPtr response);

    // --- Excitation action / service ---
    using RunExcitation = offboard_interfaces::action::RunExcitation;
    using ExciteGoalHandle = rclcpp_action::ServerGoalHandle<RunExcitation>;
    rclcpp_action::GoalResponse handle_excite_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RunExcitation::Goal> goal);
    rclcpp_action::CancelResponse handle_excite_cancel(
        const std::shared_ptr<ExciteGoalHandle> goal_handle);
    void handle_excite_accepted(const std::shared_ptr<ExciteGoalHandle> goal_handle);
    void execute_excitation(const std::shared_ptr<ExciteGoalHandle> goal_handle);
    void stop_excitation_callback(
        const offboard_interfaces::srv::StopExcitation::Request::SharedPtr request,
        const offboard_interfaces::srv::StopExcitation::Response::SharedPtr response);
    // Timer-side phase machine + helpers.
    void publish_excite_setpoint();
    bool excite_compute_baseline(Eigen::Quaterniond & q_hover, float & thrust_cmd,
                                 double & tilt_deg, double & drift_m);
    void excite_publish_attitude(const Eigen::Quaterniond & q_ned, float thrust_cmd);
    void excite_publish_rates(double wx, double wy, double wz, float thrust_cmd);
    // settle_grace suppresses the attitude-transient aborts (runaway-rate, tilt)
    // while the drone is still settling at the start of a goal — otherwise the
    // residual motion from the previous goal's REVERT trips a false abort (P2-9).
    bool excite_safety_ok(std::string & reason, bool settle_grace = false);
    bool excite_is_stationary();  // rates + horizontal velocity below settle thresholds
    bool excite_rc_abort();
    void request_excite_abort(const std::string & reason, bool emergency);

    // --- Topic callbacks ---
    void batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
    void hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);
    void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void trajectoryCorrectionCallback(const offboard_interfaces::msg::TrajectoryCorrection::SharedPtr msg);

    // --- Main loop helpers ---
    void publish_setpoint();
    void stopOffboardHeartbeat();
    void startOffboardHeartbeat();

    // --- Utils ---
    uint64_t getTimestamp();
    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    bool service_done_;
    uint8_t service_result_;
};

#endif // OFFBOARD_CONTROL_HPP_