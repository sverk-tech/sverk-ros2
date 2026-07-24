#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <Eigen/Geometry>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

namespace px4_local_pose_publisher
{

// Receives world-frame base_link pose (ENU) produced by ArucoTfNode,
// converts it to NED/FRD and sends a LocalPositionMeasurement to PX4
// EKF2 on-new (i.e. once per incoming pose, stamped with the frame capture
// time). No TF lookups, no offset computation, no fixed-rate re-sending —
// that is ArucoTfNode's job, and re-sending stale poses corrupts EKF2.
class VpePublisher : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  explicit VpePublisher(rclcpp::Node & node);
  ~VpePublisher();

  void start();
  void stop();

private:
  void worldPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void resetCounterCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void watchdogCallback();

  // Allows safe runtime reconfiguration of operational params (esp. the
  // allow_send_vio safety switch) via `ros2 param set` while the node runs.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // Publishes a single measurement to PX4 (guarded by allow_send_vio_).
  void sendMeasurement(const px4_ros2::LocalPositionMeasurement & meas);

  // ENU/FLU → NED/FRD conversions (via px4_ros_com)
  static Eigen::Vector3f    enuToNedPos(const Eigen::Vector3f & enu);
  static Eigen::Quaternionf rosToPx4Quat(const Eigen::Quaternionf & enu_flu);

  // ── Node reference ──────────────────────────────────────────────────────
  rclcpp::Node & node_;

  // ── Subscriptions ────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr world_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reset_counter_sub_;

  // ── Watchdog timer (staleness detection only — never re-sends data) ──────
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ── Dynamic parameter callback handle ────────────────────────────────────
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── State (protected by mutex_) ──────────────────────────────────────────
  std::mutex   mutex_;
  rclcpp::Time last_pose_time_;
  bool         has_data_{false};
  bool         stale_warned_{false};
  uint8_t      reset_counter_{0};

  // ── Send-time guard state (touched only in worldPoseCallback → lock-free;
  //     the node is spun single-threaded) ───────────────────────────────────
  // Monotonic guard: stamp of the last pose that passed the guard. A pose whose
  // capture stamp is not strictly newer is dropped (out-of-order / backward
  // time jump). Init to epoch so the first pose always passes.
  rclcpp::Time last_sent_sample_{0, 0, RCL_ROS_TIME};
  uint64_t     dropped_stale_{0};     // dropped: older than max_measurement_age_sec_
  uint64_t     dropped_nonmono_{0};   // dropped: not strictly newer than last sent

  // ── Parameters ───────────────────────────────────────────────────────────
  // Topics are read once at construction (subscriptions are created from them).
  std::string world_pose_topic_;
  std::string reset_counter_topic_;
  // Runtime-reconfigurable via onSetParameters(). allow_send_vio_ is read
  // lock-free in sendMeasurement() → atomic; the others are guarded by mutex_.
  std::atomic<bool> allow_send_vio_{false};
  double            watchdog_rate_hz_{10.0};
  double            data_timeout_sec_{0.5};
  // Drop a pose at send time if its capture stamp is older than this (s): a
  // post-stall burst of queued stale poses then never reaches EKF2 (which would
  // otherwise reject them past its fusion horizon or reset). <=0 disables.
  // Guarded by mutex_ (written in onSetParameters, read in worldPoseCallback).
  double            max_measurement_age_sec_{0.25};
};

class VpePublisherNode : public rclcpp::Node
{
public:
  VpePublisherNode();
  ~VpePublisherNode();

private:
  std::unique_ptr<VpePublisher> interface_;
};

} // namespace px4_local_pose_publisher
