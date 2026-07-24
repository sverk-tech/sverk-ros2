#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>
#include <vector>

namespace px4_local_pose_publisher
{

// Приём лидарной одометрии (nav_msgs/Odometry от point_lio, поза ЛИДАРА в
// мире одометрии), пересчёт в позу base_link в world ENU
// (T_final = T_post * T_odom * T_mount — математика из проверенной тестовой
// сборки ros2_uni/odom_to_pose.py), конвертация ENU/FLU -> NED/FRD и отправка
// LocalPositionMeasurement в PX4 EKF2 on-new. Аналог VpePublisher
// (pose_subscriber_tf2), но вход — одометрия с 3D-лидара, а не ArUco.
class LidarOdomPublisher : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  explicit LidarOdomPublisher(rclcpp::Node & node);
  ~LidarOdomPublisher();

  void start();
  void stop();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void watchdogCallback();

  // Горячая переконфигурация (allow_send_vio и таймауты) через `ros2 param set`.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // Отправка одного измерения в PX4 (под предохранителем allow_send_vio_).
  void sendMeasurement(const px4_ros2::LocalPositionMeasurement & meas);

  // ENU/FLU → NED/FRD (через px4_ros_com)
  static Eigen::Vector3f    enuToNedPos(const Eigen::Vector3d & enu);
  static Eigen::Quaternionf rosToPx4Quat(const Eigen::Quaterniond & enu_flu);

  // Rz(yaw)*Ry(pitch)*Rx(roll) + трансляция — как в ros2_uni/odom_to_pose.py
  static Eigen::Isometry3d makeTransform(
    const std::vector<double> & rpy, const std::vector<double> & xyz);

  // ── Node reference ──────────────────────────────────────────────────────
  rclcpp::Node & node_;

  // ── Subscriptions / watchdog ────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── Статические трансформы (читаются один раз в конструкторе) ───────────
  Eigen::Isometry3d T_mount_{Eigen::Isometry3d::Identity()};  // base_link -> лидар
  Eigen::Isometry3d T_post_{Eigen::Isometry3d::Identity()};   // мир одометрии -> world ENU

  // ── State (под mutex_) ──────────────────────────────────────────────────
  std::mutex   mutex_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_sent_time_;
  bool         has_data_{false};
  bool         stale_warned_{false};

  // ── Parameters ──────────────────────────────────────────────────────────
  std::string odom_topic_;
  std::atomic<bool> allow_send_vio_{false};
  double            watchdog_rate_hz_{10.0};
  double            data_timeout_sec_{0.5};
  double            max_rate_hz_{30.0};  // 0 = без ограничения частоты
};

class LidarOdomPublisherNode : public rclcpp::Node
{
public:
  LidarOdomPublisherNode();
  ~LidarOdomPublisherNode();

private:
  std::unique_ptr<LidarOdomPublisher> interface_;
};

}  // namespace px4_local_pose_publisher
