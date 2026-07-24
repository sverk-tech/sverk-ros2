#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <array>
#include <deque>
#include <string>
#include <memory>

namespace aruco_det_loc
{

// Receives aruco_loc output (T_aruco_camera, ENU PoseWithCovarianceStamped) and
// publishes the base_link pose for the PX4-VPE bridge. Two modes:
//
// ── Single-drone mode (swarm_mode=false, default) ──────────────────────────
//   Anchors the aruco_map into THIS drone's world (map / EKF take-off frame)
//   via a qualified, reset-aware offset, so the output matches the drone's
//   current world. Output is base_link in `world_frame_id`.
//     Static TF:   world_frame  → aruco_map_frame       (offset; re-sent on recompute)
//     Dynamic TF:  world_frame  → aruco_detected_frame  (camera pose in world)
//     Dynamic TF:  world_frame  → base_aruco_frame      (base_link pose in world)
//     Topic:       world_pose_topic  (base_link in world/ENU)
//     Topic:       reset_counter_topic (UInt8, bumped on every offset recompute
//                  and on reacquisition after a gap > reacquisition_reset_gap)
//
// ── Swarm mode (swarm_mode=true) ───────────────────────────────────────────
//   No per-drone offset and no dependency on the EKF world: publishes base_link
//   DIRECTLY in the shared `aruco_map_frame_id`, identical for every drone, so
//   absolute swarm coordinates are comparable across the fleet.
//     T_aruco_base = T_aruco_camera · T_camera_base
//     Dynamic TF:  aruco_map_frame → aruco_detected_frame (camera in aruco_map)
//     Dynamic TF:  aruco_map_frame → base_aruco_frame     (base_link in aruco_map)
//     Topic:       world_pose_topic  (base_link in aruco_map)  header.frame_id = aruco_map
//     Topic:       reset_counter_topic (UInt8, bumped on first fix and on
//                  reacquisition after a gap > reacquisition_reset_gap)
//   NOTE: assumes aruco_map is gravity-aligned (marker sheet horizontal, Z up)
//   so the pose stays a valid ENU input for the VPE bridge.
//
// Designed to run as a composable component in the same container as
// ArucoDetectNode and ArucoLocNode for zero-copy intra-process comms.
class ArucoTfNode : public rclcpp::Node
{
public:
  explicit ArucoTfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void poseCovCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool computeCandidateOffset(const tf2::Transform & T_aruco_camera,
                              const tf2::Transform & T_camera_base,
                              tf2::Transform & candidate);
  bool offsetsAgree(const tf2::Transform & lhs, const tf2::Transform & rhs,
                    double translation_tolerance, double rotation_tolerance) const;
  bool qualifyOffset(const tf2::Transform & candidate, int required_samples);
  void commitOffset(const tf2::Transform & candidate, const rclcpp::Time & stamp);
  void applyCovarianceFloor(std::array<double, 36> & covariance) const;
  void applyCovarianceWarmup(std::array<double, 36> & covariance, const rclcpp::Time & stamp);

  rcl_interfaces::msg::SetParametersResult onParams(
    const std::vector<rclcpp::Parameter> & params);

  static tf2::Transform poseToTf(const geometry_msgs::msg::Pose & pose);

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string pose_cov_topic_;
  std::string world_pose_topic_;
  std::string reset_counter_topic_;
  std::string map_offset_topic_;
  std::string camera_frame_id_;
  std::string base_link_frame_id_;
  std::string world_frame_id_;
  std::string aruco_map_frame_id_;
  std::string aruco_detected_frame_id_;
  std::string base_aruco_frame_id_;
  bool   allow_offset_timeout_reset_{false};
  bool   initialize_at_origin_{false};
  double offset_timeout_{3.0};
  int    offset_init_samples_{15};
  double offset_candidate_translation_tolerance_{0.10};
  double offset_candidate_rotation_tolerance_{0.10};
  double covariance_xy_floor_{0.0004};
  double covariance_z_floor_{0.0009};
  double covariance_angle_floor_{0.0027415568};
  // Реаквизиция: если между соседними опубликованными позами прошло больше
  // этого времени (сек, по header.stamp), бампнуть reset_counter — EKF2 примет
  // накопившийся за дыру разрыв позиции как reset, а не как скачок скорости.
  // Оффсет мира при этом НЕ пересчитывается (в отличие от
  // allow_offset_timeout_reset, который заякоривает карту заново по EKF и
  // втащил бы OF-дрейф в оффсет). <=0 — выключено.
  // 1.0 → 3.0 после полёта 2026-07-24: короткие разрывы (метка вне FOV /
  // дропаут детекции 1-3 c) при жёстком reset давали «пилу» — снос на IMU и
  // резкий возврат. Теперь короткие разрывы гасятся прогревом ковариации
  // (cov_warmup_*), reset остаётся для длинных отсутствий.
  double reacquisition_reset_gap_{3.0};
  // Прогрев ковариации после разрыва потока поз: первая поза после дыры
  // > cov_warmup_gap_sec получает дисперсии позиции × cov_warmup_factor,
  // затухающие до 1 за cov_warmup_duration_sec. EKF2 подтягивается к
  // вернувшейся позе плавно вместо рывка (и не отбрасывает её по
  // innovation-гейту при честной, но маленькой σ). factor <= 1 — выключено.
  double cov_warmup_gap_sec_{0.3};
  double cov_warmup_duration_sec_{1.5};
  double cov_warmup_factor_{25.0};
  // Оффсет T_world_aruco публикуется периодически (Гц), а не только в момент
  // (пере)вычисления. Латченного сообщения не хватает подписчикам, которые
  // ждут регулярный поток (мониторинг/offboard-тулинг). <=0 — только по
  // изменению (прежнее поведение).
  double map_offset_publish_rate_hz_{10.0};

  // Swarm mode: publish base_link directly in the shared aruco_map frame.
  bool   swarm_mode_{false};
  bool   swarm_fix_published_{false};   // bumped reset_counter on first swarm fix?

  // ── Subscription ──────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr world_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_counter_pub_;
  // T_world_aruco offset as a latched topic (single-drone mode only; in swarm
  // mode the offset is identity and nothing is published here).
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_offset_pub_;

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Периодическая переотправка последнего оффсета (см. map_offset_publish_rate_hz_).
  rclcpp::TimerBase::SharedPtr map_offset_timer_;

  // ── State ─────────────────────────────────────────────────────────────────
  // All state below is touched only from poseCovCallback on the single executor
  // thread (component_container / rclcpp::spin), so no mutex is required.
  tf2::Transform offset_;                    // T_world_aruco_map (ENU)
  bool           offset_initialized_{false};
  std::deque<tf2::Transform> offset_candidates_;
  rclcpp::Time   last_offset_time_;
  uint8_t        reset_counter_{0};
  // Стамп последней ОПУБЛИКОВАННОЙ world_pose (message-time domain) — для
  // детекта дыр реаквизиции. Обновляется в обоих режимах (single/swarm).
  bool           has_last_pose_stamp_{false};
  rclcpp::Time   last_pose_stamp_;
  // Прогрев ковариации (см. cov_warmup_*): момент начала текущего прогрева.
  bool           warmup_active_{false};
  rclcpp::Time   warmup_start_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace aruco_det_loc
