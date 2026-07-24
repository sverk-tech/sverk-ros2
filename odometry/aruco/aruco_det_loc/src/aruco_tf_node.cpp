#include "aruco_det_loc/aruco_tf_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace aruco_det_loc
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
ArucoTfNode::ArucoTfNode(const rclcpp::NodeOptions & options)
: Node("aruco_tf_node", options)
{
  // ── Parameters ─────────────────────────────────────────────────────────
  declare_parameter("pose_cov_topic",           "/aruco/loc/pose_cov");
  declare_parameter("world_pose_topic",         "/aruco/world_pose");
  declare_parameter("reset_counter_topic",      "/aruco/reset_counter");
  declare_parameter("map_offset_topic",         "/aruco/map_offset");

  declare_parameter("camera_frame_id",          "main_camera_optical");
  declare_parameter("base_link_frame_id",       "base_link");
  declare_parameter("world_frame_id",           "map");
  declare_parameter("aruco_map_frame_id",       "aruco_map");
  declare_parameter("aruco_detected_frame_id",  "aruco_map_detected");
  declare_parameter("base_aruco_frame_id",      "base_link_aruco");

  declare_parameter("allow_offset_timeout_reset", false);
  declare_parameter("initialize_at_origin", false);
  declare_parameter("offset_timeout",             3.0);
  declare_parameter("offset_init_samples", 15);
  declare_parameter("offset_candidate_translation_tolerance", 0.10);
  declare_parameter("offset_candidate_rotation_tolerance", 0.10);
  declare_parameter("covariance_xy_floor", 0.0004);
  declare_parameter("covariance_z_floor", 0.0009);
  declare_parameter("covariance_angle_floor", 0.0027415568);

  // Реаквизиция после дыры дольше этого порога (сек, по header.stamp): бамп
  // reset_counter БЕЗ пересчёта оффсета — EKF2 принимает накопившийся разрыв
  // позиции сбросом, а не рывком через инновации. <=0 — выключено.
  // 1.0 → 3.0 (полёт 2026-07-24): короткие разрывы (метка вне FOV, дропауты
  // детекции 1-3 c) при жёстком reset давали «пилу»: снос на IMU → резкий
  // возврат. Короткие разрывы теперь гасятся прогревом ковариации (ниже).
  declare_parameter("reacquisition_reset_gap", 3.0);

  // Прогрев ковариации после разрыва потока поз: дисперсии позиции первой
  // вернувшейся позы × cov_warmup_factor, затухание до 1 за
  // cov_warmup_duration_sec. Плавное подтягивание EKF2 вместо рывка (и защита
  // от отбраковки вернувшейся позы innovation-гейтом при честной σ~2-3 см).
  declare_parameter("cov_warmup_gap_sec", 0.3);
  declare_parameter("cov_warmup_duration_sec", 1.5);
  declare_parameter("cov_warmup_factor", 25.0);

  // Частота периодической переотправки оффсета T_world_aruco (Гц). Латченного
  // сообщения не хватает подписчикам с регулярным опросом; <=0 — только по
  // (пере)вычислению.
  declare_parameter("map_offset_publish_rate_hz", 10.0);

  // Swarm mode: publish base_link directly in the shared aruco_map frame
  // (offset = identity for every drone). Off = legacy per-drone world anchoring.
  declare_parameter("swarm_mode", false);

  pose_cov_topic_          = get_parameter("pose_cov_topic").as_string();
  world_pose_topic_        = get_parameter("world_pose_topic").as_string();
  reset_counter_topic_     = get_parameter("reset_counter_topic").as_string();
  map_offset_topic_        = get_parameter("map_offset_topic").as_string();
  camera_frame_id_         = get_parameter("camera_frame_id").as_string();
  base_link_frame_id_      = get_parameter("base_link_frame_id").as_string();
  world_frame_id_          = get_parameter("world_frame_id").as_string();
  aruco_map_frame_id_      = get_parameter("aruco_map_frame_id").as_string();
  aruco_detected_frame_id_ = get_parameter("aruco_detected_frame_id").as_string();
  base_aruco_frame_id_     = get_parameter("base_aruco_frame_id").as_string();
  allow_offset_timeout_reset_ = get_parameter("allow_offset_timeout_reset").as_bool();
  initialize_at_origin_    = get_parameter("initialize_at_origin").as_bool();
  offset_timeout_          = get_parameter("offset_timeout").as_double();
  offset_init_samples_     = get_parameter("offset_init_samples").as_int();
  offset_candidate_translation_tolerance_ =
    get_parameter("offset_candidate_translation_tolerance").as_double();
  offset_candidate_rotation_tolerance_ =
    get_parameter("offset_candidate_rotation_tolerance").as_double();
  covariance_xy_floor_ = get_parameter("covariance_xy_floor").as_double();
  covariance_z_floor_ = get_parameter("covariance_z_floor").as_double();
  covariance_angle_floor_ = get_parameter("covariance_angle_floor").as_double();
  reacquisition_reset_gap_ = get_parameter("reacquisition_reset_gap").as_double();
  cov_warmup_gap_sec_      = get_parameter("cov_warmup_gap_sec").as_double();
  cov_warmup_duration_sec_ = get_parameter("cov_warmup_duration_sec").as_double();
  cov_warmup_factor_       = get_parameter("cov_warmup_factor").as_double();
  map_offset_publish_rate_hz_ = get_parameter("map_offset_publish_rate_hz").as_double();
  swarm_mode_              = get_parameter("swarm_mode").as_bool();

  if (offset_init_samples_ < 1) {
    throw std::invalid_argument("offset_init_samples must be positive");
  }

  // ── TF ─────────────────────────────────────────────────────────────────
  tf_buffer_             = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_           = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_        = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  offset_.setIdentity();
  last_offset_time_ = now();

  // ── Publishers ──────────────────────────────────────────────────────────
  // world_pose is best_effort per the VPE contract; depth 10 absorbs bursts.
  world_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    world_pose_topic_, rclcpp::QoS(10).best_effort());

  // Latched (transient_local): reset_counter changes rarely, so a late-joining
  // VPE/EKF must immediately receive the current value instead of waiting for the
  // next reset. Safe now that this node runs with IPC disabled.
  reset_counter_pub_ = create_publisher<std_msgs::msg::UInt8>(
    reset_counter_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // Offset T_world_aruco for swarm/offboard tooling. Latched (transient_local)
  // so late subscribers get the last value; ALSO re-sent periodically at
  // map_offset_publish_rate_hz for subscribers that poll a steady stream.
  // Published only in single-drone mode.
  map_offset_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    map_offset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // ── Subscription ────────────────────────────────────────────────────────
  // best_effort + depth 10. aruco_loc publishes pose_cov RELIABLE, which is
  // compatible (reliable pub → best_effort sub). IPC does not engage here
  // because aruco_loc runs with intra-process comms disabled.
  pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_cov_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&ArucoTfNode::poseCovCallback, this, std::placeholders::_1));

  // ── Периодическая переотправка оффсета ───────────────────────────────
  // Однопоточный executor (component_container) сериализует таймер с
  // poseCovCallback, поэтому чтение offset_/offset_initialized_ без мьютекса
  // безопасно. До первого фикса (offset_initialized_==false) и в swarm-режиме
  // (там оффсет не вычисляется) таймер молчит.
  if (map_offset_publish_rate_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / map_offset_publish_rate_hz_);
    map_offset_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        if (!offset_initialized_) return;
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = now();
        ts.header.frame_id = world_frame_id_;
        ts.child_frame_id  = aruco_map_frame_id_;
        ts.transform       = tf2::toMsg(offset_);
        map_offset_pub_->publish(ts);
      });
  }

  // ── Dynamic parameter callback ───────────────────────────────────────
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&ArucoTfNode::onParams, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "aruco_tf_node ready | mode=%s | in='%s' world_out='%s' frame='%s'",
    swarm_mode_ ? "SWARM(aruco_map)" : "single(offset)",
    pose_cov_topic_.c_str(), world_pose_topic_.c_str(),
    swarm_mode_ ? aruco_map_frame_id_.c_str() : world_frame_id_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Dynamic parameter handler
// ─────────────────────────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult ArucoTfNode::onParams(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult r;
  r.successful = true;
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "allow_offset_timeout_reset") {
      allow_offset_timeout_reset_ = p.as_bool();
    } else if (n == "offset_timeout") {
      if (p.as_double() <= 0.0) {
        r.successful = false; r.reason = "offset_timeout must be > 0"; return r;
      }
      offset_timeout_ = p.as_double();
    } else if (n == "reacquisition_reset_gap") {
      reacquisition_reset_gap_ = p.as_double();
    } else if (n == "cov_warmup_gap_sec") {
      cov_warmup_gap_sec_ = p.as_double();
    } else if (n == "cov_warmup_duration_sec") {
      cov_warmup_duration_sec_ = p.as_double();
    } else if (n == "cov_warmup_factor") {
      cov_warmup_factor_ = p.as_double();
    } else if (n == "swarm_mode") {
      const bool v = p.as_bool();
      if (v != swarm_mode_) {
        swarm_mode_ = v;
        // Switching modes changes the output frame and the numeric pose. Force a
        // clean re-init of both paths so reset_counter bumps on the next fix and
        // EKF2 treats it as a fresh reference rather than a position jump.
        swarm_fix_published_ = false;
        offset_initialized_  = false;
        offset_candidates_.clear();
        RCLCPP_WARN(get_logger(),
          "swarm_mode -> %s at runtime: output frame becomes '%s'; reset_counter "
          "will bump on next fix. Prefer setting this at launch, not in flight.",
          v ? "ON" : "off",
          v ? aruco_map_frame_id_.c_str() : world_frame_id_.c_str());
      }
    } else if (n == "world_frame_id")  { world_frame_id_  = p.as_string(); }
    else if (n == "camera_frame_id")   { camera_frame_id_ = p.as_string(); }
    else if (n == "base_link_frame_id"){ base_link_frame_id_ = p.as_string(); }
    else if (n == "aruco_map_frame_id"){ aruco_map_frame_id_ = p.as_string(); }
    else if (n == "aruco_detected_frame_id") { aruco_detected_frame_id_ = p.as_string(); }
    else if (n == "base_aruco_frame_id")     { base_aruco_frame_id_     = p.as_string(); }
  }
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main callback
// ─────────────────────────────────────────────────────────────────────────────
void ArucoTfNode::poseCovCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Input: T_aruco_camera — camera pose in aruco_map frame (ENU, from aruco_loc).
  const tf2::Transform T_aruco_camera = poseToTf(msg->pose.pose);

  // ── Look up camera → base_link ─────────────────────────────────────────
  tf2::Transform T_camera_base;
  try {
    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_->lookupTransform(camera_frame_id_, base_link_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(tf.transform, T_camera_base);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "TF %s->%s: %s", camera_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
    return;
  }

  const rclcpp::Time stamp = msg->header.stamp;

  // ── Реаквизиция после дыры ─────────────────────────────────────────────
  // За дыру EKF2 дедреконит на optical flow и уплывает; вернувшаяся поза
  // отличается на накопленный дрейф. Бамп reset_counter говорит EKF2 принять
  // разрыв сбросом позиции (без выброса скорости через инновации). Оффсет
  // мира сознательно НЕ пересчитывается — пересчёт заякорил бы карту по
  // уплывшему EKF и втащил дрейф в оффсет насовсем. Возможный двойной
  // инкремент вместе с commitOffset безвреден: EKF2 сравнивает счётчик
  // на неравенство, а не на +1.
  if (has_last_pose_stamp_) {
    const double gap = (stamp - last_pose_stamp_).seconds();
    if (reacquisition_reset_gap_ > 0.0 && gap > reacquisition_reset_gap_) {
      if (reset_counter_ < 255) ++reset_counter_;
      std_msgs::msg::UInt8 rc;
      rc.data = reset_counter_;
      reset_counter_pub_->publish(rc);
      RCLCPP_INFO(get_logger(),
        "Reacquisition after %.2f s gap (> %.2f s) — reset_counter=%u",
        gap, reacquisition_reset_gap_, static_cast<unsigned>(reset_counter_));
    }
    // Прогрев ковариации после любого заметного разрыва (в т.ч. короче
    // reset-порога): EKF2 успел удрейфовать на IMU, честная σ~2-3 см либо
    // дёрнула бы фильтр к вернувшейся позе рывком, либо поза была бы отброшена
    // innovation-гейтом. Затухающее раздувание дисперсий → плавный съезд.
    if (cov_warmup_factor_ > 1.0 && gap > cov_warmup_gap_sec_) {
      warmup_start_ = stamp;
      warmup_active_ = true;
      RCLCPP_INFO(get_logger(),
        "Pose gap %.2f s — covariance warm-up x%.0f for %.1f s",
        gap, cov_warmup_factor_, cov_warmup_duration_sec_);
    }
  }

  auto make_tf = [&](const std::string & parent, const std::string & child,
                     const tf2::Transform & T) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = stamp;
    ts.header.frame_id = parent;
    ts.child_frame_id  = child;
    ts.transform       = tf2::toMsg(T);
    return ts;
  };

  // ── Swarm mode: base_link directly in the shared aruco_map frame ───────
  // No per-drone offset, no dependency on the EKF world (map→base_link).
  // Every drone reports in the SAME aruco_map frame → absolute, comparable.
  if (swarm_mode_) {
    const tf2::Transform T_aruco_base = T_aruco_camera * T_camera_base;

    // On the FIRST fix, bump reset_counter so the VPE/EKF2 adopts a fresh
    // external-vision reference instead of treating a far-from-origin sample
    // as a position jump (see UPGRADE/swarm notes).
    if (!swarm_fix_published_) {
      swarm_fix_published_ = true;
      if (reset_counter_ < 255) ++reset_counter_;
      std_msgs::msg::UInt8 rc; rc.data = reset_counter_;
      reset_counter_pub_->publish(rc);
      RCLCPP_INFO(get_logger(),
        "swarm_mode: first fix, base_link in '%s', reset_counter=%u",
        aruco_map_frame_id_.c_str(), static_cast<unsigned>(reset_counter_));
    }

    tf_broadcaster_->sendTransform(
      make_tf(aruco_map_frame_id_, aruco_detected_frame_id_, T_aruco_camera));
    tf_broadcaster_->sendTransform(
      make_tf(aruco_map_frame_id_, base_aruco_frame_id_, T_aruco_base));

    auto out = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    out->header.stamp    = stamp;
    out->header.frame_id = aruco_map_frame_id_;   // common absolute frame for the swarm
    const tf2::Vector3 & o = T_aruco_base.getOrigin();
    out->pose.pose.position.x  = o.x();
    out->pose.pose.position.y  = o.y();
    out->pose.pose.position.z  = o.z();
    out->pose.pose.orientation = tf2::toMsg(T_aruco_base.getRotation());
    out->pose.covariance       = msg->pose.covariance;
    applyCovarianceFloor(out->pose.covariance);
    applyCovarianceWarmup(out->pose.covariance, stamp);
    world_pose_pub_->publish(std::move(out));
    last_pose_stamp_     = stamp;
    has_last_pose_stamp_ = true;
    return;
  }

  // ── Decide whether offset needs (re)computing ─────────────────────────
  // Staleness is measured in the message-time domain: last_offset_time_ holds
  // the header stamp of the last accepted fix,
  // so we must compare against THIS message's stamp — not wall now(). Mixing the
  // two breaks under sim time (header.stamp is sim time, now() is wall time →
  // age ≈ epoch ≫ timeout → recompute every frame → reset_counter storm), and is
  // semantically wrong even on hardware.
  if (allow_offset_timeout_reset_ && offset_initialized_) {
    const double age = (stamp - last_offset_time_).seconds();
    if (age > offset_timeout_) {
      offset_initialized_ = false;
      offset_candidates_.clear();
    }
  }

  if (!offset_initialized_) {
    tf2::Transform candidate_offset;
    if (!computeCandidateOffset(T_aruco_camera, T_camera_base, candidate_offset)) {
      return;
    }
    if (!qualifyOffset(candidate_offset, offset_init_samples_)) {
      return;
    }
    commitOffset(candidate_offset, stamp);
  }

  last_offset_time_ = stamp;

  // ── Apply offset: T_world_camera = T_world_aruco * T_aruco_camera ─────
  const tf2::Transform T_world_camera = offset_ * T_aruco_camera;

  // ── base_link pose in world ────────────────────────────────────────────
  const tf2::Transform T_world_base = T_world_camera * T_camera_base;

  // ── Publish dynamic TFs ───────────────────────────────────────────────
  tf_broadcaster_->sendTransform(
    make_tf(world_frame_id_, aruco_detected_frame_id_, T_world_camera));
  tf_broadcaster_->sendTransform(
    make_tf(world_frame_id_, base_aruco_frame_id_, T_world_base));

  // ── Publish corrected world-frame base_link pose ──────────────────────
  // Published as unique_ptr to enable IPC zero-copy when inside a container.
  auto out = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  out->header.stamp    = stamp;
  out->header.frame_id = world_frame_id_;

  const tf2::Vector3 & orig = T_world_base.getOrigin();
  out->pose.pose.position.x   = orig.x();
  out->pose.pose.position.y   = orig.y();
  out->pose.pose.position.z   = orig.z();
  out->pose.pose.orientation  = tf2::toMsg(T_world_base.getRotation());
  out->pose.covariance        = msg->pose.covariance;
  applyCovarianceFloor(out->pose.covariance);
  applyCovarianceWarmup(out->pose.covariance, stamp);

  world_pose_pub_->publish(std::move(out));
  last_pose_stamp_     = stamp;
  has_last_pose_stamp_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Offset computation
// ─────────────────────────────────────────────────────────────────────────────
// Computes offset_ = T_world_aruco_map such that:
//   T_world_camera = offset_ * T_aruco_camera
//
// Derivation (chain rule):
//   T_world_aruco = T_world_base * T_base_camera * T_camera_aruco
bool ArucoTfNode::computeCandidateOffset(
  const tf2::Transform & T_aruco_camera,
  const tf2::Transform & T_camera_base,
  tf2::Transform & candidate)
{
  if (initialize_at_origin_ && !offset_initialized_) {
    const tf2::Transform T_aruco_base = T_aruco_camera * T_camera_base;
    candidate = T_aruco_base.inverse();
    return true;
  }

  // world → base_link (must be in world/ENU frame)
  tf2::Transform T_world_base;
  try {
    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_->lookupTransform(world_frame_id_, base_link_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(tf.transform, T_world_base);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Cannot compute offset: no TF %s->%s: %s",
      world_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
    return false;
  }

  const tf2::Transform T_base_camera  = T_camera_base.inverse();
  const tf2::Transform T_camera_aruco = T_aruco_camera.inverse();
  candidate = T_world_base * T_base_camera * T_camera_aruco;
  return true;
}

bool ArucoTfNode::offsetsAgree(
  const tf2::Transform & lhs,
  const tf2::Transform & rhs,
  double translation_tolerance,
  double rotation_tolerance) const
{
  const double translation_delta = (lhs.getOrigin() - rhs.getOrigin()).length();
  const double rotation_delta = lhs.getRotation().angleShortestPath(rhs.getRotation());
  return translation_delta <= translation_tolerance &&
         rotation_delta <= rotation_tolerance;
}

bool ArucoTfNode::qualifyOffset(
  const tf2::Transform & candidate,
  int required_samples)
{
  if (!offset_candidates_.empty() && !offsetsAgree(
      candidate, offset_candidates_.front(),
      offset_candidate_translation_tolerance_,
      offset_candidate_rotation_tolerance_))
  {
    offset_candidates_.clear();
  }

  offset_candidates_.push_back(candidate);
  while (static_cast<int>(offset_candidates_.size()) > required_samples) {
    offset_candidates_.pop_front();
  }
  return static_cast<int>(offset_candidates_.size()) >= required_samples;
}

void ArucoTfNode::commitOffset(
  const tf2::Transform & candidate,
  const rclcpp::Time & stamp)
{
  offset_ = candidate;

  offset_initialized_ = true;
  offset_candidates_.clear();
  last_offset_time_   = stamp;

  // Increment reset counter and publish so VpePublisher bumps EKF2 reset_counter.
  if (reset_counter_ < 255) ++reset_counter_;
  std_msgs::msg::UInt8 rc;
  rc.data = reset_counter_;
  reset_counter_pub_->publish(rc);

  // Static TF for the offset (re-published on every recompute for rviz/tools).
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp    = stamp;
  static_tf.header.frame_id = world_frame_id_;
  static_tf.child_frame_id  = aruco_map_frame_id_;
  static_tf.transform       = tf2::toMsg(offset_);
  static_tf_broadcaster_->sendTransform(static_tf);

  // Same offset as a latched topic (T_world_aruco) for offboard/swarm tooling:
  // header.frame_id = world, child_frame_id = aruco_map.
  map_offset_pub_->publish(static_tf);

  RCLCPP_INFO(get_logger(),
    "Offset (re)computed: %s->%s  t=[%.3f, %.3f, %.3f]  reset_counter=%u",
    world_frame_id_.c_str(), aruco_map_frame_id_.c_str(),
    offset_.getOrigin().x(), offset_.getOrigin().y(), offset_.getOrigin().z(),
    static_cast<unsigned>(reset_counter_));

}

void ArucoTfNode::applyCovarianceFloor(std::array<double, 36> & covariance) const
{
  covariance[0] = std::max(covariance[0], covariance_xy_floor_);
  covariance[7] = std::max(covariance[7], covariance_xy_floor_);
  covariance[14] = std::max(covariance[14], covariance_z_floor_);
  covariance[21] = std::max(covariance[21], covariance_angle_floor_);
  covariance[28] = std::max(covariance[28], covariance_angle_floor_);
  covariance[35] = std::max(covariance[35], covariance_angle_floor_);
}

// Затухающее раздувание дисперсий позиции после разрыва потока поз
// (см. cov_warmup_* и триггер в poseCovCallback). Линейное затухание
// factor → 1 за cov_warmup_duration_sec от warmup_start_.
void ArucoTfNode::applyCovarianceWarmup(std::array<double, 36> & covariance,
                                        const rclcpp::Time & stamp)
{
  if (!warmup_active_) return;
  const double elapsed = (stamp - warmup_start_).seconds();
  if (elapsed < 0.0 || elapsed >= cov_warmup_duration_sec_ ||
      cov_warmup_duration_sec_ <= 0.0) {
    warmup_active_ = false;
    return;
  }
  const double s =
    1.0 + (cov_warmup_factor_ - 1.0) * (1.0 - elapsed / cov_warmup_duration_sec_);
  covariance[0]  *= s;
  covariance[7]  *= s;
  covariance[14] *= s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────────────
tf2::Transform ArucoTfNode::poseToTf(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf.setRotation(tf2::Quaternion(
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w));
  return tf;
}

} // namespace aruco_det_loc

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_det_loc::ArucoTfNode)
