// pose_subscriber_tf2.cpp
// ---------------------------------------------------------------------------
// VPE (Visual Position Estimate) для нового ArUco composed-пайплайна.
//
// Контракт (вход):
//   aruco/world_pose     geometry_msgs/PoseWithCovarianceStamped (best_effort)
//                        — поза base_link в world-фрейме (ENU, тело FLU),
//                          уже с offset, считается ArucoTfNode.
//                          header.stamp трассируется к моменту захвата кадра.
//   aruco/reset_counter  std_msgs/UInt8 (reliable, KeepLast(1))
//                        — счётчик сброса EKF2 (бамп при пересчёте offset).
//
// Логика:
//   worldPoseCallback:  ENU/FLU -> NED/FRD (px4_ros_com), timestamp_sample из
//                       header.stamp (время захвата!), и СРАЗУ отправка в PX4
//                       (on-new). Каждое измерение уходит ровно один раз.
//   watchdogCallback:   только детект устаревания данных — НИЧЕГО не шлёт.
//                       При дропауте EKF2 коастит на IMU и сам перестаёт
//                       фьюзить EV по таймауту; повторная отправка старой позы
//                       или нейтрали дестабилизирует фильтр.
//   allow_send_vio:     предохранитель — пока false, в PX4 ничего не уходит.
// ---------------------------------------------------------------------------

#include "px4_local_pose_publisher/pose_subscriber_tf2.hpp"

#include <px4_ros_com/frame_transforms.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_local_pose_publisher
{

namespace
{
// Зажимаем дисперсию в разумный диапазон, чтобы не кормить EKF2 мусором
// (NaN/inf/<=0 интерфейс отвергает). Неизвестную/нулевую дисперсию поднимаем
// до небольшого, но осмысленного флора (~σ=1 см / 0.57°); при EKF2_EV_NOISE_MD=0
// настоящим нижним порогом всё равно служит параметр EKF2_EVP_NOISE/EVA_NOISE.
inline float clampVar(double v)
{
  constexpr double kMin = 1e-4;
  constexpr double kMax = 100.0;
  return static_cast<float>(std::clamp(std::isfinite(v) && v > 0.0 ? v : kMin, kMin, kMax));
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// VpePublisher — конструктор / параметры / подписки
// ─────────────────────────────────────────────────────────────────────────────
VpePublisher::VpePublisher(rclcpp::Node & node)
: LocalPositionMeasurementInterface(
    node, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::Unknown),
  node_(node)
{
  world_pose_topic_    = node_.declare_parameter<std::string>("world_pose_topic",    "/aruco/world_pose");
  reset_counter_topic_ = node_.declare_parameter<std::string>("reset_counter_topic", "/aruco/reset_counter");
  allow_send_vio_      = node_.declare_parameter<bool>("allow_send_vio",   false);
  watchdog_rate_hz_    = node_.declare_parameter<double>("watchdog_rate_hz", 10.0);
  data_timeout_sec_    = node_.declare_parameter<double>("data_timeout_sec", 0.5);
  max_measurement_age_sec_ = node_.declare_parameter<double>("max_measurement_age_sec", 0.25);

  last_pose_time_ = node_.now();

  // Поза: best_effort + depth 10 — совпадает с QoS паблишера ArucoTfNode.
  world_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    world_pose_topic_, rclcpp::QoS(10).best_effort(),
    std::bind(&VpePublisher::worldPoseCallback, this, std::placeholders::_1));

  // Reset-counter: reliable, KeepLast(1). NB: durability VOLATILE (не latched) —
  // если эта нода стартует ПОЗЖЕ публикации bootstrap reset_counter, она его
  // не получит. Для гарантии late-join обе стороны надо перевести на
  // transient_local; bootstrap-reset для EKF2 не критичен (см. README).
  reset_counter_sub_ = node_.create_subscription<std_msgs::msg::UInt8>(
    reset_counter_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&VpePublisher::resetCounterCallback, this, std::placeholders::_1));

  // Регистрируем ПОСЛЕ declare_parameter, чтобы коллбэк не сработал на initial-set.
  param_cb_handle_ = node_.add_on_set_parameters_callback(
    std::bind(&VpePublisher::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(node_.get_logger(),
    "VpePublisher: world_pose='%s' reset_counter='%s' watchdog=%.1fHz send_vio=%s max_age=%.0fms",
    world_pose_topic_.c_str(), reset_counter_topic_.c_str(),
    watchdog_rate_hz_, allow_send_vio_.load() ? "true" : "false",
    max_measurement_age_sec_ * 1e3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Dynamic parameter callback — безопасная горячая переконфигурация
// ─────────────────────────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult VpePublisher::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    if (name == "allow_send_vio") {
      allow_send_vio_.store(p.as_bool());
      RCLCPP_WARN(node_.get_logger(), "allow_send_vio -> %s (VIO в PX4 %s)",
        p.as_bool() ? "true" : "false",
        p.as_bool() ? "РАЗРЕШЕНА" : "заблокирована");

    } else if (name == "data_timeout_sec") {
      const double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "data_timeout_sec must be > 0";
        return result;
      }
      std::lock_guard<std::mutex> lk(mutex_);   // читается в watchdogCallback под mutex_
      data_timeout_sec_ = v;

    } else if (name == "max_measurement_age_sec") {
      const double v = p.as_double();
      if (!std::isfinite(v)) {
        result.successful = false;
        result.reason = "max_measurement_age_sec must be finite (<=0 disables)";
        return result;
      }
      std::lock_guard<std::mutex> lk(mutex_);   // читается в worldPoseCallback под mutex_
      max_measurement_age_sec_ = v;

    } else if (name == "watchdog_rate_hz") {
      const double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "watchdog_rate_hz must be > 0";
        return result;
      }
      watchdog_rate_hz_ = v;
      if (watchdog_timer_) {   // пересоздаём таймер под новую частоту
        watchdog_timer_->cancel();
        watchdog_timer_ = node_.create_wall_timer(
          std::chrono::duration<double>(1.0 / watchdog_rate_hz_),
          std::bind(&VpePublisher::watchdogCallback, this));
        RCLCPP_INFO(node_.get_logger(), "watchdog re-armed at %.1f Hz", watchdog_rate_hz_);
      }

    } else if (name == "world_pose_topic" || name == "reset_counter_topic") {
      // Подписки создаются один раз в конструкторе — менять на лету нельзя.
      result.successful = false;
      result.reason = name + " is read-only at runtime (set via launch, restart node)";
      return result;
    }
  }
  return result;
}

VpePublisher::~VpePublisher() { stop(); }

void VpePublisher::start()
{
  if (watchdog_rate_hz_ <= 0.0) {
    throw std::runtime_error("watchdog_rate_hz must be > 0");
  }
  watchdog_timer_ = node_.create_wall_timer(
    std::chrono::duration<double>(1.0 / watchdog_rate_hz_),
    std::bind(&VpePublisher::watchdogCallback, this));
  RCLCPP_INFO(node_.get_logger(), "VpePublisher watchdog at %.1f Hz", watchdog_rate_hz_);
}

void VpePublisher::stop()
{
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_ = nullptr;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void VpePublisher::resetCounterCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  reset_counter_ = msg->data;
}

void VpePublisher::worldPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // ── Send-time guard: protect EKF2 from a post-stall flush of STALE poses.
  // If the bridge/executor ever stalls while /aruco/world_pose keeps arriving,
  // the best_effort(10) queue flushes up to 10 OLD poses as a burst on resume
  // (reproduced on-drone: staleness up to ~0.5 s). Feeding those to EKF2 past
  // its fusion horizon triggers innovation-gate rejects / resets → the drone
  // jerks. Drop anything older than max_measurement_age_sec_, and anything not
  // strictly newer than the last accepted sample (out-of-order / backward jump).
  // Clock-consistent: now() and header.stamp share the clock, so a clock STEP
  // moves both together and does not spuriously drop fresh frames.
  const rclcpp::Time sample(msg->header.stamp);
  double max_age;
  { std::lock_guard<std::mutex> lk(mutex_); max_age = max_measurement_age_sec_; }

  const double age = (node_.now() - sample).seconds();
  if (max_age > 0.0 && std::isfinite(age) && age > max_age) {
    ++dropped_stale_;
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "dropped stale world_pose (age %.0f ms > %.0f ms); total_stale=%lu",
      age * 1e3, max_age * 1e3, static_cast<unsigned long>(dropped_stale_));
    return;
  }
  if (sample <= last_sent_sample_) {
    ++dropped_nonmono_;
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "dropped non-monotonic world_pose (stamp not newer than last accepted); total_nonmono=%lu",
      static_cast<unsigned long>(dropped_nonmono_));
    return;
  }
  last_sent_sample_ = sample;

  const auto & pos = msg->pose.pose.position;
  const auto & ori = msg->pose.pose.orientation;

  const Eigen::Vector3f pos_enu(
    static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z));
  const Eigen::Quaternionf q_enu_flu(
    static_cast<float>(ori.w), static_cast<float>(ori.x),
    static_cast<float>(ori.y), static_cast<float>(ori.z));

  const Eigen::Vector3f    pos_ned = enuToNedPos(pos_enu);
  const Eigen::Quaternionf q_ned   = rosToPx4Quat(q_enu_flu);

  // Диагональ ковариации позы (6x6): индексы 0,7,14 — xx,yy,zz (позиция, ENU).
  const double cov_x_enu = msg->pose.covariance[0];
  const double cov_y_enu = msg->pose.covariance[7];
  const double cov_z_enu = msg->pose.covariance[14];

  px4_ros2::LocalPositionMeasurement meas{};

  // КРИТИЧНО: время выборки = момент ЗАХВАТА кадра (header.stamp), а не now().
  // Иначе EKF2 не может компенсировать сквозную задержку пайплайна -> рывки.
  meas.timestamp_sample = sample;   // == rclcpp::Time(msg->header.stamp), см. guard выше
  meas.position_xy      = Eigen::Vector2f(pos_ned.x(), pos_ned.y());
  meas.position_z       = pos_ned.z();

  // ENU(x=East, y=North) -> NED(x=North, y=East): оси меняются местами,
  // поэтому дисперсии тоже своп: var(NED.x)=cov_y_enu, var(NED.y)=cov_x_enu.
  meas.position_xy_variance = Eigen::Vector2f(clampVar(cov_y_enu), clampVar(cov_x_enu));
  meas.position_z_variance  = clampVar(cov_z_enu);

  meas.attitude_quaternion  = q_ned;
  // Диагональ ковариации ориентации: индексы 21,28,35 — roll,pitch,yaw.
  // Поворот тела FLU->FRD (180° по X) диагональ не меняет, поэтому проброс прямой.
  meas.attitude_variance = Eigen::Vector3f(
    clampVar(msg->pose.covariance[21]),
    clampVar(msg->pose.covariance[28]),
    clampVar(msg->pose.covariance[35]));

  {
    std::lock_guard<std::mutex> lk(mutex_);
    meas.reset_counter = reset_counter_;
    last_pose_time_    = node_.now();
    has_data_          = true;
    stale_warned_      = false;
  }

  // On-new: каждое измерение уходит в EKF2 ровно один раз, со своим временем
  // захвата. Источник уже ~30 Гц — это минимум по докам PX4 для сообщений с
  // ковариацией, отдельный fixed-rate ретранслятор не нужен и вреден.
  sendMeasurement(meas);
}

void VpePublisher::watchdogCallback()
{
  // Только мониторинг устаревания — НИЧЕГО не отправляем.
  bool stale = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (has_data_ && !stale_warned_ &&
        (node_.now() - last_pose_time_).seconds() > data_timeout_sec_)
    {
      stale = true;
      stale_warned_ = true;
    }
  }
  if (stale) {
    RCLCPP_WARN(node_.get_logger(),
      "world_pose stale (> %.2f s): stopped feeding EKF2, it will coast on IMU.",
      data_timeout_sec_);
  }
}

void VpePublisher::sendMeasurement(const px4_ros2::LocalPositionMeasurement & meas)
{
  // Предохранитель: пока не разрешено, в PX4 ничего не уходит.
  if (!allow_send_vio_) return;

  try {
    update(meas);
  } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "PX4 nav interface rejected measurement: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "sendMeasurement exception: %s", e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Конвертации координат (через px4_ros_com)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Vector3f VpePublisher::enuToNedPos(const Eigen::Vector3f & enu)
{
  const Eigen::Vector3d d(enu.x(), enu.y(), enu.z());
  const Eigen::Vector3d ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(d);
  return Eigen::Vector3f(
    static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()));
}

Eigen::Quaternionf VpePublisher::rosToPx4Quat(const Eigen::Quaternionf & enu_flu)
{
  // ros_to_px4_orientation делает ОБА поворота: мировой ENU->NED И телесный
  // baselink(FLU)->aircraft(FRD). Только enu_to_ned_orientation давал бы
  // FLU-тело в NED (roll≈180°, неверный yaw) — классический баг.
  const Eigen::Quaterniond d(enu_flu.w(), enu_flu.x(), enu_flu.y(), enu_flu.z());
  const Eigen::Quaterniond ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(d);
  return Eigen::Quaternionf(
    static_cast<float>(ned.w()), static_cast<float>(ned.x()),
    static_cast<float>(ned.y()), static_cast<float>(ned.z()));
}

// ─────────────────────────────────────────────────────────────────────────────
// VpePublisherNode
// ─────────────────────────────────────────────────────────────────────────────
VpePublisherNode::VpePublisherNode()
: Node("vpe_publisher")
{
  RCLCPP_INFO(get_logger(), "Starting VpePublisherNode");

  // Совместимость message-definitions с прошивкой обеспечивается пинами submodule
  // px4_msgs/px4_ros_com на release/1.15. Runtime-проверку messageCompatibilityCheck()
  // здесь НЕ вызываем: она использует протокол message_format_request (PX4 v1.16+),
  // которого на 1.15.4 нет -> таймаут и зависание старта на несколько секунд.
  // Проверять оффлайн: scripts/check-message-compatibility.py или `ros2 topic echo`.

  interface_ = std::make_unique<VpePublisher>(*this);

  if (!interface_->doRegister()) {
    throw std::runtime_error("Failed to register with PX4 navigation interface");
  }
  RCLCPP_INFO(get_logger(), "Registered with PX4");

  interface_->start();
}

VpePublisherNode::~VpePublisherNode()
{
  if (interface_) interface_->stop();
}

}  // namespace px4_local_pose_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<px4_local_pose_publisher::VpePublisherNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
