// lidar_odom_publisher.cpp
// ---------------------------------------------------------------------------
// LPE по лидарной одометрии (3D-лидар Unitree L2 + point_lio).
//
// Контракт (вход):
//   /aft_mapped_to_init   nav_msgs/Odometry (best_effort)
//                         — поза ЛИДАРА в мире одометрии point_lio
//                           (гравитационно-выровненный, yaw произвольный).
//
// Логика (аналог pose_subscriber_tf2, вход — лидар, а не ArUco):
//   odomCallback:  T_final = T_post * T_odom * T_mount (математика проверенной
//                  тестовой сборки ros2_uni/odom_to_pose.py: T_mount — поза
//                  лидара на раме, T_post — доворот мира одометрии к ENU);
//                  затем ENU/FLU -> NED/FRD (px4_ros_com) и отправка в PX4
//                  on-new с ограничением max_rate_hz (по умолчанию 30 Гц —
//                  point_lio может отдавать сотни Гц, EKF2 это не нужно).
//   watchdogCallback: только детект устаревания — НИЧЕГО не шлёт (повторная
//                  отправка старой позы дестабилизирует EKF2, он коастит на IMU).
//   allow_send_vio: предохранитель — пока false, в PX4 ничего не уходит.
//
// Ковариация: диагональ pose.covariance одометрии с ENU->NED свопом x/y.
// point_lio обычно отдаёт нули -> floor в clampVar; поворот T_post учитывается
// только для yaw, кратных 90° (диагональ при этом не смешивается) — для
// произвольных углов пришлось бы вращать полную матрицу, а реальный нижний
// порог всё равно задают EKF2_EVP_NOISE/EVA_NOISE.
// ---------------------------------------------------------------------------

#include "px4_local_pose_publisher/lidar_odom_publisher.hpp"

#include <px4_ros_com/frame_transforms.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_local_pose_publisher
{

namespace
{
// Тот же флор, что и в pose_subscriber_tf2: EKF2 отвергает NaN/inf/<=0.
inline float clampVar(double v)
{
  constexpr double kMin = 1e-4;
  constexpr double kMax = 100.0;
  return static_cast<float>(std::clamp(std::isfinite(v) && v > 0.0 ? v : kMin, kMin, kMax));
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// LidarOdomPublisher — конструктор / параметры / подписки
// ─────────────────────────────────────────────────────────────────────────────
LidarOdomPublisher::LidarOdomPublisher(rclcpp::Node & node)
: LocalPositionMeasurementInterface(
    node, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::Unknown),
  node_(node)
{
  odom_topic_       = node_.declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
  allow_send_vio_   = node_.declare_parameter<bool>("allow_send_vio", false);
  watchdog_rate_hz_ = node_.declare_parameter<double>("watchdog_rate_hz", 10.0);
  data_timeout_sec_ = node_.declare_parameter<double>("data_timeout_sec", 0.5);
  max_rate_hz_      = node_.declare_parameter<double>("max_rate_hz", 30.0);

  // Поза лидара на раме (base_link -> лидар) и доворот мира одометрии к ENU.
  // Дефолты нейтральные; проверенные значения тестовой рамы ros2_uni — в launch.
  const auto mount_rpy = node_.declare_parameter<std::vector<double>>("mount_rpy", {0.0, 0.0, 0.0});
  const auto mount_xyz = node_.declare_parameter<std::vector<double>>("mount_xyz", {0.0, 0.0, 0.0});
  const auto post_rpy  = node_.declare_parameter<std::vector<double>>("post_rpy",  {0.0, 0.0, 0.0});
  if (mount_rpy.size() != 3 || mount_xyz.size() != 3 || post_rpy.size() != 3) {
    throw std::runtime_error("mount_rpy/mount_xyz/post_rpy must each have exactly 3 elements");
  }
  T_mount_ = makeTransform(mount_rpy, mount_xyz);
  T_post_  = makeTransform(post_rpy, {0.0, 0.0, 0.0});

  last_odom_time_ = node_.now();
  last_sent_time_ = rclcpp::Time(0, 0, node_.get_clock()->get_clock_type());

  // point_lio публикует default-QoS (reliable); best_effort-подписка совместима
  // с reliable-паблишером и не тормозит его при потере пакетов.
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(20).best_effort(),
    std::bind(&LidarOdomPublisher::odomCallback, this, std::placeholders::_1));

  // Регистрируем ПОСЛЕ declare_parameter, чтобы коллбэк не сработал на initial-set.
  param_cb_handle_ = node_.add_on_set_parameters_callback(
    std::bind(&LidarOdomPublisher::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(node_.get_logger(),
    "LidarOdomPublisher: odom='%s' max_rate=%.1fHz watchdog=%.1fHz send_vio=%s",
    odom_topic_.c_str(), max_rate_hz_, watchdog_rate_hz_,
    allow_send_vio_.load() ? "true" : "false");
}

// ─────────────────────────────────────────────────────────────────────────────
// Dynamic parameter callback — безопасная горячая переконфигурация
// ─────────────────────────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult LidarOdomPublisher::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const auto & name = p.get_name();

    if (name == "allow_send_vio") {
      allow_send_vio_.store(p.as_bool());
      RCLCPP_WARN(node_.get_logger(), "allow_send_vio -> %s (лидарная одометрия в PX4 %s)",
        p.as_bool() ? "true" : "false",
        p.as_bool() ? "РАЗРЕШЕНА" : "заблокирована");

    } else if (name == "data_timeout_sec") {
      const double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "data_timeout_sec must be > 0";
        return result;
      }
      std::lock_guard<std::mutex> lk(mutex_);
      data_timeout_sec_ = v;

    } else if (name == "max_rate_hz") {
      const double v = p.as_double();
      if (v < 0.0) {
        result.successful = false;
        result.reason = "max_rate_hz must be >= 0 (0 = unlimited)";
        return result;
      }
      std::lock_guard<std::mutex> lk(mutex_);
      max_rate_hz_ = v;

    } else if (name == "watchdog_rate_hz") {
      const double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "watchdog_rate_hz must be > 0";
        return result;
      }
      watchdog_rate_hz_ = v;
      if (watchdog_timer_) {
        watchdog_timer_->cancel();
        watchdog_timer_ = node_.create_wall_timer(
          std::chrono::duration<double>(1.0 / watchdog_rate_hz_),
          std::bind(&LidarOdomPublisher::watchdogCallback, this));
        RCLCPP_INFO(node_.get_logger(), "watchdog re-armed at %.1f Hz", watchdog_rate_hz_);
      }

    } else if (name == "odom_topic" || name == "mount_rpy" || name == "mount_xyz" ||
               name == "post_rpy") {
      // Подписка и трансформы создаются один раз в конструкторе.
      result.successful = false;
      result.reason = name + " is read-only at runtime (set via launch, restart node)";
      return result;
    }
  }
  return result;
}

LidarOdomPublisher::~LidarOdomPublisher() { stop(); }

void LidarOdomPublisher::start()
{
  if (watchdog_rate_hz_ <= 0.0) {
    throw std::runtime_error("watchdog_rate_hz must be > 0");
  }
  watchdog_timer_ = node_.create_wall_timer(
    std::chrono::duration<double>(1.0 / watchdog_rate_hz_),
    std::bind(&LidarOdomPublisher::watchdogCallback, this));
  RCLCPP_INFO(node_.get_logger(), "LidarOdomPublisher watchdog at %.1f Hz", watchdog_rate_hz_);
}

void LidarOdomPublisher::stop()
{
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_ = nullptr;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void LidarOdomPublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Ограничение частоты (point_lio может отдавать сотни Гц) — по времени
  // приёма, как в тестовой сборке. timestamp_sample при этом остаётся честным.
  {
    std::lock_guard<std::mutex> lk(mutex_);
    const auto now = node_.now();
    if (max_rate_hz_ > 0.0 &&
        (now - last_sent_time_).seconds() < 1.0 / max_rate_hz_)
    {
      last_odom_time_ = now;   // данные живые — watchdog не должен ругаться
      has_data_ = true;
      return;
    }
    last_sent_time_ = now;
    last_odom_time_ = now;
    has_data_      = true;
    stale_warned_  = false;
  }

  const auto & pos = msg->pose.pose.position;
  const auto & ori = msg->pose.pose.orientation;

  // Поза лидара в мире одометрии -> поза base_link в world ENU.
  Eigen::Isometry3d T_odom = Eigen::Isometry3d::Identity();
  T_odom.linear() = Eigen::Quaterniond(ori.w, ori.x, ori.y, ori.z).toRotationMatrix();
  T_odom.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);

  const Eigen::Isometry3d T_final = T_post_ * T_odom * T_mount_;

  const Eigen::Vector3f    pos_ned = enuToNedPos(T_final.translation());
  const Eigen::Quaternionf q_ned   = rosToPx4Quat(Eigen::Quaterniond(T_final.linear()));

  px4_ros2::LocalPositionMeasurement meas{};

  // Время выборки = штамп одометрии (момент скана), а не now() — иначе EKF2
  // не может компенсировать задержку пайплайна point_lio.
  meas.timestamp_sample = rclcpp::Time(msg->header.stamp);
  meas.position_xy      = Eigen::Vector2f(pos_ned.x(), pos_ned.y());
  meas.position_z       = pos_ned.z();

  // ENU(x=East, y=North) -> NED(x=North, y=East): дисперсии тоже своп.
  meas.position_xy_variance = Eigen::Vector2f(
    clampVar(msg->pose.covariance[7]), clampVar(msg->pose.covariance[0]));
  meas.position_z_variance  = clampVar(msg->pose.covariance[14]);

  meas.attitude_quaternion = q_ned;
  meas.attitude_variance   = Eigen::Vector3f(
    clampVar(msg->pose.covariance[21]),
    clampVar(msg->pose.covariance[28]),
    clampVar(msg->pose.covariance[35]));

  // point_lio не сбрасывает свой мир в полёте (в отличие от ArUco-offset),
  // поэтому reset_counter постоянный.
  meas.reset_counter = 0;

  sendMeasurement(meas);
}

void LidarOdomPublisher::watchdogCallback()
{
  // Только мониторинг устаревания — НИЧЕГО не отправляем.
  bool stale = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (has_data_ && !stale_warned_ &&
        (node_.now() - last_odom_time_).seconds() > data_timeout_sec_)
    {
      stale = true;
      stale_warned_ = true;
    }
  }
  if (stale) {
    RCLCPP_WARN(node_.get_logger(),
      "lidar odometry stale (> %.2f s): stopped feeding EKF2, it will coast on IMU.",
      data_timeout_sec_);
  }
}

void LidarOdomPublisher::sendMeasurement(const px4_ros2::LocalPositionMeasurement & meas)
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
// Конвертации координат (через px4_ros_com) и построение трансформов
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Vector3f LidarOdomPublisher::enuToNedPos(const Eigen::Vector3d & enu)
{
  const Eigen::Vector3d ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu);
  return Eigen::Vector3f(
    static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z()));
}

Eigen::Quaternionf LidarOdomPublisher::rosToPx4Quat(const Eigen::Quaterniond & enu_flu)
{
  // ros_to_px4_orientation делает ОБА поворота: мировой ENU->NED И телесный
  // FLU->FRD (см. подробный комментарий в pose_subscriber_tf2.cpp).
  const Eigen::Quaterniond ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(enu_flu);
  return Eigen::Quaternionf(
    static_cast<float>(ned.w()), static_cast<float>(ned.x()),
    static_cast<float>(ned.y()), static_cast<float>(ned.z()));
}

Eigen::Isometry3d LidarOdomPublisher::makeTransform(
  const std::vector<double> & rpy, const std::vector<double> & xyz)
{
  // Rz(yaw)*Ry(pitch)*Rx(roll) — тот же порядок, что в ros2_uni/odom_to_pose.py.
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() =
    (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  return T;
}

// ─────────────────────────────────────────────────────────────────────────────
// LidarOdomPublisherNode
// ─────────────────────────────────────────────────────────────────────────────
LidarOdomPublisherNode::LidarOdomPublisherNode()
: Node("lidar_odom_publisher")
{
  RCLCPP_INFO(get_logger(), "Starting LidarOdomPublisherNode");

  // messageCompatibilityCheck() не вызываем — см. комментарий в
  // pose_subscriber_tf2.cpp (протокол PX4 v1.16+, на 1.15.4 зависает).

  interface_ = std::make_unique<LidarOdomPublisher>(*this);

  if (!interface_->doRegister()) {
    throw std::runtime_error("Failed to register with PX4 navigation interface");
  }
  RCLCPP_INFO(get_logger(), "Registered with PX4");

  interface_->start();
}

LidarOdomPublisherNode::~LidarOdomPublisherNode()
{
  if (interface_) interface_->stop();
}

}  // namespace px4_local_pose_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<px4_local_pose_publisher::LidarOdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
