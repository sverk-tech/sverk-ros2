#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <fstream>
#include <chrono>
#include <cstdio>

// CPU affinity
#include <sched.h>
#include <cerrno>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "aruco_det_loc/msg/marker_array.hpp"
#include "aruco_det_loc/aruco_loc_utils.hpp"

namespace aruco_det_loc
{
using MarkerArrayMsg = aruco_det_loc::msg::MarkerArray;

// ---------------------------------------------------------------------------
// Timing helpers (same pattern as aruco_detect_node)
// ---------------------------------------------------------------------------
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

static inline double ms(TimePoint a, TimePoint b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

class ArucoLocNode : public rclcpp::Node
{
public:
  // Accept NodeOptions so this class can be loaded as a composable component.
  explicit ArucoLocNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("aruco_loc_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "Initializing ArucoLocNode");

    // Node behavior parameters
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("markers_topic", "/aruco/det/markers");
    declare_parameter<std::string>("map_markers_topic", "/aruco/map/markers");
    declare_parameter<std::string>(
      "camera_info_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info");

    // Output
    declare_parameter<std::string>("pose_topic", "/aruco/loc/pose_cov");

    // TF parameters
    declare_parameter<std::string>("tf_parent_frame", "main_camera_optical");
    declare_parameter<std::string>("tf_child_frame", "aruco_map_detected");
    declare_parameter<std::string>("tf_map_frame", "aruco_map");
    declare_parameter<std::string>("tf_markers_prefix", "aruco_");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("publish_static_markers_tf", true);
    declare_parameter<bool>("detect_individual_markers", true);
    declare_parameter<bool>("put_markers_count_to_covariance", false);

    // Frames
    declare_parameter<std::string>("world_frame_id", "map");
    declare_parameter<std::string>("base_link_frame_id", "base_link");
    declare_parameter<std::string>("camera_frame_id", "");

    // Known vertical (optional)
    declare_parameter<std::string>("known_vertical_frame", "");
    declare_parameter<bool>("flip_vertical", false);
    declare_parameter<bool>("auto_flip", false);

    // Одномаркерный режим с attitude-prior (P1′/P2′/P3′, см.
    // SINGLE_MARKER_JITTER_ANALYSIS.md ч.2). prior_enabled: TF map→base_link
    // берётся НА STAMP КАДРА (не последний), поза решается как трансляция при
    // известной ориентации, кадры с расхождением PnP↔prior отбраковываются,
    // ковариация — честная геометрическая. Требует known_vertical_frame.
    declare_parameter<bool>("single.prior_enabled", true);
    declare_parameter<double>("single.tf_timeout_sec", 0.03);
    // Гейт наклона: угол оптических осей PnP↔prior. Порог должен превышать
    // систематику монтажа камеры (prior↔vision), иначе срежет все кадры.
    declare_parameter<double>("single.gate_tilt_deg", 20.0);
    // Yaw-гейт — временна́я самосогласованность ArUco-yaw; 0 = выключен.
    declare_parameter<double>("single.gate_yaw_deg", 0.0);
    declare_parameter<double>("single.prior_tilt_sigma_deg", 1.0);
    declare_parameter<double>("single.yaw_sigma_deg", 2.0);
    declare_parameter<double>("single.rp_variance", 1.0);
    declare_parameter<double>("single.corner_sigma_px", 0.5);

    // Optional refine after RANSAC
    declare_parameter<bool>("refine_pose", false);

    // PnP / RANSAC parameters
    declare_parameter<int>("pnp.iterations", 16);
    declare_parameter<double>("pnp.reprojection_error_px", 4.0);
    declare_parameter<double>("pnp.confidence", 0.99);
    declare_parameter<int>("pnp.flags", static_cast<int>(cv::SOLVEPNP_ITERATIVE));

    declare_parameter<int>("pnp.min_used_markers", 0);
    declare_parameter<int>("pnp.min_inlier_points", 0);

    // RMSE gating threshold
    declare_parameter<double>("gating.max_inlier_rmse_px", 10.0);

    // Радиальные веса углов: σ(r) = sigma0·(1+beta·r^q), r — расстояние
    // исходного пикселя от principal point / полудиагональ кадра. Краевые
    // метки не отбрасываются, а входят в IRLS-уточнение и ковариацию с
    // весом 1/σ². Дефолты — фит по остаткам ~101k углов полётных бэгов
    // 2026-07-19 против mocap-карты с калибровкой 2026-07-20 (IMX219-160
    // crop 1280x960); после каждой перекалибровки камеры перефитить.
    declare_parameter<bool>("weights.radial_enabled", true);
    declare_parameter<double>("weights.sigma0_px", 1.50);
    declare_parameter<double>("weights.beta", 2.40);
    declare_parameter<double>("weights.q", 3.0);

    // CPU affinity — pin this node's thread to specific cores.
    // Use the check_cpu_topology.sh script to identify your Cortex-A76 core IDs.
    declare_parameter<std::vector<int64_t>>("cpu_affinity_ids", std::vector<int64_t>{});

    // Timing / profiling
    declare_parameter<bool>("enable_timing_log", false);

    // Load parameters
    enabled_               = get_parameter("enabled").as_bool();
    markers_topic_         = get_parameter("markers_topic").as_string();
    map_markers_topic_     = get_parameter("map_markers_topic").as_string();
    camera_info_topic_     = get_parameter("camera_info_topic").as_string();

    pose_topic_            = get_parameter("pose_topic").as_string();

    tf_parent_frame_       = get_parameter("tf_parent_frame").as_string();
    tf_child_frame_        = get_parameter("tf_child_frame").as_string();
    tf_map_frame_          = get_parameter("tf_map_frame").as_string();
    tf_markers_prefix_     = get_parameter("tf_markers_prefix").as_string();
    publish_tf_            = get_parameter("publish_tf").as_bool();
    publish_static_markers_tf_ = get_parameter("publish_static_markers_tf").as_bool();
    detect_individual_markers_ = get_parameter("detect_individual_markers").as_bool();
    put_markers_count_to_covariance_ = get_parameter("put_markers_count_to_covariance").as_bool();

    world_frame_id_param_  = get_parameter("world_frame_id").as_string();
    base_link_frame_id_    = get_parameter("base_link_frame_id").as_string();
    camera_frame_id_param_ = get_parameter("camera_frame_id").as_string();

    known_vertical_frame_  = get_parameter("known_vertical_frame").as_string();
    flip_vertical_         = get_parameter("flip_vertical").as_bool();
    auto_flip_             = get_parameter("auto_flip").as_bool();

    single_prior_enabled_        = get_parameter("single.prior_enabled").as_bool();
    single_tf_timeout_sec_       = get_parameter("single.tf_timeout_sec").as_double();
    single_gate_tilt_deg_        = get_parameter("single.gate_tilt_deg").as_double();
    single_gate_yaw_deg_         = get_parameter("single.gate_yaw_deg").as_double();
    single_prior_tilt_sigma_deg_ = get_parameter("single.prior_tilt_sigma_deg").as_double();
    single_yaw_sigma_deg_        = get_parameter("single.yaw_sigma_deg").as_double();
    single_rp_variance_          = get_parameter("single.rp_variance").as_double();
    single_corner_sigma_px_      = get_parameter("single.corner_sigma_px").as_double();

    refine_pose_           = get_parameter("refine_pose").as_bool();
    max_inlier_rmse_px_    = get_parameter("gating.max_inlier_rmse_px").as_double();
    enable_timing_log_     = get_parameter("enable_timing_log").as_bool();

    radial_weights_.enabled   = get_parameter("weights.radial_enabled").as_bool();
    radial_weights_.sigma0_px = get_parameter("weights.sigma0_px").as_double();
    radial_weights_.beta      = get_parameter("weights.beta").as_double();
    radial_weights_.q         = get_parameter("weights.q").as_double();

    // PnP parameters
    pnp_params_.iterations_count      = get_parameter("pnp.iterations").as_int();
    pnp_params_.reprojection_error_px = static_cast<float>(get_parameter("pnp.reprojection_error_px").as_double());
    pnp_params_.confidence            = get_parameter("pnp.confidence").as_double();
    pnp_params_.flags                 = get_parameter("pnp.flags").as_int();
    pnp_params_.min_used_markers      = get_parameter("pnp.min_used_markers").as_int();
    pnp_params_.min_inlier_points     = get_parameter("pnp.min_inlier_points").as_int();

    // Apply CPU affinity to this node's thread if cores are specified.
    {
      const auto cpu_ids = get_parameter("cpu_affinity_ids").as_integer_array();
      if (!cpu_ids.empty()) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (const auto id : cpu_ids) {
          if (id >= 0 && id < CPU_SETSIZE) CPU_SET(static_cast<int>(id), &cpuset);
        }
        if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
          RCLCPP_WARN(get_logger(), "sched_setaffinity failed: %s", std::strerror(errno));
        } else {
          RCLCPP_INFO(get_logger(), "aruco_loc: thread pinned to %zu CPU core(s)", cpu_ids.size());
        }
      }
    }

    // TF broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Publishers
    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);

    // Timing publisher — always created; only populated when enable_timing_log is true.
    // Общий /aruco/timing с aruco_detect; поле "node" в JSON различает источники.
    timing_pub_ = create_publisher<std_msgs::msg::String>("/aruco/timing", rclcpp::SystemDefaultsQoS());

    // Subscriptions
    // camera_info is RELIABLE + transient_local from the driver (often latched).
    // Match it so we still receive intrinsics when joining after the single
    // latched publication; a volatile sub would silently miss it.
    {
      auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, camera_info_qos,
        std::bind(&ArucoLocNode::cameraInfoCb, this, std::placeholders::_1));
    }

    // Map subscription (latched)
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      map_markers_sub_ = create_subscription<MarkerArrayMsg>(
        map_markers_topic_, qos,
        std::bind(&ArucoLocNode::mapMarkersCb, this, std::placeholders::_1));
    }

    // Detections subscription
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
      markers_sub_ = create_subscription<MarkerArrayMsg>(
        markers_topic_, qos,
        std::bind(&ArucoLocNode::markersCb, this, std::placeholders::_1));
    }

    // Dynamic parameters
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ArucoLocNode::onParams, this, std::placeholders::_1));

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    dist_coeffs_   = cv::Mat();

    RCLCPP_INFO(get_logger(), "aruco_loc_node ready (timing=%s)",
      enable_timing_log_ ? "ON" : "off");
  }

private:
  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;

    for (const auto& p : params) {
      const auto& name = p.get_name();

      if (name == "enabled") enabled_ = p.as_bool();
      else if (name == "known_vertical_frame") known_vertical_frame_ = p.as_string();
      else if (name == "flip_vertical") flip_vertical_ = p.as_bool();
      else if (name == "auto_flip") auto_flip_ = p.as_bool();
      else if (name == "single.prior_enabled") single_prior_enabled_ = p.as_bool();
      else if (name == "single.tf_timeout_sec") single_tf_timeout_sec_ = p.as_double();
      else if (name == "single.gate_tilt_deg") single_gate_tilt_deg_ = p.as_double();
      else if (name == "single.gate_yaw_deg") single_gate_yaw_deg_ = p.as_double();
      else if (name == "single.prior_tilt_sigma_deg") single_prior_tilt_sigma_deg_ = p.as_double();
      else if (name == "single.yaw_sigma_deg") single_yaw_sigma_deg_ = p.as_double();
      else if (name == "single.rp_variance") single_rp_variance_ = p.as_double();
      else if (name == "single.corner_sigma_px") single_corner_sigma_px_ = p.as_double();
      else if (name == "refine_pose") refine_pose_ = p.as_bool();
      else if (name == "world_frame_id") world_frame_id_param_ = p.as_string();
      else if (name == "base_link_frame_id") base_link_frame_id_ = p.as_string();
      else if (name == "camera_frame_id") camera_frame_id_param_ = p.as_string();

      else if (name == "pnp.iterations") pnp_params_.iterations_count = p.as_int();
      else if (name == "pnp.reprojection_error_px") pnp_params_.reprojection_error_px = static_cast<float>(p.as_double());
      else if (name == "pnp.confidence") pnp_params_.confidence = p.as_double();
      else if (name == "pnp.flags") pnp_params_.flags = p.as_int();
      else if (name == "pnp.min_used_markers") pnp_params_.min_used_markers = p.as_int();
      else if (name == "pnp.min_inlier_points") pnp_params_.min_inlier_points = p.as_int();
      else if (name == "gating.max_inlier_rmse_px") max_inlier_rmse_px_ = p.as_double();

      else if (name == "weights.radial_enabled") radial_weights_.enabled = p.as_bool();
      else if (name == "weights.sigma0_px") radial_weights_.sigma0_px = p.as_double();
      else if (name == "weights.beta") radial_weights_.beta = p.as_double();
      else if (name == "weights.q") radial_weights_.q = p.as_double();

      // TF parameters
      else if (name == "publish_tf") publish_tf_ = p.as_bool();
      else if (name == "publish_static_markers_tf") publish_static_markers_tf_ = p.as_bool();
      else if (name == "detect_individual_markers") detect_individual_markers_ = p.as_bool();
      else if (name == "put_markers_count_to_covariance") put_markers_count_to_covariance_ = p.as_bool();

      // Timing
      else if (name == "enable_timing_log") enable_timing_log_ = p.as_bool();

      // Topics (runtime change)
      else if (name == "markers_topic") {
        markers_topic_ = p.as_string();
      } else if (name == "map_markers_topic") {
        map_markers_topic_ = p.as_string();
      } else if (name == "camera_info_topic") {
        camera_info_topic_ = p.as_string();
      } else if (name == "pose_topic") {
        pose_topic_ = p.as_string();
      }
    }

    return r;
  }

  std::array<double, 36> covarianceToRosArray(const cv::Mat& covariance) const
  {
    std::array<double, 36> cov_array{};
    cov_array.fill(0.0);

    if (covariance.rows == 6 && covariance.cols == 6) {
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          cov_array[i * 6 + j] = covariance.at<double>(i, j);
        }
      }
    }
    return cov_array;
  }

  // Analytical Jacobian covariance using OpenCV's built-in derivative output.
  //
  // Old approach: numerical finite-differences — 12 cv::projectPoints calls per frame
  //   + allocation of a (2N × 2N) identity matrix R, then J^T R J chain.
  //
  // New approach: one cv::projectPoints call that returns the analytical Jacobian
  //   (columns 0-2 = d/d(rvec), columns 3-5 = d/d(tvec)).
  //   The formula J^T R J where R = σ²·I simplifies to σ² · J^T J, so R is
  //   never materialised — just a scalar.  Result: σ² · (J^T J + reg)^{-1}.
  //
  //   Speedup: 12× fewer projectPoints, no O(N²) matrix allocation.
  cv::Mat estimateCovarianceFromJacobian(
      const cv::Vec3d& rvec,
      const cv::Vec3d& tvec,
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& /*image_points*/,  // kept for API compat
      double pixel_error = 0.5,
      double /*epsilon*/ = 1e-6)                         // kept for API compat
  {
    const int num_points = static_cast<int>(object_points.size());
    if (num_points < 3) {
      return cv::Mat::zeros(6, 6, CV_64F);
    }

    // One projectPoints call → analytical Jacobian (2N × 14 or 2N × 10).
    // Column layout: [rvec(0-2), tvec(3-5), intrinsics/distortion(6+)]
    std::vector<cv::Point2f> proj_dummy;
    cv::Mat jac_full;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_,
                      proj_dummy, jac_full);

    if (jac_full.cols < 6) {
      // Fallback: degenerate calibration, return zeros
      return cv::Mat::zeros(6, 6, CV_64F);
    }

    // Reorder columns to ROS PoseWithCovarianceStamped convention [tx,ty,tz, rx,ry,rz].
    // OpenCV: cols 0-2 = d/d(rvec), cols 3-5 = d/d(tvec)
    cv::Mat J(2 * num_points, 6, CV_64F);
    jac_full.col(3).copyTo(J.col(0));  // d/d(tvec_x)
    jac_full.col(4).copyTo(J.col(1));  // d/d(tvec_y)
    jac_full.col(5).copyTo(J.col(2));  // d/d(tvec_z)
    jac_full.col(0).copyTo(J.col(3));  // d/d(rvec_x)
    jac_full.col(1).copyTo(J.col(4));  // d/d(rvec_y)
    jac_full.col(2).copyTo(J.col(5));  // d/d(rvec_z)

    // Fisher Information Matrix: Σ^{-1} = (1/σ²) · J^T J
    // Covariance: Σ = σ² · (J^T J + reg)^{-1}   [no large R matrix needed]
    const cv::Mat JtJ = J.t() * J;
    const cv::Mat JtJ_reg = JtJ + cv::Mat::eye(6, 6, CV_64F) * 1e-6;

    cv::Mat JtJ_inv;
    cv::invert(JtJ_reg, JtJ_inv, cv::DECOMP_SVD);

    return JtJ_inv * (pixel_error * pixel_error);
  }

  // Взвешенная ковариация позы: Σ = s²·(JᵀWJ)⁻¹, W = diag(1/σᵢ²) — та же
  // радиальная модель σ(r), что и в IRLS. Краевые точки честно раздувают
  // ковариацию вместо равного вклада. s² = max(1, χ²/dof) — страховка от
  // переоптимизма, когда фактические остатки хуже модели (смаз, плохой кадр);
  // при EKF2_EV_NOISE_MD=0 это доезжает до EKF2 поверх флора EVP_NOISE.
  cv::Mat estimateCovarianceWeighted(
      const cv::Vec3d& rvec,
      const cv::Vec3d& tvec,
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& image_points,
      const std::vector<double>& sigma_px)
  {
    const int n = static_cast<int>(object_points.size());
    if (n < 3 ||
        image_points.size() != object_points.size() ||
        sigma_px.size() != object_points.size()) {
      return cv::Mat::zeros(6, 6, CV_64F);
    }

    std::vector<cv::Point2f> proj;
    cv::Mat jac_full;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_,
                      proj, jac_full);
    if (jac_full.cols < 6 || jac_full.rows != 2 * n) {
      return cv::Mat::zeros(6, 6, CV_64F);
    }

    // Порядок ROS [tx,ty,tz, rx,ry,rz]; строки масштабируются 1/σᵢ, тогда
    // JᵀJ масштабированной матрицы = JᵀWJ, а χ² = Σ(resᵢ/σᵢ)².
    cv::Mat J(2 * n, 6, CV_64F);
    jac_full.col(3).copyTo(J.col(0));
    jac_full.col(4).copyTo(J.col(1));
    jac_full.col(5).copyTo(J.col(2));
    jac_full.col(0).copyTo(J.col(3));
    jac_full.col(1).copyTo(J.col(4));
    jac_full.col(2).copyTo(J.col(5));

    double chi2 = 0.0;
    for (int i = 0; i < n; ++i) {
      const double inv_s = 1.0 / sigma_px[i];
      J.row(2 * i)     *= inv_s;
      J.row(2 * i + 1) *= inv_s;
      const double rx = (static_cast<double>(proj[i].x) - image_points[i].x) * inv_s;
      const double ry = (static_cast<double>(proj[i].y) - image_points[i].y) * inv_s;
      chi2 += rx * rx + ry * ry;
    }
    const int dof = std::max(1, 2 * n - 6);
    const double s2 = std::max(1.0, chi2 / dof);

    const cv::Mat JtWJ = J.t() * J + cv::Mat::eye(6, 6, CV_64F) * 1e-6;
    cv::Mat JtWJ_inv;
    cv::invert(JtWJ, JtWJ_inv, cv::DECOMP_SVD);
    return JtWJ_inv * s2;
  }

  cv::Mat estimateCovarianceFromReprojectionError(
      const cv::Vec3d& rvec,
      const cv::Vec3d& tvec,
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& image_points,
      double reprojection_error)
  {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

    double total_error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
      const double dx = image_points[i].x - projected_points[i].x;
      const double dy = image_points[i].y - projected_points[i].y;
      total_error += dx * dx + dy * dy;
    }
    const double rmse = std::sqrt(total_error / std::max<size_t>(1, image_points.size()));

    const double effective_error = (reprojection_error > 0.0) ? reprojection_error : rmse;

    const double position_uncertainty    = effective_error * 0.01;
    const double orientation_uncertainty = effective_error * 0.001;

    cv::Mat covariance = cv::Mat::zeros(6, 6, CV_64F);
    covariance.at<double>(0, 0) = position_uncertainty * position_uncertainty;
    covariance.at<double>(1, 1) = position_uncertainty * position_uncertainty;
    covariance.at<double>(2, 2) = position_uncertainty * position_uncertainty;

    covariance.at<double>(3, 3) = orientation_uncertainty * orientation_uncertainty;
    covariance.at<double>(4, 4) = orientation_uncertainty * orientation_uncertainty;
    covariance.at<double>(5, 5) = orientation_uncertainty * orientation_uncertainty;

    return covariance;
  }

  cv::Mat estimatePoseCovariance(
      const cv::Vec3d& rvec,
      const cv::Vec3d& tvec,
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& image_points,
      int num_markers,
      double reprojection_error = -1.0)
  {
    cv::Mat covariance;

    if (image_points.size() >= 6 && object_points.size() >= 6) {
      try {
        double pixel_error = 0.5;
        if (reprojection_error > 0.0) {
          pixel_error = std::max(0.5, reprojection_error);
        }
        if (num_markers > 0) {
          pixel_error /= std::sqrt(static_cast<double>(num_markers));
        }
        pixel_error = std::max(0.1, std::min(pixel_error, 5.0));

        covariance = estimateCovarianceFromJacobian(rvec, tvec, object_points, image_points, pixel_error);
      } catch (const cv::Exception& e) {
        RCLCPP_WARN(get_logger(), "Jacobian covariance failed: %s", e.what());
        covariance = cv::Mat::zeros(6, 6, CV_64F);
      }
    } else {
      covariance = estimateCovarianceFromReprojectionError(rvec, tvec, object_points, image_points, reprojection_error);
    }

    covariance += cv::Mat::eye(6, 6, CV_64F) * 1e-6;
    return covariance;
  }

  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
  {
    last_camera_info_ = msg;
    if (!camera_ready_) {
      loc_utils::parseCameraInfo(*msg, camera_matrix_, dist_coeffs_);
      image_width_  = static_cast<int>(msg->width);
      image_height_ = static_cast<int>(msg->height);
      // Fisheye/equidistant support. cv::solvePnP/projectPoints only understand the
      // plumb_bob (radial-tangential) model, so feeding them the 4 equidistant
      // coefficients would silently misread them as [k1,k2,p1,p2] and wreck the pose
      // (~17 px reproj vs ~2 px). Instead: keep K, stash the fisheye coefficients,
      // and clear dist_coeffs_. The detected corners are then undistorted to ideal
      // pinhole-K pixels in markersCb (cv::fisheye::undistortPoints), after which the
      // whole pipeline runs with zero distortion and stays self-consistent.
      is_fisheye_ = (msg->distortion_model == "equidistant" && dist_coeffs_.total() == 4);
      if (is_fisheye_) {
        fisheye_D_ = dist_coeffs_.clone();
        dist_coeffs_ = cv::Mat();
        RCLCPP_INFO(get_logger(),
          "Camera calibration loaded (fisheye/equidistant, corners undistorted before PnP)");
      } else {
        if (msg->distortion_model == "equidistant") {
          RCLCPP_WARN(get_logger(),
            "equidistant model needs 4 coeffs, got %zu — falling back to plumb_bob path",
            dist_coeffs_.total());
        }
        RCLCPP_INFO(get_logger(), "Camera calibration loaded (%s)",
                    msg->distortion_model.c_str());
      }
      camera_ready_ = true;
    }
  }

  void mapMarkersCb(const MarkerArrayMsg::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lk(map_mutex_);

    map_frame_id_ = msg->header.frame_id;

    loc_utils::buildMapIndex(*msg, map_index_);
    map_ready_ = !map_index_.empty();

    if (!map_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Map received but no valid markers");
      return;
    }

    if (publish_static_markers_tf_ && detect_individual_markers_) {
      publishStaticMarkerTransforms(*msg);
    }
  }

  void publishStaticMarkerTransforms(const MarkerArrayMsg& map_msg)
  {
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    static_transforms.reserve(map_msg.markers.size());

    for (const auto& marker : map_msg.markers) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = tf_map_frame_;
      transform.child_frame_id = tf_markers_prefix_ + std::to_string(marker.id);

      transform.transform.translation.x = marker.pose.position.x;
      transform.transform.translation.y = marker.pose.position.y;
      transform.transform.translation.z = marker.pose.position.z;
      transform.transform.rotation = marker.pose.orientation;

      static_transforms.push_back(transform);
    }

    if (!static_transforms.empty()) {
      static_tf_broadcaster_->sendTransform(static_transforms);
    }
  }

  void markersCb(const MarkerArrayMsg::ConstSharedPtr& det_msg)
  {
    // ── Timing: callback entry ───────────────────────────────────────────────
    const TimePoint t_cb = Clock::now();

    // Wall time relative to node start (seconds), for cross-node correlation
    const double wall_now = ms(node_start_, t_cb) * 1e-3;

    // Message age: how long since the image was stamped (ROS time → wall δ)
    const double img_stamp = det_msg->header.stamp.sec
                           + det_msg->header.stamp.nanosec * 1e-9;
    const double ros_now   = this->now().seconds();
    const double markers_age_ms = (ros_now - img_stamp) * 1e3;

    const uint32_t t_seq = ++timing_seq_;
    // ────────────────────────────────────────────────────────────────────────

    if (!enabled_ || !camera_ready_) {
      return;
    }

    // Map copy under mutex
    loc_utils::MapIndex map_copy;
    std::string map_frame_copy;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      if (!map_ready_) {
        return;
      }
      map_copy = map_index_;
      map_frame_copy = map_frame_id_;
    }

    const TimePoint t_map = Clock::now();

    const std::string world_frame =
      !world_frame_id_param_.empty() ? world_frame_id_param_
                                     : (!map_frame_copy.empty() ? map_frame_copy : std::string("map"));

    const std::string camera_frame =
      !camera_frame_id_param_.empty() ? camera_frame_id_param_
                                      : (!det_msg->header.frame_id.empty()
                                           ? det_msg->header.frame_id
                                           : (last_camera_info_ && !last_camera_info_->header.frame_id.empty()
                                                ? last_camera_info_->header.frame_id
                                                : std::string("camera_optical_1")));

    (void)camera_frame;

    // 3D–2D correspondences for PnP
    auto corr = loc_utils::buildCorrespondences(*det_msg, map_copy);

    // Радиальная σ(r) на точку — по ИСХОДНЫМ (дисторченным) пикселям, поэтому
    // считается ДО fisheye-ректификации. Порядок совпадает с corr.
    std::vector<double> point_sigma_px;
    if (radial_weights_.enabled && !corr.image_points_px.empty()) {
      const double cx = camera_matrix_.at<double>(0, 2);
      const double cy = camera_matrix_.at<double>(1, 2);
      const double w  = image_width_  > 0 ? image_width_  : 2.0 * cx;
      const double h  = image_height_ > 0 ? image_height_ : 2.0 * cy;
      const double half_diag = 0.5 * std::hypot(w, h);
      point_sigma_px.reserve(corr.image_points_px.size());
      for (const auto& p : corr.image_points_px) {
        const double r = std::hypot(p.x - cx, p.y - cy) / half_diag;
        point_sigma_px.push_back(radial_weights_.sigmaPx(r));
      }
    }

    // Fisheye: rectify the detected corners to ideal pinhole-K pixels so the rest of
    // the pipeline (solvePnP/RANSAC, reprojection, covariance) runs with zero
    // distortion. No-op for the plumb_bob path (is_fisheye_ == false).
    if (is_fisheye_ && !corr.image_points_px.empty()) {
      std::vector<cv::Point2f> undistorted;
      cv::fisheye::undistortPoints(corr.image_points_px, undistorted,
                                   camera_matrix_, fisheye_D_,
                                   cv::noArray(), camera_matrix_);
      corr.image_points_px = std::move(undistorted);
    }

    const TimePoint t_corr = Clock::now();

    const int n_corr_points  = static_cast<int>(corr.object_points_world.size());
    const int n_used_markers = static_cast<int>(corr.used_marker_ids.size());

    if (n_corr_points < 4) {
      return;
    }

    // Detections present on map
    int valid_markers_count = 0;
    for (const auto& marker : det_msg->markers) {
      if (map_copy.find(marker.id) != map_copy.end()) {
        valid_markers_count++;
      }
    }

    // Pose estimate: RANSAC for multi-marker, IPPE_SQUARE for one marker.
    loc_utils::PnPResult pnp;
    if (!loc_utils::estimatePosePnPRansac(corr, camera_matrix_, dist_coeffs_, pnp_params_, pnp)) {
      return;
    }

    const TimePoint t_pnp = Clock::now();

    // Уточнение позы после RANSAC.
    // С радиальными весами — IRLS на инлаерах: краевые точки участвуют, но с
    // весом 1/σ(r)² (+Huber от остаточных промахов). Иначе — прежний plain refine.
    bool irls_applied = false;
    std::vector<cv::Point3f> inlier_obj;
    std::vector<cv::Point2f> inlier_img;
    std::vector<double>      inlier_sigma;
    if (radial_weights_.enabled && pnp.method == loc_utils::PnPMethod::Ransac &&
        point_sigma_px.size() == corr.image_points_px.size()) {
      inlier_obj.reserve(pnp.inlier_indices.size());
      inlier_img.reserve(pnp.inlier_indices.size());
      inlier_sigma.reserve(pnp.inlier_indices.size());
      for (const int idx : pnp.inlier_indices) {
        inlier_obj.push_back(corr.object_points_world[idx]);
        inlier_img.push_back(corr.image_points_px[idx]);
        inlier_sigma.push_back(point_sigma_px[idx]);
      }
      irls_applied = loc_utils::refinePoseIrlsWeighted(
        inlier_obj, inlier_img, inlier_sigma,
        camera_matrix_, dist_coeffs_, pnp.rvec_cw, pnp.tvec_cw);
      if (irls_applied) {
        // RMSE пересчитывается после сдвига позы — гейтинг ниже работает по нему.
        pnp.reproj_rmse_inliers = loc_utils::reprojectionRmse(
          corr.object_points_world, corr.image_points_px,
          camera_matrix_, dist_coeffs_,
          pnp.rvec_cw, pnp.tvec_cw, &pnp.inlier_indices);
      }
    }
    if (!irls_applied && refine_pose_ && pnp.method == loc_utils::PnPMethod::Ransac) {
      (void)loc_utils::refinePoseSolvePnP(
        corr, camera_matrix_, dist_coeffs_,
        pnp.rvec_cw, pnp.tvec_cw,
        static_cast<int>(cv::SOLVEPNP_ITERATIVE));
    }

    const TimePoint t_refine = Clock::now();

    // Inlier RMSE gating
    const bool gating_passed = !(max_inlier_rmse_px_ > 0.0 &&
                                  std::isfinite(pnp.reproj_rmse_inliers) &&
                                  pnp.reproj_rmse_inliers > max_inlier_rmse_px_);
    if (!gating_passed) {
      return;
    }

    // world→camera → world←camera
    const bool single_marker_pose =
      pnp.method == loc_utils::PnPMethod::SingleMarkerIppeSquare;
    if (single_marker_pose && known_vertical_frame_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "single-marker IPPE localization without known_vertical_frame is unreliable; "
        "configure known_vertical_frame with auto_flip to suppress planar flip ambiguity");
    }

    const tf2::Transform T_camera_world =
      loc_utils::tfFromRvecTvec_worldToCamera(pnp.rvec_cw, pnp.tvec_cw);
    tf2::Transform T_world_camera = T_camera_world.inverse();

    // Optional known-vertical blend — ТОЛЬКО для одномаркерной позы (IPPE_SQUARE),
    // где реальна планарная flip-неоднозначность. Для мультимаркерного RANSAC
    // (>=2 меток) сырая PnP-ориентация уже хорошо обусловлена и корректна, а бленд
    // её ПОРТИТ: applyKnownVerticalKeepYaw выбрасывает ArUco-ориентацию и
    // подставляет производную от EKF base_link с auto_flip (180° вокруг X).
    // isFlipped() вырождена для смотрящей вниз камеры (|roll| или |pitch| > 90°
    // почти всегда), из-за чего в мультимаркерном режиме публиковалась
    // перевёрнутая ориентация (origin оказывался ЗА камерой → "world axes:
    // behind camera"), хотя reproj_rmse (считается ВЫШЕ по коду) оставался ~4px.
    //
    // P1′ (single.prior_enabled): бленд чинит только ориентацию, а ПОЗИЦИЯ от
    // свободного PnP несёт весь вред (rolling-shutter шер → наклон → ×h, зеркальная
    // IPPE-гипотеза). Поэтому при включённом prior: (1) TF attitude берётся на
    // stamp кадра, не последний; (2) кадры с расхождением PnP↔prior гейтятся;
    // (3) после бленда трансляция решается заново линейной LSQ при известной
    // ориентации. Детали и цифры: SINGLE_MARKER_JITTER_ANALYSIS.md, часть 2.
    bool tf_ok = true;
    bool single_prior_used = false;      // t-only решение применено
    bool prior_stamp_aligned = false;    // TF взят на stamp кадра (не fallback)
    cv::Mat single_pos_cov;              // 3x3 позиционная ковариация t-only LSQ
    if (single_marker_pose && !known_vertical_frame_.empty()) {
      try {
        geometry_msgs::msg::TransformStamped vertical_tf;
        if (single_prior_enabled_) {
          // Attitude на stamp кадра (буфер интерполирует). Последний TF в полёте
          // отстаёт от кадра на 30-70 мс: при 30-60°/с это 1-4° ошибки prior →
          // h·tan() = 3-10 см сноса позиции. Fallback на последний — если
          // стемпованный ещё не доехал (старт, дыра в одометрии).
          try {
            vertical_tf = tf_buffer_.lookupTransform(
              world_frame, known_vertical_frame_,
              rclcpp::Time(det_msg->header.stamp),
              rclcpp::Duration::from_seconds(std::max(0.0, single_tf_timeout_sec_)));
            prior_stamp_aligned = true;
          } catch (const tf2::TransformException&) {
            vertical_tf =
              tf_buffer_.lookupTransform(world_frame, known_vertical_frame_, tf2::TimePointZero);
          }
        } else {
          vertical_tf =
            tf_buffer_.lookupTransform(world_frame, known_vertical_frame_, tf2::TimePointZero);
        }

        tf2::Transform T_world_vertical;
        tf2::fromMsg(vertical_tf.transform, T_world_vertical);

        tf2::Transform T_vertical_camera;
        T_vertical_camera.setIdentity();

        if (known_vertical_frame_ != camera_frame) {
          const auto vc_tf =
            tf_buffer_.lookupTransform(known_vertical_frame_, camera_frame, tf2::TimePointZero);
          tf2::fromMsg(vc_tf.transform, T_vertical_camera);
        }

        const tf2::Transform T_world_camera_from_vertical = T_world_vertical * T_vertical_camera;
        tf2::Quaternion q_wc = T_world_camera.getRotation();
        tf2::Quaternion q_wcam_v = T_world_camera_from_vertical.getRotation();

        // P2′-выбор гипотезы + гейт наклона. Важно: prior живёт в EKF-фрейме,
        // PnP — во фрейме карты меток, их взаимный yaw произволен (курс EKF
        // случаен на каждом старте), поэтому сырые оптические оси НЕ сравнимы
        // (азимут наклона развёрнут на этот yaw — «ложные 20°+»). KeepYaw как
        // раз выравнивает yaw prior'а к yaw гипотезы, т.е. переносит prior в
        // фрейм карты. Корректная метрика: угол между осью гипотезы и осью ЕЁ
        // ЖЕ бленда. Ею же выбирается гипотеза (min-RMSE-селектор в
        // около-надирной геометрии может брать зеркальную ветвь — ч.2.3).
        if (single_prior_enabled_) {
          std::vector<loc_utils::SingleMarkerHypothesis> hyps;
          if (!loc_utils::estimateSingleMarkerHypotheses(
                corr, camera_matrix_, dist_coeffs_, hyps) || hyps.empty()) {
            hyps.push_back({pnp.rvec_cw, pnp.tvec_cw, pnp.reproj_rmse_inliers});
          }

          size_t best_i = 0;
          double best_dev_deg = std::numeric_limits<double>::infinity();
          tf2::Transform best_T_wc;
          tf2::Quaternion best_q_blend;
          for (size_t i = 0; i < hyps.size(); ++i) {
            const tf2::Transform T_wc_i =
              loc_utils::tfFromRvecTvec_worldToCamera(hyps[i].rvec_cw, hyps[i].tvec_cw).inverse();
            const tf2::Quaternion q_i = T_wc_i.getRotation();
            const tf2::Quaternion q_blend_i =
              loc_utils::transportPriorKeepHeading(q_i, q_wcam_v);
            const tf2::Vector3 z_hyp   = tf2::Matrix3x3(q_i).getColumn(2);
            const tf2::Vector3 z_blend = tf2::Matrix3x3(q_blend_i).getColumn(2);
            const double dev_deg =
              std::acos(std::clamp(z_hyp.dot(z_blend), -1.0, 1.0)) * 180.0 / M_PI;
            if (dev_deg < best_dev_deg) {
              best_dev_deg = dev_deg;
              best_i = i;
              best_T_wc = T_wc_i;
              best_q_blend = q_blend_i;
            }
          }

          if (single_gate_tilt_deg_ > 0.0 && best_dev_deg > single_gate_tilt_deg_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "single-marker pose gated: best hypothesis tilt dev %.1f deg (max %.1f, %zu hyps)",
              best_dev_deg, single_gate_tilt_deg_, hyps.size());
            return;
          }

          // Принятая гипотеза: позиция гипотезы (уточнится t-only ниже),
          // ориентация — её бленд (roll/pitch prior + ArUco yaw).
          pnp.rvec_cw = hyps[best_i].rvec_cw;
          pnp.tvec_cw = hyps[best_i].tvec_cw;
          pnp.reproj_rmse_inliers = hyps[best_i].rmse;
          T_world_camera = best_T_wc;
          T_world_camera.setRotation(best_q_blend);
          q_wc = best_q_blend;

          // Гейт yaw: временна́я самосогласованность ArUco-yaw (сравнение с
          // последним принятым одномаркерным кадром — общий фрейм, корректно).
          // По умолчанию выключен (0.0) — включать после проверки на стенде.
          if (single_gate_yaw_deg_ > 0.0) {
            double r_cur = 0.0, p_cur = 0.0, yaw_cur = 0.0;
            tf2::Matrix3x3(q_wc).getRPY(r_cur, p_cur, yaw_cur);
            const double stamp_sec = rclcpp::Time(det_msg->header.stamp).seconds();
            if (has_last_single_yaw_ && (stamp_sec - last_single_yaw_stamp_) < 2.0) {
              double dy = yaw_cur - last_single_yaw_;
              while (dy > M_PI)  dy -= 2.0 * M_PI;
              while (dy < -M_PI) dy += 2.0 * M_PI;
              const double yaw_dev_deg = std::abs(dy) * 180.0 / M_PI;
              if (yaw_dev_deg > single_gate_yaw_deg_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                  "single-marker pose gated: yaw jump %.1f deg (max %.1f)",
                  yaw_dev_deg, single_gate_yaw_deg_);
                return;
              }
            }
            last_single_yaw_       = yaw_cur;
            last_single_yaw_stamp_ = stamp_sec;
            has_last_single_yaw_   = true;
          }
        } else {
          loc_utils::applyKnownVerticalKeepYaw(q_wc, q_wcam_v, flip_vertical_, auto_flip_);
          T_world_camera.setRotation(q_wc);
        }

        // P1′: трансляция при известной (блендованной) ориентации.
        if (single_prior_enabled_) {
          tf2::Vector3 cam_pos;
          if (loc_utils::solveTranslationKnownOrientation(
                corr.object_points_world, corr.image_points_px,
                camera_matrix_, dist_coeffs_, T_world_camera.getBasis(),
                cam_pos, &single_pos_cov, single_corner_sigma_px_)) {
            T_world_camera.setOrigin(cam_pos);
            single_prior_used = true;
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "single-marker t-only solve failed (cheirality/degenerate); using IPPE position");
          }
        }
      } catch (const tf2::TransformException& e) {
        tf_ok = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "known_vertical lookup failed: %s", e.what());
      }
    }

    if (single_marker_pose && !known_vertical_frame_.empty() && !tf_ok) {
      return;
    }

    const TimePoint t_tf = Clock::now();

    // PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header.stamp = det_msg->header.stamp;
    // frame_id must match the coordinate system in which the pose is expressed.
    // T_world_camera is the camera pose IN world space → frame_id = world frame ("map").
    pose_cov_msg.header.frame_id = world_frame;
    pose_cov_msg.pose.pose = loc_utils::poseMsgFromTf(T_world_camera);

    // Covariance
    const bool using_count_cov = put_markers_count_to_covariance_;
    if (using_count_cov) {
      pose_cov_msg.pose.covariance.fill(0.0);
      pose_cov_msg.pose.covariance[0] = static_cast<double>(valid_markers_count);
    } else {
      cv::Mat pose_covariance;
      bool weighted_cov_ok = false;
      if (single_prior_used && !single_pos_cov.empty()) {
        // P3′: честная одномаркерная ковариация. Позиция — якобиан t-only LSQ +
        // изотропный член (h·tan σ_prior)² на xy (ошибка prior-ориентации δθ
        // смещает позицию на h·δθ). Прежний путь (…FromReprojectionError,
        // σ=rmse·0.01) отдавал мм-ковариацию при реальной дм-ошибке — EKF2
        // доверял мусору и контроллер отрабатывал каждый выброс. roll/pitch
        // публикуются с большой дисперсией: это рециркулированный EKF-attitude,
        // не измерение ArUco; yaw — настоящее измерение, дисперсия честная.
        pose_covariance = cv::Mat::zeros(6, 6, CV_64F);
        single_pos_cov.copyTo(pose_covariance(cv::Rect(0, 0, 3, 3)));
        double mean_obj_z = 0.0;
        for (const auto& op : corr.object_points_world) mean_obj_z += op.z;
        mean_obj_z /= std::max<size_t>(1, corr.object_points_world.size());
        const double h_above =
          std::abs(T_world_camera.getOrigin().z() - mean_obj_z);
        const double tilt_var = std::pow(
          h_above * std::tan(single_prior_tilt_sigma_deg_ * M_PI / 180.0), 2);
        pose_covariance.at<double>(0, 0) += tilt_var;
        pose_covariance.at<double>(1, 1) += tilt_var;
        pose_covariance.at<double>(2, 2) += 1e-4;  // z-флор (1 см)²
        pose_covariance.at<double>(3, 3) = single_rp_variance_;
        pose_covariance.at<double>(4, 4) = single_rp_variance_;
        const double yaw_sigma = single_yaw_sigma_deg_ * M_PI / 180.0;
        pose_covariance.at<double>(5, 5) = yaw_sigma * yaw_sigma;
        weighted_cov_ok = true;  // ковариация собрана — общие пути не нужны
      } else if (irls_applied) {
        pose_covariance = estimateCovarianceWeighted(
          pnp.rvec_cw, pnp.tvec_cw, inlier_obj, inlier_img, inlier_sigma);
        weighted_cov_ok = cv::norm(pose_covariance) > 1e-9;
        if (weighted_cov_ok) {
          pose_covariance += cv::Mat::eye(6, 6, CV_64F) * 1e-6;
        }
      }
      if (!weighted_cov_ok) {
        std::vector<cv::Point3f> object_points_vec(
          corr.object_points_world.begin(),
          corr.object_points_world.end());

        std::vector<cv::Point2f> image_points_vec(
          corr.image_points_px.begin(),
          corr.image_points_px.end());

        const double reprojection_error =
          (std::isfinite(pnp.reproj_rmse_inliers) ? pnp.reproj_rmse_inliers : 1.0) *
          pnp.covariance_confidence_scale;

        pose_covariance = estimatePoseCovariance(
          pnp.rvec_cw, pnp.tvec_cw,
          object_points_vec, image_points_vec,
          valid_markers_count, reprojection_error);
      }

      const auto cov_array = covarianceToRosArray(pose_covariance);
      std::copy(cov_array.begin(), cov_array.end(), pose_cov_msg.pose.covariance.begin());
    }

    const TimePoint t_cov = Clock::now();

    // unique_ptr publish enables IPC zero-copy when inside a composable container
    pose_cov_pub_->publish(
      std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_msg));

    if (publish_tf_ && !tf_child_frame_.empty()) {
      geometry_msgs::msg::TransformStamped camera_map_tf;
      camera_map_tf.header.stamp = det_msg->header.stamp;
      camera_map_tf.header.frame_id = tf_parent_frame_;
      camera_map_tf.child_frame_id = tf_child_frame_;
      camera_map_tf.transform = tf2::toMsg(T_camera_world);

      tf_broadcaster_->sendTransform(camera_map_tf);
    }

    const TimePoint t_pub = Clock::now();

    // ── Publish timing JSON ──────────────────────────────────────────────────
    if (enable_timing_log_) {
      // image_to_pose_ms: total pipeline latency from image capture to pose pub.
      // markers_age_ms already covers: DDS transport + aruco_detect processing.
      // ms(t_cb, t_pub) covers: loc node processing.
      const double image_to_pose_ms = markers_age_ms + ms(t_cb, t_pub);

      const double reproj_rmse = std::isfinite(pnp.reproj_rmse_inliers)
                                   ? pnp.reproj_rmse_inliers
                                   : -1.0;

      // Одномаркерный режим: prior — t-only с TF на stamp кадра; prior_latest —
      // t-only, но TF-фоллбэк на последний; legacy — старый путь (IPPE-позиция);
      // "-" — мультимаркер.
      const char* sm_mode = "-";
      if (single_marker_pose) {
        sm_mode = single_prior_used ? (prior_stamp_aligned ? "prior" : "prior_latest")
                                    : "legacy";
      }

      char buf[1024];
      std::snprintf(buf, sizeof(buf),
        "{"
        "\"node\":\"loc\","
        "\"sm\":\"%s\","
        "\"seq\":%u,"
        "\"image_stamp\":%.6f,"
        "\"wall_now\":%.6f,"
        "\"markers_age_ms\":%.3f,"
        "\"map_copy_ms\":%.3f,"
        "\"corr_ms\":%.3f,"
        "\"pnp_ms\":%.3f,"
        "\"refine_ms\":%.3f,"
        "\"tf_ms\":%.3f,"
        "\"cov_ms\":%.3f,"
        "\"pub_ms\":%.3f,"
        "\"total_ms\":%.3f,"
        "\"image_to_pose_ms\":%.3f,"
        "\"n_markers\":%d,"
        "\"n_corr_points\":%d,"
        "\"n_inliers\":%d,"
        "\"pnp_method\":\"%s\","
        "\"required_inliers\":%d,"
        "\"reproj_rmse\":%.4f,"
        "\"gating_passed\":%s,"
        "\"tf_ok\":%s,"
        "\"put_count_cov\":%s,"
        "\"irls\":%s"
        "}",
        sm_mode,
        t_seq,
        img_stamp,
        wall_now,
        markers_age_ms,
        ms(t_cb,   t_map),
        ms(t_map,  t_corr),
        ms(t_corr, t_pnp),
        ms(t_pnp,  t_refine),
        ms(t_refine, t_tf),
        ms(t_tf,   t_cov),
        ms(t_cov,  t_pub),
        ms(t_cb,   t_pub),
        image_to_pose_ms,
        n_used_markers,
        n_corr_points,
        static_cast<int>(pnp.inlier_indices.size()),
        loc_utils::pnpMethodName(pnp.method),
        pnp.required_inlier_points,
        reproj_rmse,
        gating_passed ? "true" : "false",
        tf_ok        ? "true" : "false",
        using_count_cov ? "true" : "false",
        irls_applied ? "true" : "false");

      std_msgs::msg::String tmsg;
      tmsg.data = buf;
      timing_pub_->publish(tmsg);
    }
    // ────────────────────────────────────────────────────────────────────────
  }

private:
  // Parameters and state
  bool enabled_{true};

  std::string markers_topic_;
  std::string map_markers_topic_;
  std::string camera_info_topic_;

  std::string pose_topic_;

  // TF parameters
  std::string tf_parent_frame_;
  std::string tf_child_frame_;
  std::string tf_map_frame_;
  std::string tf_markers_prefix_;
  bool publish_tf_{true};
  bool publish_static_markers_tf_{true};
  bool detect_individual_markers_{true};
  bool put_markers_count_to_covariance_{false};

  std::string world_frame_id_param_;
  std::string base_link_frame_id_;
  std::string camera_frame_id_param_;

  std::string known_vertical_frame_;
  bool flip_vertical_{false};
  bool auto_flip_{false};

  // Одномаркерный режим с attitude-prior (P1′/P2′/P3′)
  bool single_prior_enabled_{true};
  double single_tf_timeout_sec_{0.03};
  double single_gate_tilt_deg_{20.0};
  double single_gate_yaw_deg_{0.0};
  double single_prior_tilt_sigma_deg_{1.0};
  double single_yaw_sigma_deg_{2.0};
  double single_rp_variance_{1.0};
  double single_corner_sigma_px_{0.5};
  // Временна́я самосогласованность yaw (гейт single.gate_yaw_deg)
  bool has_last_single_yaw_{false};
  double last_single_yaw_{0.0};
  double last_single_yaw_stamp_{0.0};

  bool refine_pose_{false};
  double max_inlier_rmse_px_{10.0};

  // Радиальные веса углов (IRLS + взвешенная ковариация)
  loc_utils::RadialWeightModel radial_weights_;

  // Camera intrinsics
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;          // plumb_bob coeffs; empty when is_fisheye_
  cv::Mat fisheye_D_;            // 4 equidistant coeffs when is_fisheye_
  bool is_fisheye_{false};       // camera calibrated with the fisheye/equidistant model
  bool camera_ready_{false};
  int image_width_{0};           // из camera_info; для нормировки радиуса σ(r)
  int image_height_{0};

  // Marker map cache
  std::mutex map_mutex_;
  loc_utils::MapIndex map_index_;
  bool map_ready_{false};
  std::string map_frame_id_;

  loc_utils::PnPRansacParams pnp_params_;

  // Timing / profiling
  bool enable_timing_log_{false};
  uint32_t timing_seq_{0};
  const TimePoint node_start_{Clock::now()};

  // Subscriptions and publishers
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<MarkerArrayMsg>::SharedPtr map_markers_sub_;
  rclcpp::Subscription<MarkerArrayMsg>::SharedPtr markers_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace aruco_det_loc

// Register as a composable node component.
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_det_loc::ArucoLocNode)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_det_loc::ArucoLocNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
