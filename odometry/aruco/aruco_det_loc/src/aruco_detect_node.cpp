#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <utility>
#include <algorithm>
#include <chrono>

// CPU affinity
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "aruco_det_loc/msg/marker.hpp"
#include "aruco_det_loc/msg/marker_array.hpp"

#include "std_srvs/srv/set_bool.hpp"

#if __has_include("cv_bridge/cv_bridge.h")
#  include "cv_bridge/cv_bridge.h"
#else
#  include "cv_bridge/cv_bridge.hpp"
#endif
#include "image_transport/image_transport.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

// ArUco3 fields (useAruco3Detection, minMarkerLengthRatioOriginalImg)
// were added in OpenCV 4.7.0. Ubuntu 22.04 ships 4.5.4, so guard them.
#define ARUCO3_SUPPORTED \
  (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

namespace aruco_det_loc
{

using MarkerMsg      = aruco_det_loc::msg::Marker;
using MarkerArrayMsg = aruco_det_loc::msg::MarkerArray;
using SetBool        = std_srvs::srv::SetBool;
using Clock          = std::chrono::steady_clock;
using TimePoint      = Clock::time_point;

// Milliseconds between two steady_clock time points
static inline double ms(TimePoint a, TimePoint b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

class ArucoDetectNode : public rclcpp::Node
{
public:
  explicit ArucoDetectNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("aruco_detect_node", options)
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<double>("marker_size", 0.5);
    declare_parameter<bool>("send_tf", true);
    declare_parameter<std::string>("frame_id_prefix", "aruco_");
    declare_parameter<bool>("use_map_markers", true);
    declare_parameter<std::string>("parent_frame_id", "main_camera_optical");
    declare_parameter<bool>("detect_individual_markers", true);
    declare_parameter<bool>("estimate_marker_pose", true);
    declare_parameter<double>("max_detect_fps", 0.0);
    declare_parameter<bool>("pnp_fps_cap_enabled", false);
    declare_parameter<int>("pnp_fps_cap", 10);
    declare_parameter<bool>("publish_debug_image", true);
    declare_parameter<std::string>("debug_image_topic", "/aruco/vis/debug_image");
    declare_parameter<std::string>("debug_service_name", "toggle_debug");
    declare_parameter<bool>("publish_world_debug_image", false);
    declare_parameter<std::string>("world_debug_image_topic", "/aruco/vis/world_debug_image");
    declare_parameter<std::string>("pose_cov_topic", "/aruco/loc/pose_cov");
    declare_parameter<double>("world_axes_length", 0.5);
    declare_parameter<int>("world_axes_thickness", 3);
    declare_parameter<bool>("pose_cov_is_world_to_camera", false);
    declare_parameter<double>("world_pose_max_age_sec", 0.2);
    declare_parameter<bool>("publish_visualization", true);
    declare_parameter<std::string>(
      "image_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image");
    declare_parameter<std::string>(
      "camera_info_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info");
    declare_parameter<std::string>("map_markers_topic", "/aruco/map/markers");
    declare_parameter<int>("dictionary_id", 3);
    this->declare_parameter<std::vector<int64_t>>("size_override.ids", std::vector<int64_t>{});
    this->declare_parameter<std::vector<double>>("size_override.sizes", std::vector<double>{});
    declare_parameter<bool>("use_aruco3_detection", false);
    declare_parameter<double>("aruco3_min_marker_length_ratio", 0.10);
    declare_parameter<std::vector<int64_t>>("marker_id_whitelist", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("cpu_affinity_ids", std::vector<int64_t>{});
    declare_parameter<int>("opencv_num_threads", 2);
    // Per-marker solvePnP control:
    //   - Map markers: NEVER run solvePnP here — aruco_loc_node does global PnP for them.
    //   - Non-map markers: optional, controlled by this flag.
    declare_parameter<bool>("pnp_non_map_markers", false);

    // Coarse-to-fine: detect on downscaled gray, refine corners on full-res
    declare_parameter<bool>("coarse_to_fine_enabled", false);
    declare_parameter<double>("coarse_scale", 0.25);        // downsample factor (0.25 = 320×240 from 1280×960)
    declare_parameter<bool>("corner_refine_pnp_only", false); // true = refine only markers sent to solvePnP
    declare_parameter<int>("subpix_win_size", 6);           // cornerSubPix half-window (covers ±4 px from ×4 scale)

    // Profiling: when true, publishes per-frame JSON timing to /aruco/timing.
    // Enable via:  ros2 param set /aruco_detect enable_timing_log true
    declare_parameter<bool>("enable_timing_log", false);

    enabled_                   = get_parameter("enabled").as_bool();
    default_marker_size_       = get_parameter("marker_size").as_double();
    send_tf_                   = get_parameter("send_tf").as_bool();
    frame_id_prefix_           = get_parameter("frame_id_prefix").as_string();
    use_map_markers_           = get_parameter("use_map_markers").as_bool();
    detect_individual_markers_ = get_parameter("detect_individual_markers").as_bool();
    parent_frame_id_           = get_parameter("parent_frame_id").as_string();
    if (parent_frame_id_.empty()) parent_frame_id_ = "main_camera_optical";
    estimate_marker_pose_      = get_parameter("estimate_marker_pose").as_bool();
    max_detect_fps_            = get_parameter("max_detect_fps").as_double();
    if (!std::isfinite(max_detect_fps_)) max_detect_fps_ = 0.0;
    pnp_fps_cap_enabled_       = get_parameter("pnp_fps_cap_enabled").as_bool();
    pnp_fps_cap_               = get_parameter("pnp_fps_cap").as_int();
    if (pnp_fps_cap_ <= 0) pnp_fps_cap_ = 1;
    publish_debug_image_         = get_parameter("publish_debug_image").as_bool();
    debug_image_topic_           = get_parameter("debug_image_topic").as_string();
    debug_service_name_          = get_parameter("debug_service_name").as_string();
    publish_world_debug_image_   = get_parameter("publish_world_debug_image").as_bool();
    world_debug_image_topic_     = get_parameter("world_debug_image_topic").as_string();
    pose_cov_topic_              = get_parameter("pose_cov_topic").as_string();
    world_axes_length_           = get_parameter("world_axes_length").as_double();
    world_axes_thickness_        = get_parameter("world_axes_thickness").as_int();
    pose_cov_is_world_to_camera_ = get_parameter("pose_cov_is_world_to_camera").as_bool();
    world_pose_max_age_sec_      = get_parameter("world_pose_max_age_sec").as_double();
    publish_visualization_     = get_parameter("publish_visualization").as_bool();
    image_topic_       = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    map_markers_topic_ = get_parameter("map_markers_topic").as_string();
    dictionary_id_               = get_parameter("dictionary_id").as_int();
    use_aruco3_detection_        = get_parameter("use_aruco3_detection").as_bool();
    aruco3_min_marker_length_ratio_ = get_parameter("aruco3_min_marker_length_ratio").as_double();
    pnp_non_map_markers_         = get_parameter("pnp_non_map_markers").as_bool();
    coarse_to_fine_enabled_      = get_parameter("coarse_to_fine_enabled").as_bool();
    coarse_scale_                = get_parameter("coarse_scale").as_double();
    corner_refine_pnp_only_      = get_parameter("corner_refine_pnp_only").as_bool();
    subpix_win_size_             = get_parameter("subpix_win_size").as_int();
    enable_timing_log_           = get_parameter("enable_timing_log").as_bool();

    if (default_marker_size_ <= 0.0) default_marker_size_ = 0.16;
    if (!(world_axes_length_ > 0.0) || !std::isfinite(world_axes_length_)) world_axes_length_ = 0.5;
    if (world_axes_thickness_ <= 0) world_axes_thickness_ = 3;

    // CPU affinity
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
          RCLCPP_INFO(get_logger(), "aruco_detect: thread pinned to %zu CPU core(s)", cpu_ids.size());
        }
      }
      const int ocv_threads = get_parameter("opencv_num_threads").as_int();
      if (ocv_threads >= 0) {
        cv::setNumThreads(ocv_threads);
        if (ocv_threads > 0) ::setenv("OMP_NUM_THREADS", std::to_string(ocv_threads).c_str(), 1);
        RCLCPP_INFO(get_logger(), "aruco_detect: OpenCV threads set to %d", ocv_threads);
      }
    }

    try {
      dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id_));
    } catch (...) {
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->adaptiveThreshWinSizeMin  = 5;
    detector_params_->adaptiveThreshWinSizeMax  = 13;
    detector_params_->adaptiveThreshWinSizeStep = 4;
#if ARUCO3_SUPPORTED
    detector_params_->useAruco3Detection = use_aruco3_detection_;
    detector_params_->minMarkerLengthRatioOriginalImg =
      use_aruco3_detection_ ? static_cast<float>(aruco3_min_marker_length_ratio_) : 0.0f;
#else
    if (use_aruco3_detection_) {
      RCLCPP_WARN(get_logger(),
        "use_aruco3_detection=true ignored: OpenCV %d.%d.%d < 4.7 (no ArUco3 support)",
        CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION);
    }
#endif

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    dist_coeffs_   = cv::Mat();

    updateWhitelistFromParameter();
    updateOverridesFromParameters();

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ArucoDetectNode::onParams, this, std::placeholders::_1));

    // camera_info is published RELIABLE + transient_local by the camera driver
    // (often latched / published only once). A volatile best-effort subscriber
    // can miss that single sample and never obtain intrinsics → PnP never runs.
    // Match reliable + transient_local so a late-joining node still gets K and D.
    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, camera_info_qos,
      std::bind(&ArucoDetectNode::cameraInfoCb, this, std::placeholders::_1));

    updateMapMarkersSubscription();
    updatePoseCovSubscription();

    debug_srv_ = create_service<SetBool>(
      debug_service_name_,
      std::bind(&ArucoDetectNode::setDebugSrvCb, this,
                std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    markers_pub_ = create_publisher<MarkerArrayMsg>("/aruco/det/markers", qos);

    if (publish_visualization_) {
      vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/aruco/vis/markers", qos);
    }

    // Timing publisher: общий /aruco/timing для detect и loc (best-effort,
    // shallow queue). Ноды различаются полем "node" в JSON —
    // aruco_profiler.py разбирает поток по нему.
    timing_pub_ = create_publisher<std_msgs::msg::String>(
      "/aruco/timing", rclcpp::QoS(rclcpp::KeepLast(20)).best_effort());

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        if (init_timer_) { init_timer_->cancel(); init_timer_.reset(); }
        initialize();
      });
  }

  void initialize()
  {
    if (initialized_) return;
    initialized_ = true;
    auto self = this->shared_from_this();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(self);
    image_transport::ImageTransport it(self);
    // BEST_EFFORT + KEEP_LAST(1): camera publishes sensor data at full rate; the
    // ArUco callback is slower than the frame rate, so with the default sensor-data
    // depth (5) stale frames queue up and every pose (hence the EV odometry sent to
    // PX4) is built from an old frame — the EV sample then lands outside the EKF's
    // fusion buffer and is dropped. Depth 1 keeps only the newest frame and drops
    // the backlog, so ArUco always works on the freshest image and EV latency stays
    // at ~one frame instead of growing unbounded.
    rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
    image_qos.depth = 1;
    image_sub_ = it.subscribe(
      image_topic_, image_qos,
      std::bind(&ArucoDetectNode::imageCb, this, std::placeholders::_1),
      std::shared_ptr<void>(), nullptr, rclcpp::SubscriptionOptions());
    // Always create publishers; whether we actually publish is controlled by the bool flags.
    // Creating them inside onParams() crashes because ImageTransport::advertise() declares
    // QoS parameters, which is forbidden from within a parameter-set callback.
    debug_image_pub_       = it.advertise(debug_image_topic_, 1);
    world_debug_image_pub_ = it.advertise(world_debug_image_topic_, 1);
  }

private:
  void updateOverridesFromParameters()
  {
    auto ids   = get_parameter("size_override.ids").as_integer_array();
    auto sizes = get_parameter("size_override.sizes").as_double_array();
    setOverridesFromArrays(ids, sizes);
  }

  void setOverridesFromArrays(const std::vector<int64_t>& ids,
                              const std::vector<double>& sizes)
  {
    size_override_param_.clear();
    if (ids.size() != sizes.size()) return;
    for (size_t i = 0; i < ids.size(); ++i) {
      const double s = sizes[i];
      if (std::isfinite(s) && s > 0.0) size_override_param_[static_cast<uint32_t>(ids[i])] = s;
    }
  }

  void updateWhitelistFromParameter()
  {
    const auto ids = get_parameter("marker_id_whitelist").as_integer_array();
    setWhitelistFromIds(ids);
  }

  void setWhitelistFromIds(const std::vector<int64_t>& ids)
  {
    marker_id_whitelist_.clear();
    for (const auto id : ids) {
      if (id >= 0) marker_id_whitelist_.insert(static_cast<uint32_t>(id));
    }
  }

  void updateMapMarkersSubscription()
  {
    if (use_map_markers_) {
      if (!map_markers_sub_) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_markers_sub_ = create_subscription<MarkerArrayMsg>(
          map_markers_topic_, qos,
          std::bind(&ArucoDetectNode::mapMarkersCb, this, std::placeholders::_1));
      }
    } else {
      map_markers_sub_.reset();
      map_ids_.clear();
      size_override_map_.clear();
    }
  }

  void updatePoseCovSubscription()
  {
    if (publish_world_debug_image_) {
      if (!pose_cov_sub_) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_cov_topic_, qos,
          std::bind(&ArucoDetectNode::poseCovCb, this, std::placeholders::_1));
      }
    } else {
      pose_cov_sub_.reset();
      std::lock_guard<std::mutex> lk(world_pose_mutex_);
      have_world_pose_ = false;
    }
  }

  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    auto next_size_override_ids = get_parameter("size_override.ids").as_integer_array();
    auto next_size_override_sizes = get_parameter("size_override.sizes").as_double_array();
    bool size_override_changed = false;
    for (const auto& p : params) {
      const auto& name = p.get_name();
      if (name == "enabled") { enabled_ = p.as_bool(); }
      else if (name == "marker_size") {
        const double v = p.as_double();
        if (v <= 0.0) { r.successful = false; r.reason = "marker_size must be > 0"; return r; }
        default_marker_size_ = v;
      }
      else if (name == "send_tf") { send_tf_ = p.as_bool(); }
      else if (name == "frame_id_prefix") { frame_id_prefix_ = p.as_string(); }
      else if (name == "map_markers_topic") {
        map_markers_topic_ = p.as_string();
        map_markers_sub_.reset();
        updateMapMarkersSubscription();
      }
      else if (name == "use_map_markers") { use_map_markers_ = p.as_bool(); updateMapMarkersSubscription(); }
      else if (name == "detect_individual_markers") { detect_individual_markers_ = p.as_bool(); }
      else if (name == "estimate_marker_pose") { estimate_marker_pose_ = p.as_bool(); }
      else if (name == "max_detect_fps") {
        const double v = p.as_double();
        if (!std::isfinite(v)) { r.successful = false; r.reason = "max_detect_fps must be finite"; return r; }
        max_detect_fps_ = v;
      }
      else if (name == "pnp_fps_cap_enabled") {
        pnp_fps_cap_enabled_ = p.as_bool();
        if (pnp_fps_cap_enabled_ && pnp_fps_cap_ <= 0) pnp_fps_cap_ = 1;
      }
      else if (name == "pnp_fps_cap") {
        const int v = p.as_int();
        if (v <= 0) { r.successful = false; r.reason = "pnp_fps_cap must be > 0"; return r; }
        pnp_fps_cap_ = v;
      }
      else if (name == "publish_debug_image") {
        publish_debug_image_ = p.as_bool();
      }
      else if (name == "debug_image_topic") {
        debug_image_topic_ = p.as_string();
      }
      else if (name == "debug_service_name") { debug_service_name_ = p.as_string(); }
      else if (name == "publish_world_debug_image") {
        publish_world_debug_image_ = p.as_bool();
        pose_cov_sub_.reset();
        updatePoseCovSubscription();
        if (!publish_world_debug_image_) {
          std::lock_guard<std::mutex> lk(world_pose_mutex_);
          have_world_pose_ = false;
        }
      }
      else if (name == "world_debug_image_topic") {
        world_debug_image_topic_ = p.as_string();
      }
      else if (name == "pose_cov_topic") { pose_cov_topic_ = p.as_string(); pose_cov_sub_.reset(); updatePoseCovSubscription(); }
      else if (name == "world_axes_length") {
        world_axes_length_ = p.as_double();
        if (!(world_axes_length_ > 0.0) || !std::isfinite(world_axes_length_)) world_axes_length_ = 0.5;
      }
      else if (name == "world_axes_thickness") {
        world_axes_thickness_ = p.as_int();
        if (world_axes_thickness_ <= 0) world_axes_thickness_ = 3;
      }
      else if (name == "pose_cov_is_world_to_camera") { pose_cov_is_world_to_camera_ = p.as_bool(); }
      else if (name == "world_pose_max_age_sec") { world_pose_max_age_sec_ = p.as_double(); }
      else if (name == "size_override.ids") {
        next_size_override_ids = p.as_integer_array();
        size_override_changed = true;
      }
      else if (name == "size_override.sizes") {
        next_size_override_sizes = p.as_double_array();
        size_override_changed = true;
      }
      else if (name == "dictionary_id") {
        dictionary_id_ = p.as_int();
        dictionary_ = cv::aruco::getPredefinedDictionary(
          static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id_));
      }
      else if (name == "use_aruco3_detection") {
        use_aruco3_detection_ = p.as_bool();
#if ARUCO3_SUPPORTED
        detector_params_->useAruco3Detection = use_aruco3_detection_;
        detector_params_->minMarkerLengthRatioOriginalImg =
          use_aruco3_detection_ ? static_cast<float>(aruco3_min_marker_length_ratio_) : 0.0f;
#endif
      }
      else if (name == "aruco3_min_marker_length_ratio") {
        aruco3_min_marker_length_ratio_ = p.as_double();
#if ARUCO3_SUPPORTED
        if (use_aruco3_detection_)
          detector_params_->minMarkerLengthRatioOriginalImg = static_cast<float>(aruco3_min_marker_length_ratio_);
#endif
      }
      else if (name == "marker_id_whitelist") { setWhitelistFromIds(p.as_integer_array()); }
      else if (name == "pnp_non_map_markers") { pnp_non_map_markers_ = p.as_bool(); }
      else if (name == "coarse_to_fine_enabled") { coarse_to_fine_enabled_ = p.as_bool(); }
      else if (name == "coarse_scale") {
        const double v = p.as_double();
        if (v <= 0.0 || v > 1.0) { r.successful = false; r.reason = "coarse_scale must be in (0, 1]"; return r; }
        coarse_scale_ = v;
      }
      else if (name == "corner_refine_pnp_only") { corner_refine_pnp_only_ = p.as_bool(); }
      else if (name == "subpix_win_size") {
        const int v = p.as_int();
        if (v < 1) { r.successful = false; r.reason = "subpix_win_size must be >= 1"; return r; }
        subpix_win_size_ = v;
      }
      else if (name == "enable_timing_log") {
        enable_timing_log_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "aruco_detect timing_log %s",
                    enable_timing_log_ ? "ENABLED" : "disabled");
      }
      else if (name == "publish_visualization") {
        publish_visualization_ = p.as_bool();
        if (publish_visualization_) {
          if (!vis_pub_) {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
            vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/aruco/vis/markers", qos);
          }
        } else {
          if (vis_pub_) {
            visualization_msgs::msg::MarkerArray arr;
            visualization_msgs::msg::Marker clear;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(clear);
            vis_pub_->publish(arr);
          }
          vis_pub_.reset();
        }
      }
      else if (name == "parent_frame_id") {
        parent_frame_id_ = p.as_string();
        if (parent_frame_id_.empty()) { r.successful = false; r.reason = "parent_frame_id must be non-empty"; return r; }
      }
    }
    if (size_override_changed) {
      setOverridesFromArrays(next_size_override_ids, next_size_override_sizes);
    }
    return r;
  }

  void setDebugSrvCb(const std::shared_ptr<SetBool::Request> req,
                     std::shared_ptr<SetBool::Response> res)
  {
    const bool enable = req->data;
    std::vector<rclcpp::Parameter> ps;
    ps.emplace_back("publish_debug_image", enable);
    ps.emplace_back("publish_world_debug_image", enable);
    ps.emplace_back("publish_visualization", enable);
    const auto results = this->set_parameters(ps);
    bool ok = true; std::string reason;
    for (const auto& rr : results) {
      if (!rr.successful) { ok = false; if (!reason.empty()) reason += "; "; reason += rr.reason; }
    }
    res->success = ok;
    res->message = ok ? (enable ? "debug enabled" : "debug disabled") : reason;
  }

  // Frame-rate throttle for the whole detect->odometry pipeline. Runs BEFORE the
  // expensive convert + detectMarkers so dropped frames cost ~nothing. Uses a phase
  // accumulator (next_due advances by exactly 1/fps) so the AVERAGE output rate
  // equals the target even when it is not an integer divisor of the source rate
  // (e.g. 30 Hz capped to 20 Hz yields ~20 Hz, not the 15 Hz a "drop if too soon
  // since last processed" rule would give). Returns true => PROCESS this frame.
  bool admitDetectFrame(const sensor_msgs::msg::Image& msg, TimePoint now)
  {
    if (!(max_detect_fps_ > 0.0) || !std::isfinite(max_detect_fps_)) return true;

    // Prefer the capture stamp (monotonic, timesync-aligned); fall back to the
    // steady clock when stamps are unset. next_due lives in the active domain.
    bool uses_stamp = false;
    const rclcpp::Time stamp = msg.header.stamp;
    int64_t t_ns;
    if (stamp.nanoseconds() > 0) {
      uses_stamp = true;
      t_ns = stamp.nanoseconds();
    } else {
      t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
               now.time_since_epoch()).count();
    }

    int64_t min_period_ns = static_cast<int64_t>(std::llround(1.0e9 / max_detect_fps_));
    if (min_period_ns < 1) min_period_ns = 1;

    // (Re)prime on first frame, clock-domain switch (stamp<->steady), or a backward
    // time jump (bag loop / sim reset): process this frame and reset the accumulator.
    if (!throttle_primed_ || uses_stamp != next_due_uses_stamp_ || t_ns < last_seen_frame_ns_) {
      throttle_primed_     = true;
      next_due_uses_stamp_ = uses_stamp;
      last_seen_frame_ns_  = t_ns;
      next_due_ns_         = t_ns + min_period_ns;
      return true;
    }

    last_seen_frame_ns_ = t_ns;

    if (t_ns < next_due_ns_) return false;            // arrived too soon -> drop

    next_due_ns_ += min_period_ns;                    // keep average rate (phase accumulator)
    if (next_due_ns_ <= t_ns) next_due_ns_ = t_ns + min_period_ns;  // gap / slow source -> resync, no burst
    return true;
  }

  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
  {
    last_camera_info_ = msg;
    if (!camera_ready_) { parseCameraInfo(*msg); camera_ready_ = true; }
  }

  void mapMarkersCb(const MarkerArrayMsg::ConstSharedPtr& msg)
  {
    map_ids_.clear(); size_override_map_.clear();
    for (const auto& m : msg->markers) {
      map_ids_.insert(m.id);
      if (m.size > 0.0f) size_override_map_[m.id] = static_cast<double>(m.size);
    }
  }

  void poseCovCb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    tf2::Transform T; tf2::fromMsg(msg->pose.pose, T);
    const rclcpp::Time pose_stamp = msg->header.stamp;

    // Pose-driven world_debug rendering: the pose carries the SAME stamp as the
    // image it was computed from (aruco_loc copies det_msg->header.stamp, which
    // is the capture stamp). Find that exact frame in the buffer and draw the
    // axes on IT — so the overlay is always spatially aligned to the markers,
    // instead of drawing the previous cycle's pose on the current frame (the
    // ~1-cycle "axes lag behind the markers" the operator observed).
    cv::Mat matched;   // our own frame copy, evicted from the buffer below
    {
      std::lock_guard<std::mutex> lk(world_pose_mutex_);
      last_pose_stamp_ = pose_stamp;
      last_pose_tf_ = T;
      have_world_pose_ = true;

      if (wantWorldDbg() && !world_frame_buf_.empty()) {
        int best = -1; int64_t best_dns = INT64_MAX;
        for (size_t i = 0; i < world_frame_buf_.size(); ++i) {
          const int64_t dns = std::llabs((world_frame_buf_[i].first - pose_stamp).nanoseconds());
          if (dns < best_dns) { best_dns = dns; best = static_cast<int>(i); }
        }
        constexpr int64_t kMatchTolNs = 5'000'000;  // 5 ms — exact match expected; small slack only
        if (best >= 0 && best_dns <= kMatchTolNs) {
          matched = world_frame_buf_[best].second;
          // Drop the matched frame and everything older: their poses (if any)
          // already arrived or were gated out — keeps the buffer small.
          world_frame_buf_.erase(world_frame_buf_.begin(), world_frame_buf_.begin() + best + 1);
        }
      }
    }

    if (!matched.empty()) {
      drawWorldAxesOverlay(matched, T);   // draws on our evicted copy, never the live buffer
      auto world_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", matched).toImageMsg();
      world_msg->header.stamp = pose_stamp;
      world_msg->header.frame_id = parent_frame_id_;
      world_debug_image_pub_.publish(world_msg);
    }
  }

  // True iff the world_debug topic is enabled AND someone is subscribed.
  bool wantWorldDbg() const {
    return publish_world_debug_image_ && world_debug_image_pub_ &&
           world_debug_image_pub_.getNumSubscribers() > 0;
  }

  // Draw world-frame axes (or a red status string) onto `img` in place, using
  // camera pose `pose_tf`. Mirrors the model-aware axis draw formerly inlined in
  // imageCb; the age check is gone because the frame and pose share a stamp here.
  void drawWorldAxesOverlay(cv::Mat& img, const tf2::Transform& pose_tf)
  {
    if (!camera_ready_) {
      cv::putText(img, "world axes: camera not ready", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      return;
    }
    const tf2::Transform T_camera_world =
      pose_cov_is_world_to_camera_ ? pose_tf : pose_tf.inverse();
    const tf2::Vector3 t = T_camera_world.getOrigin();
    if (t.z() <= 0.0) {
      cv::putText(img, "world axes: behind camera", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      return;
    }
    cv::Vec3d tvec_cv(t.x(), t.y(), t.z());
    tf2::Matrix3x3 m3(T_camera_world.getRotation());
    cv::Mat R(3, 3, CV_64F);
    R.at<double>(0,0)=m3[0][0]; R.at<double>(0,1)=m3[0][1]; R.at<double>(0,2)=m3[0][2];
    R.at<double>(1,0)=m3[1][0]; R.at<double>(1,1)=m3[1][1]; R.at<double>(1,2)=m3[1][2];
    R.at<double>(2,0)=m3[2][0]; R.at<double>(2,1)=m3[2][1]; R.at<double>(2,2)=m3[2][2];
    cv::Vec3d rvec_cv; cv::Rodrigues(R, rvec_cv);
    drawAxesModelAware(img, rvec_cv, tvec_cv, world_axes_length_, world_axes_thickness_);
    cv::putText(img, "world axes", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    if (!enabled_) return;

    // ── Timing ─────────────────────────────────────────────────────────────
    const TimePoint t_cb   = Clock::now();
    const double img_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (!admitDetectFrame(*msg, t_cb)) return;
    const uint32_t t_seq = ++timing_seq_;
    // ───────────────────────────────────────────────────────────────────────

    MarkerArrayMsg out;
    out.header = msg->header;
    out.header.frame_id = parent_frame_id_;

    const bool input_is_mono =
      (msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "8UC1");

    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat gray;
    try {
      if (input_is_mono) {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        gray = cv_ptr->image;
      } else {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      }
    } catch (...) { return; }

    if (gray.empty()) {
      if (publish_visualization_) publishVis(out);
      markers_pub_->publish(std::make_unique<MarkerArrayMsg>(std::move(out)));
      return;
    }

    // ── Timing ─────────────────────────────────────────────────────────────
    const TimePoint t_convert = Clock::now();   // image loaded & converted to gray
    // ───────────────────────────────────────────────────────────────────────

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    if (coarse_to_fine_enabled_) {
      // ── Coarse detect on downscaled gray ─────────────────────────────────
      const int dw = std::max(1, static_cast<int>(gray.cols * coarse_scale_));
      const int dh = std::max(1, static_cast<int>(gray.rows * coarse_scale_));
      const double scale_inv = 1.0 / coarse_scale_;

      cv::Mat coarse_gray;
      if (dw < gray.cols || dh < gray.rows) {
        cv::resize(gray, coarse_gray, cv::Size(dw, dh), 0, 0, cv::INTER_AREA);
      } else {
        coarse_gray = gray;
      }

      // Force CORNER_REFINE_NONE for coarse step; restore afterward
      const int saved_refine = detector_params_->cornerRefinementMethod;
      detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
      cv::aruco::detectMarkers(coarse_gray, dictionary_, corners, ids, detector_params_, rejected);
      detector_params_->cornerRefinementMethod = saved_refine;

      // Scale corners and rejected back to full-res pixel coordinates
      const auto scale_quads = [scale_inv](std::vector<std::vector<cv::Point2f>>& quads) {
        for (auto& quad : quads)
          for (auto& pt : quad) { pt.x *= static_cast<float>(scale_inv); pt.y *= static_cast<float>(scale_inv); }
      };
      scale_quads(corners);
      scale_quads(rejected);
    } else {
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);
    }

    // ── Timing ─────────────────────────────────────────────────────────────
    const TimePoint t_detect = Clock::now();    // detectMarkers done
    // ───────────────────────────────────────────────────────────────────────

    // ── Coarse-to-fine: cornerSubPix on full-res gray ─────────────────────
    TimePoint t_refine = t_detect;   // default: no-op, refine cost = 0
    if (coarse_to_fine_enabled_ && !corners.empty()) {
      // Determine which markers to refine
      std::vector<size_t> refine_idx;
      refine_idx.reserve(ids.size());
      if (corner_refine_pnp_only_) {
        for (size_t i = 0; i < ids.size(); ++i) {
          if (shouldProcessMarker(static_cast<uint32_t>(ids[i]))) refine_idx.push_back(i);
        }
      } else {
        for (size_t i = 0; i < ids.size(); ++i) refine_idx.push_back(i);
      }

      if (!refine_idx.empty()) {
        // Flatten selected corners, refine, write back — preserves order
        std::vector<cv::Point2f> flat;
        flat.reserve(refine_idx.size() * 4);
        for (size_t idx : refine_idx)
          for (const auto& pt : corners[idx]) flat.push_back(pt);

        const int hw = std::max(1, subpix_win_size_);
        cv::cornerSubPix(gray, flat,
                         cv::Size(hw, hw), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

        for (size_t j = 0; j < refine_idx.size(); ++j) {
          const size_t mi = refine_idx[j];
          for (int k = 0; k < 4; ++k) corners[mi][k] = flat[j * 4 + k];
        }
      }
      t_refine = Clock::now();    // cornerSubPix done
    }
    // ─────────────────────────────────────────────────────────────────────

    const bool want_marker_dbg =
      publish_debug_image_ && debug_image_pub_ && (debug_image_pub_.getNumSubscribers() > 0);

    std::vector<int> out_index(ids.size(), -1);
    std::vector<double> det_sizes(ids.size(), default_marker_size_);

    out.markers.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      const uint32_t id = static_cast<uint32_t>(ids[i]);
      if (!shouldProcessMarker(id)) continue;
      MarkerMsg m;
      m.id   = id;
      m.size = static_cast<float>(markerSize(id));
      det_sizes[i] = static_cast<double>(m.size);
      for (int k = 0; k < 4; ++k) {
        m.corners[k].x = corners[i][k].x;
        m.corners[k].y = corners[i][k].y;
        m.corners[k].z = 0.0f;
      }
      out.markers.push_back(m);
      out_index[i] = static_cast<int>(out.markers.size() - 1);
    }

    std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
    std::vector<uint8_t>   pnp_have(ids.size(), 0), pnp_fresh(ids.size(), 0);

    const bool need_pose_outputs = camera_ready_ && estimate_marker_pose_ && !ids.empty();
    const bool need_axes         = camera_ready_ && want_marker_dbg && !ids.empty();

    bool do_fresh_pnp = (need_pose_outputs || need_axes);
    if (do_fresh_pnp && pnp_fps_cap_enabled_ && pnp_fps_cap_ > 0) {
      const rclcpp::Time img_stamp_ros = msg->header.stamp;
      if (last_pnp_run_stamp_.nanoseconds() != 0) {
        const double dt = (img_stamp_ros - last_pnp_run_stamp_).seconds();
        const double min_period = 1.0 / static_cast<double>(pnp_fps_cap_);
        if (std::isfinite(dt) && dt >= 0.0 && dt < min_period) do_fresh_pnp = false;
      }
    }

    int pnp_calls_fresh = 0;

    if ((need_pose_outputs || need_axes) && !ids.empty()) {
      for (size_t i = 0; i < ids.size(); ++i) {
        if (out_index[i] < 0) continue;
        MarkerMsg& m = out.markers[static_cast<size_t>(out_index[i])];
        const uint32_t id = m.id;

        // Map markers: loc_node runs global solvePnP — skip per-marker PnP here entirely.
        // Non-map markers: only if pnp_non_map_markers_ is enabled.
        const bool is_map_marker = use_map_markers_ && (map_ids_.find(id) != map_ids_.end());
        if (is_map_marker || !pnp_non_map_markers_) continue;

        bool used_cache = false;
        auto cache_it = pnp_cache_.find(id);

        if (do_fresh_pnp) {
          const double sz = det_sizes[i];
          const double h = sz * 0.5;
          std::vector<cv::Point3f> obj = {
            {-static_cast<float>(h),  static_cast<float>(h), 0.0f},
            { static_cast<float>(h),  static_cast<float>(h), 0.0f},
            { static_cast<float>(h), -static_cast<float>(h), 0.0f},
            {-static_cast<float>(h), -static_cast<float>(h), 0.0f},
          };
          // Fisheye: rectify corners to ideal pinhole-K px so solvePnP (which is
          // plumb_bob-only, and here gets an empty dist_coeffs_) yields the true pose.
          std::vector<cv::Point2f> img_undist;
          if (is_fisheye_) {
            cv::fisheye::undistortPoints(corners[i], img_undist, camera_matrix_,
                                         fisheye_D_, cv::noArray(), camera_matrix_);
          }
          const std::vector<cv::Point2f>& img = is_fisheye_ ? img_undist : corners[i];
          cv::Vec3d rvec, tvec;
          bool ok = false;

          std::vector<cv::Mat> rvecs_sol, tvecs_sol;
          std::vector<double> reproj_errs;
          const int n_sol = cv::solvePnPGeneric(
            obj, img, camera_matrix_, dist_coeffs_,
            rvecs_sol, tvecs_sol, false, cv::SOLVEPNP_IPPE_SQUARE,
            cv::noArray(), cv::noArray(), reproj_errs);

          if (n_sol > 0) {
            int best = 0;
            if (n_sol >= 2) {
              if (cache_it != pnp_cache_.end() && cache_it->second.valid) {
                const cv::Vec3d& prev_r = cache_it->second.rvec;
                cv::Mat Ra, Rb0, Rb1;
                cv::Rodrigues(prev_r, Ra);
                cv::Rodrigues(cv::Vec3d(rvecs_sol[0]), Rb0);
                cv::Rodrigues(cv::Vec3d(rvecs_sol[1]), Rb1);
                auto angDist = [](const cv::Mat& A, const cv::Mat& B) {
                  cv::Mat Rd = A.t() * B;
                  const double tr = Rd.at<double>(0,0)+Rd.at<double>(1,1)+Rd.at<double>(2,2);
                  return std::acos(std::clamp((tr-1.0)*0.5,-1.0,1.0));
                };
                best = (angDist(Ra, Rb0) <= angDist(Ra, Rb1)) ? 0 : 1;
              } else {
                best = (reproj_errs[0] <= reproj_errs[1]) ? 0 : 1;
              }
            }
            rvec = cv::Vec3d(rvecs_sol[best]);
            tvec = cv::Vec3d(tvecs_sol[best]);
            ok = true;
          }

          if (ok) {
            rvecs[i] = rvec; tvecs[i] = tvec;
            pnp_have[i] = 1; pnp_fresh[i] = 1;
            ++pnp_calls_fresh;
            CachedPnP cp;
            cp.rvec = rvec; cp.tvec = tvec;
            cp.stamp = msg->header.stamp;
            fillPose(rvec, tvec, cp.pose);
            cp.valid = true;
            pnp_cache_[id] = cp;
            cache_it = pnp_cache_.find(id);
          }
        }

        if (!pnp_have[i]) {
          if (cache_it != pnp_cache_.end() && cache_it->second.valid) {
            rvecs[i] = cache_it->second.rvec;
            tvecs[i] = cache_it->second.tvec;
            pnp_have[i] = 1;
            used_cache = true;
          }
        }

        if (need_pose_outputs && pnp_have[i]) {
          if (used_cache) {
            m.pose = cache_it->second.pose;
          } else {
            if (cache_it != pnp_cache_.end() && cache_it->second.valid)
              m.pose = cache_it->second.pose;
            else
              fillPose(rvecs[i], tvecs[i], m.pose);
          }
          if (send_tf_ && tf_broadcaster_ && map_ids_.find(m.id) == map_ids_.end()) {
            geometry_msgs::msg::TransformStamped tfm;
            tfm.header = msg->header;
            tfm.child_frame_id = frame_id_prefix_ + std::to_string(m.id);
            tfm.transform.translation.x = tvecs[i][0];
            tfm.transform.translation.y = tvecs[i][1];
            tfm.transform.translation.z = tvecs[i][2];
            tfm.transform.rotation = m.pose.orientation;
            tf_broadcaster_->sendTransform(tfm);
          }
        }
      }
      if (do_fresh_pnp) last_pnp_run_stamp_ = msg->header.stamp;
    }

    // ── Timing ─────────────────────────────────────────────────────────────
    const TimePoint t_pnp = Clock::now();    // per-marker PnP done
    // ───────────────────────────────────────────────────────────────────────

    // Debug: marker overlay
    cv::Mat marker_debug_image;
    if (want_marker_dbg) {
      if (input_is_mono) cv::cvtColor(gray, marker_debug_image, cv::COLOR_GRAY2BGR);
      else               cv_ptr->image.copyTo(marker_debug_image);

      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(marker_debug_image, corners, ids);
        if (camera_ready_) {
          for (size_t i = 0; i < ids.size(); ++i) {
            if (out_index[i] < 0 || !pnp_have[i]) continue;
            const double sz = det_sizes[i];
            drawAxesModelAware(marker_debug_image, rvecs[i], tvecs[i], sz * 0.5, 3);
            cv::Point2f center(0, 0);
            for (const auto& c : corners[i]) center += c;
            center *= 0.25f;
            const std::string id_text = "ID: " + std::to_string(static_cast<uint32_t>(ids[i]));
            cv::putText(marker_debug_image, id_text,
                        cv::Point(static_cast<int>(center.x + 20), static_cast<int>(center.y - 10)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
          }
        }
      }
      if (!rejected.empty())
        cv::aruco::drawDetectedMarkers(marker_debug_image, rejected, cv::noArray(), cv::Scalar(0, 0, 255));
    }

    // World-axes overlay is POSE-DRIVEN (see poseCovCb): the axes are drawn on
    // the exact frame the pose was computed from, once that pose arrives — so
    // they always stay locked to the markers instead of lagging one cycle.
    // Here we only (a) buffer this base frame keyed by its capture stamp, and
    // (b) if there is NO fresh lock, publish a live status frame so the stream
    // keeps moving while markers are out of view (otherwise it would freeze on
    // the last locked frame). While locked, poseCovCb owns the publishing and
    // this path stays silent — no double-publish.
    if (wantWorldDbg()) {
      cv::Mat base;
      if (input_is_mono) cv::cvtColor(gray, base, cv::COLOR_GRAY2BGR);
      else               cv_ptr->image.copyTo(base);   // deep copy: outlives msg buffer

      const rclcpp::Time frame_stamp = msg->header.stamp;
      bool have_pose = false, have_fresh_lock = false;
      {
        std::lock_guard<std::mutex> lk(world_pose_mutex_);
        world_frame_buf_.emplace_back(frame_stamp, base);
        while (world_frame_buf_.size() > kWorldFrameBufMax) world_frame_buf_.pop_front();
        have_pose = have_world_pose_;
        if (have_world_pose_) {
          if (world_pose_max_age_sec_ > 0.0) {
            const double age = std::fabs((frame_stamp - last_pose_stamp_).seconds());
            have_fresh_lock = std::isfinite(age) && age <= world_pose_max_age_sec_;
          } else {
            have_fresh_lock = true;
          }
        }
      }
      if (!have_fresh_lock) {
        cv::Mat live = base.clone();   // annotate a copy; leave the buffered frame clean
        cv::putText(live, have_pose ? "world axes: searching (pose too old)"
                                    : "world axes: no pose yet",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        auto live_msg = cv_bridge::CvImage(msg->header, "bgr8", live).toImageMsg();
        world_debug_image_pub_.publish(live_msg);
      }
    } else {
      std::lock_guard<std::mutex> lk(world_pose_mutex_);
      world_frame_buf_.clear();   // no subscribers → release buffered frames
    }

    // ── Timing: capture counts before move ─────────────────────────────────
    const size_t _n_detected  = ids.size();
    const size_t _n_processed = out.markers.size();
    // ───────────────────────────────────────────────────────────────────────

    if (publish_visualization_) publishVis(out);
    markers_pub_->publish(std::make_unique<MarkerArrayMsg>(std::move(out)));

    // ── Timing ─────────────────────────────────────────────────────────────
    const TimePoint t_pub = Clock::now();   // markers published
    // ───────────────────────────────────────────────────────────────────────

    if (want_marker_dbg && !marker_debug_image.empty()) {
      auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", marker_debug_image).toImageMsg();
      debug_image_pub_.publish(debug_msg);
    }
    // world_debug_image is published from poseCovCb (pose-driven), not here.

    // ── Timing: publish JSON ────────────────────────────────────────────────
    if (enable_timing_log_) {
      // wall_now: seconds since node start (monotonic, for pairing with loc)
      const double wall_now = ms(node_start_, t_pub) * 1e-3;
      char buf[640];
      std::snprintf(buf, sizeof(buf),
        "{\"node\":\"detect\",\"seq\":%u"
        ",\"image_stamp\":%.6f"
        ",\"wall_now\":%.6f"
        ",\"convert_ms\":%.3f"
        ",\"detect_ms\":%.3f"
        ",\"refine_ms\":%.3f"
        ",\"pnp_ms\":%.3f"
        ",\"total_ms\":%.3f"
        ",\"n_detected\":%zu"
        ",\"n_processed\":%zu"
        ",\"pnp_calls_fresh\":%d"
        ",\"pnp_capped\":%s"
        ",\"input_mono\":%s"
        ",\"coarse_to_fine\":%s"
        "}",
        t_seq,
        img_stamp,
        wall_now,
        ms(t_cb, t_convert),
        ms(t_convert, t_detect),
        ms(t_detect, t_refine),
        ms(t_refine, t_pnp),
        ms(t_cb, t_pub),
        _n_detected, _n_processed,
        pnp_calls_fresh,
        (do_fresh_pnp ? "false" : "true"),
        (input_is_mono ? "true" : "false"),
        (coarse_to_fine_enabled_ ? "true" : "false"));
      std_msgs::msg::String tmsg;
      tmsg.data = buf;
      timing_pub_->publish(tmsg);
    }
    // ───────────────────────────────────────────────────────────────────────
  }

  bool shouldProcessMarker(uint32_t id) const
  {
    if (!marker_id_whitelist_.empty()) return marker_id_whitelist_.count(id) > 0;
    if (use_map_markers_ && !detect_individual_markers_) return map_ids_.find(id) != map_ids_.end();
    return true;
  }

  double markerSize(uint32_t id) const
  {
    auto itp = size_override_param_.find(id);
    if (itp != size_override_param_.end()) return itp->second;
    if (use_map_markers_) {
      auto itm = size_override_map_.find(id);
      if (itm != size_override_map_.end()) return itm->second;
    }
    return default_marker_size_;
  }

  void parseCameraInfo(const sensor_msgs::msg::CameraInfo& msg)
  {
    for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i/3, i%3) = msg.k[i];
    cv::Mat d = cv::Mat::zeros(static_cast<int>(msg.d.size()), 1, CV_64F);
    for (size_t i = 0; i < msg.d.size(); ++i) d.at<double>(static_cast<int>(i), 0) = msg.d[i];
    // Fisheye/equidistant support. cv::solvePnP*/projectPoints/drawFrameAxes only
    // understand the plumb_bob model — feeding them the 4 equidistant coefficients
    // silently misreads them as [k1,k2,p1,p2] and corrupts pose + axis overlays.
    // Stash the fisheye coeffs separately and keep dist_coeffs_ empty; per-marker PnP
    // undistorts its corners via cv::fisheye first, and axis drawing uses the fisheye
    // projection (see drawAxesModelAware). No effect on the plumb_bob path.
    is_fisheye_ = (msg.distortion_model == "equidistant" && msg.d.size() == 4);
    if (is_fisheye_) { fisheye_D_ = d; dist_coeffs_ = cv::Mat(); }
    else             { dist_coeffs_ = d; fisheye_D_ = cv::Mat(); }
  }

  // Draw XYZ axes at a marker pose, honouring the camera model so the overlay lands
  // on the real marker even under strong fisheye distortion.
  void drawAxesModelAware(cv::Mat& img, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                          double length, int thickness) const
  {
    if (!is_fisheye_) {
      cv::drawFrameAxes(img, camera_matrix_, dist_coeffs_, rvec, tvec, length, thickness);
      return;
    }
    const float L = static_cast<float>(length);
    std::vector<cv::Point3f> axis = {{0,0,0}, {L,0,0}, {0,L,0}, {0,0,L}};
    std::vector<cv::Point2f> p;
    cv::fisheye::projectPoints(axis, p, rvec, tvec, camera_matrix_, fisheye_D_);
    cv::line(img, p[0], p[1], cv::Scalar(0,0,255), thickness);  // X red
    cv::line(img, p[0], p[2], cv::Scalar(0,255,0), thickness);  // Y green
    cv::line(img, p[0], p[3], cv::Scalar(255,0,0), thickness);  // Z blue
  }

  static void fillPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, geometry_msgs::msg::Pose& pose)
  {
    pose.position.x = tvec[0]; pose.position.y = tvec[1]; pose.position.z = tvec[2];
    cv::Mat R; cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 m(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                     R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                     R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2));
    tf2::Quaternion q; m.getRotation(q);
    pose.orientation = tf2::toMsg(q);
  }

  void publishVis(const MarkerArrayMsg& dets)
  {
    if (!publish_visualization_) return;
    if (!vis_pub_ || vis_pub_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    if (dets.markers.empty()) {
      vis_pub_->publish(std::make_unique<visualization_msgs::msg::MarkerArray>(std::move(arr)));
      return;
    }
    int idgen = 0;
    for (const auto& m : dets.markers) {
      const double sz = (m.size > 0.0f) ? static_cast<double>(m.size) : default_marker_size_;
      if (m.pose.orientation.w == 0.0 && m.pose.orientation.x == 0.0 &&
          m.pose.orientation.y == 0.0 && m.pose.orientation.z == 0.0) continue;
      visualization_msgs::msg::Marker cube;
      cube.header = dets.header; cube.ns = "aruco"; cube.id = idgen++;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose = m.pose; cube.scale.x = sz; cube.scale.y = sz; cube.scale.z = 0.001;
      cube.color.r = 1.0f; cube.color.g = 1.0f; cube.color.b = 1.0f; cube.color.a = 0.9f;
      arr.markers.push_back(cube);
      visualization_msgs::msg::Marker text;
      text.header = dets.header; text.ns = "aruco_id"; text.id = idgen++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = m.pose; text.pose.position.z += sz * 0.25;
      text.scale.z = sz * 0.6; text.color.g = 1.0f; text.color.a = 1.0f;
      text.text = std::to_string(m.id);
      arr.markers.push_back(text);
    }
    vis_pub_->publish(std::make_unique<visualization_msgs::msg::MarkerArray>(std::move(arr)));
  }

private:
  bool initialized_{false};
  bool enabled_{true};
  double default_marker_size_{0.16};
  bool send_tf_{true};
  std::string frame_id_prefix_{"aruco_"};
  bool use_map_markers_{true};
  bool detect_individual_markers_{true};
  bool estimate_marker_pose_{true};
  double  max_detect_fps_{0.0};
  bool    throttle_primed_{false};
  bool    next_due_uses_stamp_{false};
  int64_t next_due_ns_{0};
  int64_t last_seen_frame_ns_{0};
  bool pnp_fps_cap_enabled_{false};
  int pnp_fps_cap_{10};
  rclcpp::Time last_pnp_run_stamp_{0, 0, RCL_ROS_TIME};

  struct CachedPnP {
    cv::Vec3d rvec{0,0,0}, tvec{0,0,0};
    geometry_msgs::msg::Pose pose{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };
  std::unordered_map<uint32_t, CachedPnP> pnp_cache_;

  bool publish_debug_image_{true};
  std::string debug_image_topic_{"/aruco/vis/debug_image"};
  std::string debug_service_name_{"toggle_debug"};
  rclcpp::Service<SetBool>::SharedPtr debug_srv_;
  std::string parent_frame_id_{"main_camera_optical"};
  bool publish_world_debug_image_{false};
  std::string world_debug_image_topic_{"/aruco/vis/world_debug_image"};
  std::string pose_cov_topic_{"/aruco/loc/pose_cov"};
  double world_axes_length_{0.5};
  int world_axes_thickness_{3};
  bool pose_cov_is_world_to_camera_{false};
  double world_pose_max_age_sec_{0.2};
  bool publish_visualization_{true};
  std::string image_topic_, camera_info_topic_, map_markers_topic_;
  int dictionary_id_{2};
  bool use_aruco3_detection_{false};
  double aruco3_min_marker_length_ratio_{0.10};
  bool pnp_non_map_markers_{false};
  bool coarse_to_fine_enabled_{false};
  double coarse_scale_{0.25};
  bool corner_refine_pnp_only_{false};
  int subpix_win_size_{6};
  std::unordered_set<uint32_t> marker_id_whitelist_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  cv::Mat camera_matrix_, dist_coeffs_;   // dist_coeffs_ empty when is_fisheye_
  cv::Mat fisheye_D_;                      // 4 equidistant coeffs when is_fisheye_
  bool is_fisheye_{false};                 // camera calibrated with fisheye/equidistant
  bool camera_ready_{false};
  std::unordered_map<uint32_t, double> size_override_param_, size_override_map_;
  std::unordered_set<uint32_t> map_ids_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<MarkerArrayMsg>::SharedPtr map_markers_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  std::mutex world_pose_mutex_;
  bool have_world_pose_{false};
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Transform last_pose_tf_{};
  // Recent base frames (capture stamp -> BGR image) for pose-driven world_debug
  // rendering. Written in imageCb, consumed in poseCovCb; both run on the single
  // container spin thread, but the mutex keeps it correct under a future _mt
  // container too. Only populated while world_debug has subscribers.
  std::deque<std::pair<rclcpp::Time, cv::Mat>> world_frame_buf_;
  static constexpr size_t kWorldFrameBufMax = 6;

  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  image_transport::Publisher debug_image_pub_, world_debug_image_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Timing / profiling
  bool enable_timing_log_{false};
  uint32_t timing_seq_{0};
  const TimePoint node_start_{Clock::now()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
};

} // namespace aruco_det_loc

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_det_loc::ArucoDetectNode)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_det_loc::ArucoDetectNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
