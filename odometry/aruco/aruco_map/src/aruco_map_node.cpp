#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
#include <unordered_map>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "aruco_det_loc/msg/marker.hpp"
#include "aruco_det_loc/msg/marker_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

using aruco_det_loc::msg::Marker;
using aruco_det_loc::msg::MarkerArray;

class ArucoMapNode : public rclcpp::Node
{
public:
  ArucoMapNode()
  : rclcpp::Node("aruco_map")
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("marker_size", 0.16);  // meters
    this->declare_parameter<std::string>("topic_name", "/aruco/map/markers");

    // Debug images per plane
    this->declare_parameter<bool>("publish_debug_images", true);
    this->declare_parameter<double>("plane_angle_threshold_deg", 10.0);
    this->declare_parameter<double>("plane_distance_threshold_m", 0.05);
    this->declare_parameter<double>("debug_pixels_per_meter", 500.0);
    this->declare_parameter<int>("debug_image_margin_px", 40);
    this->declare_parameter<int>("debug_image_max_size_px", 4096);
    this->declare_parameter<std::string>(
      "debug_image_topic_prefix",
      "/aruco/vis/map_debug_image/plane_"
    );

    // Debug frame rendering parameters
    this->declare_parameter<std::string>("debug_aruco_dictionary", "DICT_4X4_1000");
    this->declare_parameter<int>("debug_aruco_border_bits", 1);
    this->declare_parameter<bool>("debug_draw_marker_ids", false);

    // World origin axes on debug image
    this->declare_parameter<bool>("debug_draw_origin", true);
    this->declare_parameter<double>("debug_origin_axis_length", 0.5); // meters
    this->declare_parameter<int>("debug_origin_axis_width_px", 3);

    this->declare_parameter<std::string>(
      "map_file",
      ament_index_cpp::get_package_share_directory("aruco_map") +
      "/config/markers.txt"
    );
    this->declare_parameter<double>("debug_image_publish_rate_hz", 1.0);

    frame_id_    = this->get_parameter("frame_id").as_string();
    marker_size_ = this->get_parameter("marker_size").as_double();
    topic_name_  = this->get_parameter("topic_name").as_string();
    map_file_    = this->get_parameter("map_file").as_string();

    publish_debug_images_ = this->get_parameter("publish_debug_images").as_bool();
    plane_angle_threshold_deg_ = this->get_parameter("plane_angle_threshold_deg").as_double();
    plane_distance_threshold_m_ = this->get_parameter("plane_distance_threshold_m").as_double();
    debug_pixels_per_meter_ = this->get_parameter("debug_pixels_per_meter").as_double();
    debug_image_margin_px_ = this->get_parameter("debug_image_margin_px").as_int();
    debug_image_max_size_px_ = this->get_parameter("debug_image_max_size_px").as_int();
    debug_image_topic_prefix_ = this->get_parameter("debug_image_topic_prefix").as_string();
    debug_image_publish_rate_hz_ = this->get_parameter("debug_image_publish_rate_hz").as_double();

    debug_aruco_dictionary_name_ = this->get_parameter("debug_aruco_dictionary").as_string();
    debug_aruco_border_bits_ = this->get_parameter("debug_aruco_border_bits").as_int();
    debug_draw_marker_ids_ = this->get_parameter("debug_draw_marker_ids").as_bool();

    debug_draw_origin_ = this->get_parameter("debug_draw_origin").as_bool();
    debug_origin_axis_length_ = this->get_parameter("debug_origin_axis_length").as_double();
    debug_origin_axis_width_px_ = this->get_parameter("debug_origin_axis_width_px").as_int();

    if (marker_size_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "marker_size <= 0; overriding to 0.16 m");
      marker_size_ = 0.16;
    }

    if (debug_aruco_border_bits_ < 1) {
      RCLCPP_WARN(this->get_logger(), "debug_aruco_border_bits < 1; overriding to 1");
      debug_aruco_border_bits_ = 1;
    }

    initArucoDictionary();

    // transient_local: late subscribers still receive the map
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    publisher_ = this->create_publisher<MarkerArray>(topic_name_, qos);

    buildMapFromFile(map_file_);

    if (publish_debug_images_) {
      publishDebugImages();
    } else {
      RCLCPP_INFO(this->get_logger(), "Debug plane images are disabled (publish_debug_images:=false)");
    }

    // Publish immediately and on timer
    publishMap();
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ArucoMapNode::publishMap, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Aruco map node started. Loaded %zu markers from '%s'. Publishing on '%s' (frame_id='%s').",
      map_.markers.size(), map_file_.c_str(), topic_name_.c_str(), frame_id_.c_str());
  }

private:
  void initArucoDictionary()
  {
    // Dictionary name -> OpenCV ArUco id
    static const std::unordered_map<std::string, int> kDictMap = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
    };

    int dict_id = cv::aruco::DICT_4X4_50;
    auto it = kDictMap.find(debug_aruco_dictionary_name_);
    if (it != kDictMap.end()) {
      dict_id = it->second;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown debug_aruco_dictionary='%s'. Falling back to DICT_4X4_50.",
        debug_aruco_dictionary_name_.c_str());
      debug_aruco_dictionary_name_ = "DICT_4X4_50";
    }

    debug_aruco_dict_ = cv::aruco::getPredefinedDictionary(dict_id);
  }

  void buildMapFromFile(const std::string & filename)
  {
    map_.markers.clear();
    map_.header.frame_id = frame_id_;
    map_.header.stamp = this->now();

    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open marker file: '%s'", filename.c_str());
      return;
    }

    std::string line;
    size_t line_number = 0;

    while (std::getline(file, line)) {
      ++line_number;

      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::istringstream iss(line);
      int id_int;
      double size;
      double x, y, z;
      double roll, pitch, yaw;

      // Exactly 8 fields: id size x y z yaw pitch roll
      if (!(iss >> id_int >> size >> x >> y >> z >> yaw >> pitch >> roll)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not parse line %zu in '%s': '%s'",
          line_number, filename.c_str(), line.c_str());
        continue;
      }

      Marker marker = makeMarker(
        static_cast<uint32_t>(id_int),
        size,
        x, y, z,
        roll, pitch, yaw);

      map_.markers.push_back(marker);
    }

    file.close();

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu markers from file '%s'.",
      map_.markers.size(), filename.c_str());
  }

  Marker makeMarker(
    uint32_t id,
    double size,
    double x, double y, double z,
    double roll, double pitch, double yaw)
  {
    Marker m;
    m.id = id;
    m.size = static_cast<float>(size);

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    m.pose = pose;
    return m;
  }

  struct PlaneCluster
  {
    tf2::Vector3 n;                 // plane normal (unit)
    double d{0.0};                  // plane offset n·x = d
    tf2::Vector3 origin;            // point on plane (closest to world origin)
    tf2::Vector3 u;                 // in-plane axis
    tf2::Vector3 v;                 // in-plane axis
    std::vector<size_t> indices;    // marker indices on this plane
  };

  tf2::Vector3 canonicalizeNormal(tf2::Vector3 n) const
  {
    constexpr double kEps = 1e-12;
    if (n.length2() < kEps) {
      return tf2::Vector3(0.0, 0.0, 1.0);
    }
    n.normalize();
    if (n.z() < -1e-9) {
      n *= -1.0;
    } else if (std::abs(n.z()) <= 1e-9) {
      if (n.y() < -1e-9) {
        n *= -1.0;
      } else if (std::abs(n.y()) <= 1e-9 && n.x() < 0.0) {
        n *= -1.0;
      }
    }
    return n;
  }

  tf2::Vector3 markerPosition(const Marker & m) const
  {
    return tf2::Vector3(m.pose.position.x, m.pose.position.y, m.pose.position.z);
  }

  tf2::Quaternion markerQuaternion(const Marker & m) const
  {
    tf2::Quaternion q;
    tf2::fromMsg(m.pose.orientation, q);
    q.normalize();
    return q;
  }

  tf2::Vector3 markerNormal(const Marker & m) const
  {
    auto q = markerQuaternion(m);
    auto n = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
    return canonicalizeNormal(n);
  }

  // In-plane basis: map world +X to +u (right on image), +Y to +v (up) when possible.
  void planeBasisFromNormal(const tf2::Vector3 & n_in, tf2::Vector3 & u_out, tf2::Vector3 & v_out) const
  {
    tf2::Vector3 n = canonicalizeNormal(n_in);

    const tf2::Vector3 wx(1.0, 0.0, 0.0);
    const tf2::Vector3 wy(0.0, 1.0, 0.0);
    const tf2::Vector3 wz(0.0, 0.0, 1.0);

    auto projToPlane = [&](const tf2::Vector3 & a) {
      return a - n * n.dot(a);
    };

    constexpr double kEps = 1e-12;

    tf2::Vector3 projX = projToPlane(wx);
    tf2::Vector3 projY = projToPlane(wy);

    tf2::Vector3 u;
    if (projX.length2() > kEps) {
      u = projX;
    } else if (projY.length2() > kEps) {
      u = projY;
    } else {
      tf2::Vector3 ref = wz;
      if (std::abs(n.dot(ref)) > 0.9) {
        ref = wx;
      }
      u = n.cross(ref);
      if (u.length2() < kEps) {
        ref = wy;
        u = n.cross(ref);
      }
    }

    if (u.length2() < kEps) {
      u = wx;
    }
    u.normalize();

    if (projX.length2() > kEps && u.dot(projX) < 0.0) {
      u *= -1.0;
    }

    tf2::Vector3 v;
    if (projY.length2() > kEps) {
      v = projY - u * u.dot(projY);
      if (v.length2() < kEps) {
        v = n.cross(u);
      }
    } else {
      v = n.cross(u);
    }

    if (v.length2() < kEps) {
      tf2::Vector3 ref = wz;
      if (std::abs(n.dot(ref)) > 0.9) {
        ref = wx;
      }
      v = n.cross(ref);
    }

    if (v.length2() < kEps) {
      v = wy;
    }
    v.normalize();

    if (projY.length2() > kEps && v.dot(projY) < 0.0) {
      v *= -1.0;
    }

    u_out = u;
    v_out = v;
  }

  std::vector<PlaneCluster> detectPlanes(const std::vector<Marker> & markers) const
  {
    std::vector<PlaneCluster> planes;
    if (markers.empty()) {
      return planes;
    }

    constexpr double kPi = 3.14159265358979323846;
    double angle_thr_rad = std::max(0.0, plane_angle_threshold_deg_) * kPi / 180.0;
    double dist_thr = std::max(0.0, plane_distance_threshold_m_);

    std::vector<bool> used(markers.size(), false);
    planes.reserve(markers.size());

    for (size_t i = 0; i < markers.size(); ++i) {
      if (used[i]) {
        continue;
      }
      PlaneCluster pc;
      pc.indices.push_back(i);
      used[i] = true;

      bool added_any = true;
      while (added_any) {
        tf2::Vector3 n_sum(0.0, 0.0, 0.0);
        double d_sum = 0.0;
        for (auto idx : pc.indices) {
          auto n_i = markerNormal(markers[idx]);
          auto p_i = markerPosition(markers[idx]);
          n_sum += n_i;
          d_sum += n_i.dot(p_i);
        }
        pc.n = canonicalizeNormal(n_sum);
        pc.d = d_sum / static_cast<double>(pc.indices.size());

        added_any = false;
        for (size_t j = 0; j < markers.size(); ++j) {
          if (used[j]) {
            continue;
          }
          auto n_j = markerNormal(markers[j]);
          double dot = std::abs(pc.n.dot(n_j));
          dot = std::max(-1.0, std::min(1.0, dot));
          double angle = std::acos(dot);
          if (angle > angle_thr_rad) {
            continue;
          }
          auto p_j = markerPosition(markers[j]);
          double plane_dist = std::abs(pc.n.dot(p_j) - pc.d);
          if (plane_dist > dist_thr) {
            continue;
          }

          pc.indices.push_back(j);
          used[j] = true;
          added_any = true;
        }
      }

      pc.origin = pc.n * pc.d;
      planeBasisFromNormal(pc.n, pc.u, pc.v);
      planes.push_back(pc);
    }

    std::sort(planes.begin(), planes.end(), [](const PlaneCluster & a, const PlaneCluster & b) {
      return a.d < b.d;
    });

    return planes;
  }

  // cv::Mat BGR8 → sensor_msgs/Image
  sensor_msgs::msg::Image matToImageMsg(const cv::Mat & mat) const
  {
    cv::Mat img = mat;
    if (!img.isContinuous()) {
      img = img.clone();
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(img.rows);
    msg.width  = static_cast<uint32_t>(img.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(img.step);

    const size_t nbytes = img.total() * img.elemSize();
    msg.data.resize(nbytes);
    std::memcpy(msg.data.data(), img.data, nbytes);
    return msg;
  }

  // Debug image: real ArUco patterns on the plane; optional world origin axes.
  sensor_msgs::msg::Image renderPlaneDebugImage(const PlaneCluster & plane) const
  {
    const int margin_px = std::max(0, debug_image_margin_px_);
    const int max_size_px = std::max(256, debug_image_max_size_px_);
    double ppm = std::max(1.0, debug_pixels_per_meter_);

    // Whether world origin (0,0,0) lies on this plane
    const bool origin_in_plane =
      (std::abs(plane.d) < std::max(0.001, plane_distance_threshold_m_));

    const bool draw_origin = debug_draw_origin_ && origin_in_plane;

    auto toUV = [&](const tf2::Vector3 & p) {
      tf2::Vector3 dp = p - plane.origin;
      return std::pair<double, double>(plane.u.dot(dp), plane.v.dot(dp));
    };

    // Bounds from marker corners
    double min_u = std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();

    auto expandBoundsUV = [&](double u, double v) {
      min_u = std::min(min_u, u);
      min_v = std::min(min_v, v);
      max_u = std::max(max_u, u);
      max_v = std::max(max_v, v);
    };

    // Expand bounds for markers
    for (auto idx : plane.indices) {
      const auto & m = map_.markers[idx];
      const double size_m = (m.size > 0.0f) ? static_cast<double>(m.size) : marker_size_;
      auto p_c = markerPosition(m);
      auto q = markerQuaternion(m);

      tf2::Vector3 x_axis = tf2::quatRotate(q, tf2::Vector3(1.0, 0.0, 0.0));
      tf2::Vector3 y_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 1.0, 0.0));
      x_axis = x_axis - plane.n * plane.n.dot(x_axis);
      y_axis = y_axis - plane.n * plane.n.dot(y_axis);

      if (x_axis.length2() < 1e-12) x_axis = plane.u;
      x_axis.normalize();

      y_axis = plane.n.cross(x_axis);
      if (y_axis.length2() < 1e-12) y_axis = plane.v;
      y_axis.normalize();

      const double h = 0.5 * size_m;
      const tf2::Vector3 corners[4] = {
        p_c + x_axis * h + y_axis * h,
        p_c - x_axis * h + y_axis * h,
        p_c - x_axis * h - y_axis * h,
        p_c + x_axis * h - y_axis * h,
      };

      for (const auto & c : corners) {
        auto [uu, vv] = toUV(c);
        expandBoundsUV(uu, vv);
      }
    }

    if (!std::isfinite(min_u) || !std::isfinite(min_v) || !std::isfinite(max_u) || !std::isfinite(max_v)) {
      cv::Mat img(200, 400, CV_8UC3, cv::Scalar(255, 255, 255));
      cv::putText(img, "No markers", cv::Point(10, 80),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
      return matToImageMsg(img);
    }

    const double marker_min_u = min_u;
    const double marker_min_v = min_v;
    const double marker_max_u = max_u;
    const double marker_max_v = max_v;

    const tf2::Vector3 world_origin(0.0, 0.0, 0.0);
    double origin_u = 0.0;
    double origin_v = 0.0;

    // Include origin in bounds; do not expand by axis tips
    if (draw_origin) {
      auto uv = toUV(world_origin);
      origin_u = uv.first;
      origin_v = uv.second;
      expandBoundsUV(origin_u, origin_v);
    }

    // Clamp axis length to map extent on image
    double axis_len = 0.0;
    if (draw_origin) {
      axis_len = std::max(0.0, debug_origin_axis_length_);

      auto maxLenInMarkerBounds = [&](double du, double dv) -> double {
        constexpr double kInf = 1e100;

        auto bound1D = [&](double o, double d, double lo, double hi) -> double {
          if (std::abs(d) < 1e-12) {
            if (o < lo || o > hi) {
              return 0.0;
            }
            return kInf;
          }
          double t1 = (lo - o) / d;
          double t2 = (hi - o) / d;
          double tmax = std::max(t1, t2);
          if (tmax < 0.0) {
            return 0.0;
          }
          return tmax;
        };

        double l_u = bound1D(origin_u, du, marker_min_u, marker_max_u);
        double l_v = bound1D(origin_v, dv, marker_min_v, marker_max_v);
        return std::max(0.0, std::min(l_u, l_v));
      };

      const tf2::Vector3 ex(1.0, 0.0, 0.0);
      const tf2::Vector3 ey(0.0, 1.0, 0.0);
      const tf2::Vector3 ez(0.0, 0.0, 1.0);

      const double dux = plane.u.dot(ex);
      const double dvx = plane.v.dot(ex);
      const double duy = plane.u.dot(ey);
      const double dvy = plane.v.dot(ey);
      const double duz = plane.u.dot(ez);
      const double dvz = plane.v.dot(ez);

      double fit = std::numeric_limits<double>::infinity();
      if (std::hypot(dux, dvx) > 1e-9) {
        fit = std::min(fit, maxLenInMarkerBounds(dux, dvx));
      }
      if (std::hypot(duy, dvy) > 1e-9) {
        fit = std::min(fit, maxLenInMarkerBounds(duy, dvy));
      }
      if (std::hypot(duz, dvz) > 1e-9) {
        fit = std::min(fit, maxLenInMarkerBounds(duz, dvz));
      }

      if (std::isfinite(fit)) {
        axis_len = std::max(0.0, std::min(axis_len, fit));
      } else {
        axis_len = 0.0;
      }
    }

    double range_u = std::max(1e-9, max_u - min_u);
    double range_v = std::max(1e-9, max_v - min_v);

    // Fit within max image size
    {
      double ppm_fit_u = (max_size_px - 2.0 * margin_px) / range_u;
      double ppm_fit_v = (max_size_px - 2.0 * margin_px) / range_v;
      double ppm_fit = std::max(1.0, std::min(ppm_fit_u, ppm_fit_v));
      ppm = std::min(ppm, ppm_fit);
    }

    int width = static_cast<int>(std::ceil(range_u * ppm)) + 2 * margin_px;
    int height = static_cast<int>(std::ceil(range_v * ppm)) + 2 * margin_px;
    width = std::max(64, std::min(width, max_size_px));
    height = std::max(64, std::min(height, max_size_px));

    cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    auto uvToPix = [&](double u, double v) {
      int x = margin_px + static_cast<int>(std::lround((u - min_u) * ppm));
      int y = margin_px + static_cast<int>(std::lround((max_v - v) * ppm));
      return cv::Point(x, y);
    };

    // Draw markers (ArUco bitmap) warped to plane
    for (auto idx : plane.indices) {
      const auto & m = map_.markers[idx];
      const double size_m = (m.size > 0.0f) ? static_cast<double>(m.size) : marker_size_;
      auto p_c = markerPosition(m);
      auto q = markerQuaternion(m);

      tf2::Vector3 x_axis = tf2::quatRotate(q, tf2::Vector3(1.0, 0.0, 0.0));
      tf2::Vector3 y_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 1.0, 0.0));
      x_axis = x_axis - plane.n * plane.n.dot(x_axis);
      y_axis = y_axis - plane.n * plane.n.dot(y_axis);

      if (x_axis.length2() < 1e-12) x_axis = plane.u;
      x_axis.normalize();

      y_axis = plane.n.cross(x_axis);
      if (y_axis.length2() < 1e-12) y_axis = plane.v;
      y_axis.normalize();

      const double h = 0.5 * size_m;
      const tf2::Vector3 corners3d[4] = {
        p_c + x_axis * h + y_axis * h,
        p_c - x_axis * h + y_axis * h,
        p_c - x_axis * h - y_axis * h,
        p_c + x_axis * h - y_axis * h,
      };

      // Corners in pixels
      std::vector<cv::Point2f> pts;
      pts.reserve(4);
      for (const auto & c : corners3d) {
        auto [uu, vv] = toUV(c);
        cv::Point p = uvToPix(uu, vv);
        pts.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
      }

      // Corner order {TL, TR, BR, BL}; source 0=TR, 1=TL, 2=BL, 3=BR
      std::vector<cv::Point2f> dst(4);
      dst[0] = pts[1]; // TL
      dst[1] = pts[0]; // TR
      dst[2] = pts[3]; // BR
      dst[3] = pts[2]; // BL

      // Marker raster size in px (speed clamp)
      int marker_px = static_cast<int>(std::lround(size_m * ppm));
      marker_px = std::max(32, std::min(marker_px, 2048));

      // Generate marker bitmap
      cv::Mat marker_gray(marker_px, marker_px, CV_8UC1, cv::Scalar(255));
      bool drew_marker = false;
      try {
        if (debug_aruco_dict_) {
          cv::aruco::drawMarker(debug_aruco_dict_, static_cast<int>(m.id), marker_px, marker_gray, debug_aruco_border_bits_);
          drew_marker = true;
        }
      } catch (const cv::Exception & e) {
        (void)e;
        drew_marker = false;
      }

      // ID out of dictionary: placeholder square + optional id text
      if (!drew_marker) {
        std::vector<cv::Point> poly = {
          cv::Point(static_cast<int>(std::lround(dst[0].x)), static_cast<int>(std::lround(dst[0].y))),
          cv::Point(static_cast<int>(std::lround(dst[1].x)), static_cast<int>(std::lround(dst[1].y))),
          cv::Point(static_cast<int>(std::lround(dst[2].x)), static_cast<int>(std::lround(dst[2].y))),
          cv::Point(static_cast<int>(std::lround(dst[3].x)), static_cast<int>(std::lround(dst[3].y)))
        };
        cv::polylines(img, poly, true, cv::Scalar(0, 0, 0), 2);
        if (debug_draw_marker_ids_) {
          auto [cu, cvv] = toUV(p_c);
          cv::Point center_px = uvToPix(cu, cvv);
          cv::putText(
            img,
            std::to_string(m.id),
            center_px + cv::Point(5, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 0, 0),
            2);
        }
        continue;
      }

      cv::Mat marker_bgr;
      cv::cvtColor(marker_gray, marker_bgr, cv::COLOR_GRAY2BGR);

      std::vector<cv::Point2f> src = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(marker_px - 1), 0.0f),
        cv::Point2f(static_cast<float>(marker_px - 1), static_cast<float>(marker_px - 1)),
        cv::Point2f(0.0f, static_cast<float>(marker_px - 1))
      };

      cv::Mat H = cv::getPerspectiveTransform(src, dst);

      // Warp to full canvas
      cv::Mat warped(img.size(), img.type(), cv::Scalar(255, 255, 255));
      cv::warpPerspective(
        marker_bgr,
        warped,
        H,
        img.size(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(255, 255, 255));

      // Quad mask: marker region only
      cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(0));
      std::vector<cv::Point> poly = {
        cv::Point(static_cast<int>(std::lround(dst[0].x)), static_cast<int>(std::lround(dst[0].y))),
        cv::Point(static_cast<int>(std::lround(dst[1].x)), static_cast<int>(std::lround(dst[1].y))),
        cv::Point(static_cast<int>(std::lround(dst[2].x)), static_cast<int>(std::lround(dst[2].y))),
        cv::Point(static_cast<int>(std::lround(dst[3].x)), static_cast<int>(std::lround(dst[3].y)))
      };
      cv::fillConvexPoly(mask, poly, cv::Scalar(255));

      warped.copyTo(img, mask);

      // Optional id label
      if (debug_draw_marker_ids_) {
        auto [cu, cvv] = toUV(p_c);
        cv::Point center_px = uvToPix(cu, cvv);
        cv::putText(
          img,
          std::to_string(m.id),
          center_px + cv::Point(5, -5),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(0, 0, 255),
          2);
      }
    }

    // World origin axes
    if (draw_origin) {
      auto [ou, ov] = toUV(world_origin);

      cv::Point p_origin = uvToPix(ou, ov);

      const tf2::Vector3 world_axis_x(axis_len, 0.0, 0.0);
      const tf2::Vector3 world_axis_y(0.0, axis_len, 0.0);
      const tf2::Vector3 world_axis_z(0.0, 0.0, axis_len);

      // Arrow if projected length is non-zero
      auto drawAxis = [&](const tf2::Vector3 & tip, const cv::Scalar & color) {
        auto [tu, tv] = toUV(tip);

        cv::Point p_tip = uvToPix(tu, tv);

        if (cv::norm(p_origin - p_tip) > 1.0) {
          cv::arrowedLine(img, p_origin, p_tip, color, debug_origin_axis_width_px_, cv::LINE_AA);
        }
      };

      // Z — blue
      drawAxis(world_axis_z, cv::Scalar(255, 0, 0));
      // Y — green
      drawAxis(world_axis_y, cv::Scalar(0, 255, 0));
      // X — red
      drawAxis(world_axis_x, cv::Scalar(0, 0, 255));

      // Circle at origin
      cv::circle(img, p_origin, std::max(2, debug_origin_axis_width_px_ + 2), cv::Scalar(0, 0, 0), -1);
      cv::circle(img, p_origin, std::max(1, debug_origin_axis_width_px_), cv::Scalar(255, 255, 255), -1);
    }

    return matToImageMsg(img);
  }

  void republishDebugImages()
  {
    if (debug_image_publishers_.empty() || cached_plane_images_.empty()) {
      return;
    }
    for (size_t i = 0; i < debug_image_publishers_.size(); ++i) {
      cached_plane_images_[i].header.stamp = this->now();
      debug_image_publishers_[i]->publish(cached_plane_images_[i]);
    }
  }

  void publishDebugImages()
  {
    cached_planes_ = detectPlanes(map_.markers);
    if (cached_planes_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No planes detected (marker list empty?) - debug images not published");
      return;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    debug_image_publishers_.clear();
    debug_image_publishers_.reserve(cached_planes_.size());

    cached_plane_images_.clear();
    cached_plane_images_.reserve(cached_planes_.size());

    for (size_t i = 0; i < cached_planes_.size(); ++i) {
      std::string topic = debug_image_topic_prefix_ + std::to_string(i + 1);
      auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic, qos);
      debug_image_publishers_.push_back(pub);

      cached_plane_images_.push_back(renderPlaneDebugImage(cached_planes_[i]));
      debug_image_publishers_.back()->publish(cached_plane_images_.back());
    }

    // Periodic republish for late subscribers
    const double hz = std::max(0.0, debug_image_publish_rate_hz_);
    if (hz > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / hz);
      debug_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ArucoMapNode::republishDebugImages, this));
    }
  }

  void publishMap()
  {
    map_.header.stamp = this->now();
    publisher_->publish(map_);
  }

  // Class members
  std::string frame_id_{"world"};
  double marker_size_{0.16};
  std::string topic_name_{"/aruco/map/markers"};
  std::string map_file_{"/config/markers.txt"};
  double debug_image_publish_rate_hz_{1.0};

  // Debug image parameters
  bool publish_debug_images_{true};
  double plane_angle_threshold_deg_{10.0};
  double plane_distance_threshold_m_{0.05};
  double debug_pixels_per_meter_{500.0};
  int debug_image_margin_px_{40};
  int debug_image_max_size_px_{4096};
  std::string debug_image_topic_prefix_{"/aruco/vis/map_debug_image/plane_"};

  // Debug draw toggles
  std::string debug_aruco_dictionary_name_{"DICT_4X4_50"};
  int debug_aruco_border_bits_{1};
  bool debug_draw_marker_ids_{false};
  cv::Ptr<cv::aruco::Dictionary> debug_aruco_dict_;

  // Origin axis parameters
  bool debug_draw_origin_{true};
  double debug_origin_axis_length_{0.5};
  int debug_origin_axis_width_px_{3};

  MarkerArray map_;
  rclcpp::Publisher<MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  std::vector<PlaneCluster> cached_planes_;
  std::vector<sensor_msgs::msg::Image> cached_plane_images_;

  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> debug_image_publishers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
