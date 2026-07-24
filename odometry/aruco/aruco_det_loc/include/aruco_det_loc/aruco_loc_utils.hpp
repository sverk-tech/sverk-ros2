#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "aruco_det_loc/msg/marker.hpp"
#include "aruco_det_loc/msg/marker_array.hpp"

namespace aruco_det_loc::loc_utils
{

using MarkerMsg      = aruco_det_loc::msg::Marker;
using MarkerArrayMsg = aruco_det_loc::msg::MarkerArray;

constexpr int kCornersPerMarker = 4;

// -----------------------------
// Basic sanity checks
// -----------------------------

inline bool isFinite(double v) { return std::isfinite(v); }

inline bool isValidQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const bool all_zero = (q.w == 0.0 && q.x == 0.0 && q.y == 0.0 && q.z == 0.0);
  if (all_zero) return false;
  return isFinite(q.w) && isFinite(q.x) && isFinite(q.y) && isFinite(q.z);
}

inline tf2::Quaternion normalize(tf2::Quaternion q)
{
  const double n = q.length();
  if (n > 0.0) q /= n;
  return q;
}

// Camera intrinsics (K, distortion)

inline void parseCameraInfo(const sensor_msgs::msg::CameraInfo& msg, cv::Mat& K, cv::Mat& dist)
{
  if (K.empty()) K = cv::Mat::zeros(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    K.at<double>(i / 3, i % 3) = msg.k[i];
  }

  dist = cv::Mat::zeros(static_cast<int>(msg.d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg.d.size(); ++i) {
    dist.at<double>(static_cast<int>(i), 0) = msg.d[i];
  }
}

// Pose / tf2::Transform

inline tf2::Transform tfFromPoseMsg(const geometry_msgs::msg::Pose& p)
{
  tf2::Quaternion q;
  tf2::fromMsg(p.orientation, q);
  q = normalize(q);
  tf2::Vector3 t(p.position.x, p.position.y, p.position.z);
  return tf2::Transform(q, t);
}

inline geometry_msgs::msg::Pose poseMsgFromTf(const tf2::Transform& T)
{
  geometry_msgs::msg::Pose p;
  p.position.x = T.getOrigin().x();
  p.position.y = T.getOrigin().y();
  p.position.z = T.getOrigin().z();
  p.orientation = tf2::toMsg(T.getRotation());
  return p;
}

inline geometry_msgs::msg::Transform transformMsgFromTf(const tf2::Transform& T)
{
  geometry_msgs::msg::Transform tr;
  tr.translation.x = T.getOrigin().x();
  tr.translation.y = T.getOrigin().y();
  tr.translation.z = T.getOrigin().z();
  tr.rotation = tf2::toMsg(T.getRotation());
  return tr;
}

inline tf2::Transform tfFromRvecTvec_worldToCamera(const cv::Vec3d& rvec_cw, const cv::Vec3d& tvec_cw)
{
  cv::Mat R;
  cv::Rodrigues(rvec_cw, R);
  tf2::Matrix3x3 m(
    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
  tf2::Quaternion q;
  m.getRotation(q);
  q = normalize(q);

  tf2::Vector3 t(tvec_cw[0], tvec_cw[1], tvec_cw[2]);
  return tf2::Transform(q, t);  // T_camera_world from OpenCV rvec/tvec
}

// Marker corners in marker frame (order matches detect node)

inline std::array<cv::Point3f, kCornersPerMarker> markerLocalCorners(double size_m)
{
  const double h = 0.5 * size_m;
  return {
    cv::Point3f(static_cast<float>(-h), static_cast<float>( h), 0.0f),
    cv::Point3f(static_cast<float>( h), static_cast<float>( h), 0.0f),
    cv::Point3f(static_cast<float>( h), static_cast<float>(-h), 0.0f),
    cv::Point3f(static_cast<float>(-h), static_cast<float>(-h), 0.0f),
  };
}

inline std::array<cv::Point3f, kCornersPerMarker>
transformCorners(const tf2::Transform& T, const std::array<cv::Point3f, kCornersPerMarker>& pts)
{
  std::array<cv::Point3f, kCornersPerMarker> out;
  for (int i = 0; i < kCornersPerMarker; ++i) {
    const tf2::Vector3 p_local(pts[i].x, pts[i].y, pts[i].z);
    const tf2::Vector3 p_world = T * p_local;
    out[i] = cv::Point3f(
      static_cast<float>(p_world.x()),
      static_cast<float>(p_world.y()),
      static_cast<float>(p_world.z()));
  }
  return out;
}

// Map index: marker poses in world and corners in world

struct MapMarker
{
  uint32_t id{0};
  double size_m{0.0};
  tf2::Transform T_world_marker;
  std::array<cv::Point3f, kCornersPerMarker> corners_world{};
};

using MapIndex = std::unordered_map<uint32_t, MapMarker>;

inline void buildMapIndex(const MarkerArrayMsg& map_msg, MapIndex& index_out)
{
  index_out.clear();
  index_out.reserve(map_msg.markers.size());

  for (const auto& m : map_msg.markers) {
    // Require valid orientation and positive size.
    const double size = static_cast<double>(m.size);
    if (!(size > 0.0) || !isFinite(size)) continue;
    if (!isValidQuaternion(m.pose.orientation)) continue;

    MapMarker mm;
    mm.id = m.id;
    mm.size_m = size;
    mm.T_world_marker = tfFromPoseMsg(m.pose);
    mm.corners_world = transformCorners(mm.T_world_marker, markerLocalCorners(size));

    index_out.emplace(mm.id, std::move(mm));
  }
}

// Global PnP correspondences (4 points per known marker)

struct Correspondences
{
  std::vector<cv::Point3f> object_points_world;
  std::vector<cv::Point2f> image_points_px;
  std::vector<uint32_t>    point_marker_ids;
  std::vector<uint32_t>    used_marker_ids;
  std::vector<tf2::Transform> used_marker_world_transforms;
  std::vector<double> used_marker_sizes_m;
};

inline Correspondences buildCorrespondences(const MarkerArrayMsg& det_msg, const MapIndex& map_index)
{
  Correspondences corr;
  corr.object_points_world.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.image_points_px.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.point_marker_ids.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.used_marker_ids.reserve(det_msg.markers.size());
  corr.used_marker_world_transforms.reserve(det_msg.markers.size());
  corr.used_marker_sizes_m.reserve(det_msg.markers.size());

  for (const auto& det : det_msg.markers) {
    auto it = map_index.find(det.id);
    if (it == map_index.end()) continue;

    // 4 world corners + 4 image pixels
    const auto& W = it->second.corners_world;
    for (int k = 0; k < kCornersPerMarker; ++k) {
      const double u = det.corners[k].x;
      const double v = det.corners[k].y;
      if (!isFinite(u) || !isFinite(v)) continue;

      corr.object_points_world.push_back(W[k]);
      corr.image_points_px.emplace_back(static_cast<float>(u), static_cast<float>(v));
      corr.point_marker_ids.push_back(det.id);
    }

    corr.used_marker_ids.push_back(det.id);
    corr.used_marker_world_transforms.push_back(it->second.T_world_marker);
    corr.used_marker_sizes_m.push_back(it->second.size_m);
  }

  return corr;
}

// PnP and reprojection metrics

struct PnPRansacParams
{
  // 16 iterations is sufficient for w≈0.95 inlier rate, s=4, p=0.99
  // (RANSAC formula: N = log(1-p) / log(1-w^s) ≈ 3).
  // 150 was a 50× overestimate — reduced to 16 for performance.
  int iterations_count{16};
  float reprojection_error_px{4.0f};
  double confidence{0.99};
  int flags{cv::SOLVEPNP_ITERATIVE};

  // <=0 means automatic hybrid gating:
  // - 1 marker: IPPE_SQUARE without RANSAC (4 square corners)
  // - >=2 markers: RANSAC with inlier count derived from marker count
  // >0 restores the legacy hard minimum.
  int min_used_markers{0};
  int min_inlier_points{0};
};

enum class PnPMethod
{
  None,
  Ransac,
  SingleMarkerIppeSquare,
};

struct PnPResult
{
  cv::Vec3d rvec_cw{};
  cv::Vec3d tvec_cw{};
  std::vector<int> inlier_indices;
  double reproj_rmse_all{std::numeric_limits<double>::quiet_NaN()};
  double reproj_rmse_inliers{std::numeric_limits<double>::quiet_NaN()};
  PnPMethod method{PnPMethod::None};
  int used_markers{0};
  int required_inlier_points{0};
  double covariance_confidence_scale{1.0};
};

inline const char* pnpMethodName(PnPMethod method)
{
  switch (method) {
    case PnPMethod::Ransac: return "ransac";
    case PnPMethod::SingleMarkerIppeSquare: return "single_ippe_square";
    case PnPMethod::None:
    default: return "none";
  }
}

inline double reprojectionRmse(const std::vector<cv::Point3f>& obj,
                               const std::vector<cv::Point2f>& img,
                               const cv::Mat& K, const cv::Mat& dist,
                               const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                               const std::vector<int>* subset_indices = nullptr)
{
  if (obj.empty() || img.empty() || obj.size() != img.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj, rvec, tvec, K, dist, proj);

  double sum2 = 0.0;
  size_t n = 0;

  auto accum = [&](int i) {
    const double du = static_cast<double>(proj[i].x) - static_cast<double>(img[i].x);
    const double dv = static_cast<double>(proj[i].y) - static_cast<double>(img[i].y);
    sum2 += du * du + dv * dv;
    ++n;
  };

  if (subset_indices && !subset_indices->empty()) {
    for (int idx : *subset_indices) {
      if (idx >= 0 && static_cast<size_t>(idx) < obj.size()) accum(idx);
    }
  } else {
    for (size_t i = 0; i < obj.size(); ++i) accum(static_cast<int>(i));
  }

  if (n == 0) return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(sum2 / static_cast<double>(n));
}

inline int autoRansacMinInlierPoints(int used_markers)
{
  return std::max(
    6,
    static_cast<int>(std::ceil(0.5 * static_cast<double>(kCornersPerMarker) *
                               static_cast<double>(used_markers))));
}

inline cv::Vec3d vec3dFromCvMat(const cv::Mat& m)
{
  cv::Mat m64;
  m.convertTo(m64, CV_64F);
  cv::Mat flat = m64.reshape(1, 3);
  return cv::Vec3d(flat.at<double>(0, 0), flat.at<double>(1, 0), flat.at<double>(2, 0));
}

inline void rvecTvecFromTf_worldToCamera(const tf2::Transform& T_camera_world,
                                         cv::Vec3d& rvec_cw,
                                         cv::Vec3d& tvec_cw)
{
  const tf2::Matrix3x3& Rtf = T_camera_world.getBasis();
  cv::Mat R = (cv::Mat_<double>(3, 3) <<
    Rtf[0][0], Rtf[0][1], Rtf[0][2],
    Rtf[1][0], Rtf[1][1], Rtf[1][2],
    Rtf[2][0], Rtf[2][1], Rtf[2][2]);

  cv::Rodrigues(R, rvec_cw);

  const tf2::Vector3& t = T_camera_world.getOrigin();
  tvec_cw = cv::Vec3d(t.x(), t.y(), t.z());
}

inline bool estimatePoseSingleMarkerIppeSquare(const Correspondences& corr,
                                               const cv::Mat& K,
                                               const cv::Mat& dist,
                                               const PnPRansacParams& p,
                                               PnPResult& out)
{
  constexpr int kSingleMarkerInliers = kCornersPerMarker;

  if (corr.object_points_world.size() != kCornersPerMarker ||
      corr.image_points_px.size() != kCornersPerMarker ||
      corr.used_marker_world_transforms.size() != 1 ||
      corr.used_marker_sizes_m.size() != 1 ||
      !(corr.used_marker_sizes_m.front() > 0.0)) {
    return false;
  }

  if (p.min_inlier_points > 0 && kSingleMarkerInliers < p.min_inlier_points) {
    return false;
  }

  const auto local_corners = markerLocalCorners(corr.used_marker_sizes_m.front());
  std::vector<cv::Point3f> object_points_marker(local_corners.begin(), local_corners.end());

  std::vector<cv::Mat> rvecs_marker;
  std::vector<cv::Mat> tvecs_marker;
  const int solutions = cv::solvePnPGeneric(
    object_points_marker,
    corr.image_points_px,
    K,
    dist,
    rvecs_marker,
    tvecs_marker,
    false,
    cv::SOLVEPNP_IPPE_SQUARE);

  if (solutions <= 0 || rvecs_marker.empty() || tvecs_marker.empty()) {
    return false;
  }

  const tf2::Transform T_marker_world = corr.used_marker_world_transforms.front().inverse();

  double best_rmse = std::numeric_limits<double>::infinity();
  cv::Vec3d best_rvec_cw{};
  cv::Vec3d best_tvec_cw{};

  const size_t n_solutions = std::min(rvecs_marker.size(), tvecs_marker.size());
  for (size_t i = 0; i < n_solutions; ++i) {
    const cv::Vec3d rvec_cm = vec3dFromCvMat(rvecs_marker[i]);
    const cv::Vec3d tvec_cm = vec3dFromCvMat(tvecs_marker[i]);

    const tf2::Transform T_camera_marker =
      tfFromRvecTvec_worldToCamera(rvec_cm, tvec_cm);
    const tf2::Transform T_camera_world = T_camera_marker * T_marker_world;

    cv::Vec3d rvec_cw;
    cv::Vec3d tvec_cw;
    rvecTvecFromTf_worldToCamera(T_camera_world, rvec_cw, tvec_cw);

    const double rmse = reprojectionRmse(
      corr.object_points_world, corr.image_points_px, K, dist, rvec_cw, tvec_cw);
    if (std::isfinite(rmse) && rmse < best_rmse) {
      best_rmse = rmse;
      best_rvec_cw = rvec_cw;
      best_tvec_cw = tvec_cw;
    }
  }

  if (!std::isfinite(best_rmse)) {
    return false;
  }

  out.rvec_cw = best_rvec_cw;
  out.tvec_cw = best_tvec_cw;
  out.inlier_indices = {0, 1, 2, 3};
  out.reproj_rmse_all = best_rmse;
  out.reproj_rmse_inliers = best_rmse;
  out.method = PnPMethod::SingleMarkerIppeSquare;
  out.used_markers = 1;
  out.required_inlier_points = kSingleMarkerInliers;
  out.covariance_confidence_scale = 2.0;

  return true;
}

inline bool estimatePosePnPRansac(const Correspondences& corr,
                                 const cv::Mat& K, const cv::Mat& dist,
                                 const PnPRansacParams& p,
                                 PnPResult& out)
{
  out = PnPResult{};

  const int used_markers = static_cast<int>(corr.used_marker_ids.size());
  if (p.min_used_markers > 0 && used_markers < p.min_used_markers) return false;
  if (corr.object_points_world.size() < 4 || corr.image_points_px.size() < 4) return false;
  if (corr.object_points_world.size() != corr.image_points_px.size()) return false;

  if (used_markers == 1) {
    return estimatePoseSingleMarkerIppeSquare(corr, K, dist, p, out);
  }

  if (used_markers < 2) return false;

  const int required_inlier_points =
    p.min_inlier_points > 0 ? p.min_inlier_points : autoRansacMinInlierPoints(used_markers);

  cv::Vec3d rvec, tvec;
  std::vector<int> inliers;

  const bool ok = cv::solvePnPRansac(
    corr.object_points_world,
    corr.image_points_px,
    K,
    dist,
    rvec,
    tvec,
    false,
    p.iterations_count,
    p.reprojection_error_px,
    p.confidence,
    inliers,
    p.flags);

  if (!ok) return false;
  if (static_cast<int>(inliers.size()) < required_inlier_points) return false;

  out.rvec_cw = rvec;
  out.tvec_cw = tvec;
  out.inlier_indices = std::move(inliers);
  out.method = PnPMethod::Ransac;
  out.used_markers = used_markers;
  out.required_inlier_points = required_inlier_points;

  // reproj_rmse_all is intentionally NOT computed here — it requires a full
  // projectPoints call over all points but is never consumed downstream.
  // out.reproj_rmse_all stays NaN (default).

  out.reproj_rmse_inliers = reprojectionRmse(
    corr.object_points_world, corr.image_points_px, K, dist, rvec, tvec, &out.inlier_indices);

  return true;
}

// Extra solvePnP with RANSAC init
inline bool refinePoseSolvePnP(const Correspondences& corr,
                              const cv::Mat& K, const cv::Mat& dist,
                              cv::Vec3d& rvec_io, cv::Vec3d& tvec_io,
                              int flags = cv::SOLVEPNP_ITERATIVE)
{
  if (corr.object_points_world.size() < 4) return false;
  return cv::solvePnP(corr.object_points_world,
                      corr.image_points_px,
                      K, dist,
                      rvec_io, tvec_io,
                      true,
                      flags);
}

// ── Радиальная модель шума углов и взвешенное IRLS-уточнение ────────────────
//
// Ошибка углов растёт от центра кадра к краю (fisheye-калибровка хуже всего
// описывает периферию). Вместо отбрасывания краевых меток каждая точка входит
// в решение с весом 1/σ(r)², где r — расстояние ИСХОДНОГО (дисторченного)
// пикселя от principal point, нормированное на полудиагональ кадра:
//
//   σ(r) = sigma0_px · (1 + beta · r^q)
//
// Дефолты перефичены после перекалибровки 2026-07-20 (IMX219-160, crop
// 1280x960, реплей ~101k углов полётных бэгов против mocap-карты):
// 1.50·(1+2.40·r^3) px — плоские ~1.5 px до r≈0.5, ~5 px на дальнем углу.
struct RadialWeightModel
{
  bool enabled{true};
  double sigma0_px{1.50};
  double beta{2.40};
  double q{3.0};

  double sigmaPx(double r_norm) const
  {
    const double r = std::max(0.0, r_norm);
    return std::max(0.05, sigma0_px * (1.0 + beta * std::pow(r, q)));
  }
};

// Гаусс-Ньютон с перевзвешиванием (IRLS, Huber) поверх RANSAC-инициализации.
// Минимизирует Σ wᵢ‖proj(xᵢ)-uᵢ‖², wᵢ = huber(eᵢ/σᵢ)/σᵢ² — краевые точки
// участвуют, но тянут решение слабее пропорционально ожидаемой ошибке.
// Якобиан аналитический из cv::projectPoints (колонки 0-2 d/drvec, 3-5 d/dtvec).
inline bool refinePoseIrlsWeighted(const std::vector<cv::Point3f>& obj,
                                   const std::vector<cv::Point2f>& img,
                                   const std::vector<double>& sigma_px,
                                   const cv::Mat& K, const cv::Mat& dist,
                                   cv::Vec3d& rvec_io, cv::Vec3d& tvec_io,
                                   int iterations = 3, double huber_c = 1.345)
{
  const int n = static_cast<int>(obj.size());
  if (n < 4 ||
      img.size() != obj.size() ||
      sigma_px.size() != obj.size()) {
    return false;
  }

  cv::Vec3d rvec = rvec_io;
  cv::Vec3d tvec = tvec_io;

  for (int it = 0; it < iterations; ++it) {
    std::vector<cv::Point2f> proj;
    cv::Mat jac;
    cv::projectPoints(obj, rvec, tvec, K, dist, proj, jac);
    if (jac.cols < 6 || jac.rows != 2 * n) return false;

    cv::Mat JtWJ = cv::Mat::zeros(6, 6, CV_64F);
    cv::Mat JtWr = cv::Mat::zeros(6, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
      const double rx = static_cast<double>(proj[i].x) - img[i].x;
      const double ry = static_cast<double>(proj[i].y) - img[i].y;
      const double s  = sigma_px[i];
      const double e  = std::sqrt(rx * rx + ry * ry) / s;
      const double huber = (e <= huber_c) ? 1.0 : huber_c / e;
      const double w = huber / (s * s);
      for (int comp = 0; comp < 2; ++comp) {
        const int row = 2 * i + comp;
        const double res = (comp == 0) ? rx : ry;
        double jrow[6];
        for (int c = 0; c < 6; ++c) jrow[c] = jac.at<double>(row, c);
        for (int a = 0; a < 6; ++a) {
          JtWr.at<double>(a, 0) += w * jrow[a] * res;
          for (int b = a; b < 6; ++b) {
            JtWJ.at<double>(a, b) += w * jrow[a] * jrow[b];
          }
        }
      }
    }
    cv::completeSymm(JtWJ);
    JtWJ += cv::Mat::eye(6, 6, CV_64F) * 1e-9;

    cv::Mat delta;
    if (!cv::solve(JtWJ, JtWr, delta, cv::DECOMP_CHOLESKY)) return false;

    for (int c = 0; c < 3; ++c) {
      rvec[c] -= delta.at<double>(c, 0);
      tvec[c] -= delta.at<double>(c + 3, 0);
    }
    if (cv::norm(delta) < 1e-8) break;
  }

  for (int c = 0; c < 3; ++c) {
    if (!std::isfinite(rvec[c]) || !std::isfinite(tvec[c])) return false;
  }
  rvec_io = rvec;
  tvec_io = tvec;
  return true;
}

// Known vertical: keep yaw, roll/pitch from vertical; quaternions share parent frame.

inline bool isFlipped(const tf2::Quaternion& q)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return (std::abs(pitch) > M_PI / 2.0) || (std::abs(roll) > M_PI / 2.0);
}

inline void applyKnownVerticalKeepYaw(tf2::Quaternion& orientation_io,
                                      tf2::Quaternion vertical,
                                      bool flip_vertical = false,
                                      bool auto_flip = false)
{
  orientation_io = normalize(orientation_io);
  vertical = normalize(vertical);

  if (flip_vertical || (auto_flip && !isFlipped(orientation_io))) {
    const tf2::Quaternion flip(tf2::Vector3(1,0,0), M_PI);
    vertical = vertical * flip;
    vertical = normalize(vertical);
  }

  const tf2::Quaternion diff = orientation_io.inverse() * vertical;

  double r, p, yaw_diff;
  tf2::Matrix3x3(diff).getRPY(r, p, yaw_diff);

  const tf2::Quaternion qz(tf2::Vector3(0,0,1), -yaw_diff);
  vertical = vertical * qz;
  orientation_io = normalize(vertical);
}

// ── Перенос prior-ориентации во фрейм гипотезы (для одномаркерного режима) ──
//
// Prior живёт в EKF-фрейме, гипотеза — во фрейме карты меток; их взаимный yaw
// произволен. Перенос — поворот prior вокруг МИРОВОЙ вертикали так, чтобы
// heading (горизонтальная проекция оси X камеры) совпал с heading гипотезы:
// наклон prior к гравитации сохраняется ТОЧНО, курс берётся ArUco-шный.
// NB: applyKnownVerticalKeepYaw для этой задачи непригодна — она крутит вокруг
// ОПТИЧЕСКОЙ оси камеры, что при наклоне tilt и yaw-смещении Δψ уводит ось на
// ~2·tilt·sin(Δψ/2) (измерено на стенде: 20.7° при tilt 11° и Δψ~127°).
inline tf2::Quaternion transportPriorKeepHeading(const tf2::Quaternion& q_hyp,
                                                 const tf2::Quaternion& q_prior)
{
  const tf2::Vector3 x_h = tf2::Matrix3x3(normalize(q_hyp)).getColumn(0);
  const tf2::Vector3 x_p = tf2::Matrix3x3(normalize(q_prior)).getColumn(0);
  const double head_h = std::atan2(x_h.y(), x_h.x());
  const double head_p = std::atan2(x_p.y(), x_p.x());
  const tf2::Quaternion rz(tf2::Vector3(0, 0, 1), head_h - head_p);
  return normalize(rz * normalize(q_prior));
}

// ── Одномаркерный режим: обе IPPE-гипотезы (для выбора по prior) ────────────
//
// estimatePoseSingleMarkerIppeSquare выбирает гипотезу по min RMSE, но в
// около-надирной геометрии селектор ненадёжен (зеркальная ветвь может фититься
// лучше; см. SINGLE_MARKER_JITTER_ANALYSIS.md ч.2.3). Эта функция отдаёт ВСЕ
// гипотезы (мировой фрейм, отсортированы по RMSE) — нода выбирает по
// согласованности с attitude-prior.
struct SingleMarkerHypothesis
{
  cv::Vec3d rvec_cw{};
  cv::Vec3d tvec_cw{};
  double rmse{std::numeric_limits<double>::quiet_NaN()};
};

inline bool estimateSingleMarkerHypotheses(const Correspondences& corr,
                                           const cv::Mat& K, const cv::Mat& dist,
                                           std::vector<SingleMarkerHypothesis>& out)
{
  out.clear();
  if (corr.object_points_world.size() != kCornersPerMarker ||
      corr.image_points_px.size() != kCornersPerMarker ||
      corr.used_marker_world_transforms.size() != 1 ||
      corr.used_marker_sizes_m.size() != 1 ||
      !(corr.used_marker_sizes_m.front() > 0.0)) {
    return false;
  }

  const auto local_corners = markerLocalCorners(corr.used_marker_sizes_m.front());
  std::vector<cv::Point3f> object_points_marker(local_corners.begin(), local_corners.end());

  std::vector<cv::Mat> rvecs_marker;
  std::vector<cv::Mat> tvecs_marker;
  const int solutions = cv::solvePnPGeneric(
    object_points_marker, corr.image_points_px, K, dist,
    rvecs_marker, tvecs_marker, false, cv::SOLVEPNP_IPPE_SQUARE);
  if (solutions <= 0 || rvecs_marker.empty() || tvecs_marker.empty()) {
    return false;
  }

  const tf2::Transform T_marker_world = corr.used_marker_world_transforms.front().inverse();
  const size_t n_solutions = std::min(rvecs_marker.size(), tvecs_marker.size());
  for (size_t i = 0; i < n_solutions; ++i) {
    const cv::Vec3d rvec_cm = vec3dFromCvMat(rvecs_marker[i]);
    const cv::Vec3d tvec_cm = vec3dFromCvMat(tvecs_marker[i]);
    const tf2::Transform T_camera_world =
      tfFromRvecTvec_worldToCamera(rvec_cm, tvec_cm) * T_marker_world;

    SingleMarkerHypothesis h;
    rvecTvecFromTf_worldToCamera(T_camera_world, h.rvec_cw, h.tvec_cw);
    h.rmse = reprojectionRmse(
      corr.object_points_world, corr.image_points_px, K, dist, h.rvec_cw, h.tvec_cw);
    if (std::isfinite(h.rmse)) out.push_back(h);
  }
  std::sort(out.begin(), out.end(),
            [](const SingleMarkerHypothesis& a, const SingleMarkerHypothesis& b) {
              return a.rmse < b.rmse;
            });
  return !out.empty();
}

// ── Одномаркерный режим: трансляция при известной ориентации ────────────────
//
// Свободный 6-DOF PnP по 4 копланарным точкам конвертирует шум углов и
// rolling-shutter шер в наклон, а наклон × высота — в позицию (30°/с корпуса →
// медиана ~14 см ошибки XY; см. SINGLE_MARKER_JITTER_ANALYSIS.md, часть 2).
// При известной ориентации камеры (EKF roll/pitch + ArUco yaw) позиция решается
// линейно и к этим механизмам нечувствительна: для нормализованного луча
// (x̂,ŷ,1) и Y = R_cw·X_world условие коллинеарности (Y+t) ∥ (x̂,ŷ,1) даёт
// 2 линейных уравнения на точку → LSQ 2N×3 на t (world→camera).
//
// R_wc — оси камеры в мире (T_world_camera.getBasis()), УЖЕ после бленда
// known_vertical (roll/pitch от EKF, yaw от ArUco). Для fisheye-пути углы
// приходят ректифицированными к идеальным pinhole-K пикселям с пустым dist —
// undistortPoints тогда только нормализует на K.
inline bool solveTranslationKnownOrientation(
    const std::vector<cv::Point3f>& obj_world,
    const std::vector<cv::Point2f>& img_px,
    const cv::Mat& K, const cv::Mat& dist,
    const tf2::Matrix3x3& R_wc,
    tf2::Vector3& cam_pos_world_out,
    cv::Mat* cov_pos_world = nullptr,
    double corner_sigma_px = 0.5)
{
  const int n = static_cast<int>(obj_world.size());
  if (n < 3 || img_px.size() != obj_world.size() || K.empty()) return false;

  std::vector<cv::Point2f> norm_pts;
  cv::undistortPoints(img_px, norm_pts, K, dist);

  const tf2::Matrix3x3 R_cw = R_wc.transpose();

  cv::Mat A = cv::Mat::zeros(2 * n, 3, CV_64F);
  cv::Mat b = cv::Mat::zeros(2 * n, 1, CV_64F);
  std::vector<double> Yz(n);
  for (int i = 0; i < n; ++i) {
    const tf2::Vector3 X(obj_world[i].x, obj_world[i].y, obj_world[i].z);
    const tf2::Vector3 Y = R_cw * X;
    Yz[i] = Y.z();
    const double xh = norm_pts[i].x;
    const double yh = norm_pts[i].y;
    A.at<double>(2 * i,     0) = 1.0;
    A.at<double>(2 * i,     2) = -xh;
    b.at<double>(2 * i,     0) = xh * Y.z() - Y.x();
    A.at<double>(2 * i + 1, 1) = 1.0;
    A.at<double>(2 * i + 1, 2) = -yh;
    b.at<double>(2 * i + 1, 0) = yh * Y.z() - Y.y();
  }

  cv::Mat t;
  if (!cv::solve(A, b, t, cv::DECOMP_SVD)) return false;
  for (int c = 0; c < 3; ++c) {
    if (!std::isfinite(t.at<double>(c))) return false;
  }

  // Cheirality: все точки строго перед камерой.
  const double tz = t.at<double>(2);
  for (int i = 0; i < n; ++i) {
    if (Yz[i] + tz <= 0.05) return false;
  }

  // p_c = -R_cwᵀ·t = -R_wc·t
  const tf2::Vector3 t_vec(t.at<double>(0), t.at<double>(1), t.at<double>(2));
  cam_pos_world_out = -(R_wc * t_vec);

  if (cov_pos_world) {
    // Σ_t = σ_n²·(AᵀA)⁻¹, σ_n = σ_px/f̄ (шум углов в нормализованных коорд.);
    // Σ_p = R_wc·Σ_t·R_wcᵀ (p = -R_wc·t). Ошибка prior-ориентации добавляется
    // на уровне ноды (изотропный член (h·tan σ_prior)² на xy).
    const double f_mean = 0.5 * (K.at<double>(0, 0) + K.at<double>(1, 1));
    const double sn = corner_sigma_px / std::max(f_mean, 1.0);
    cv::Mat AtA_inv;
    cv::invert(A.t() * A + cv::Mat::eye(3, 3, CV_64F) * 1e-12, AtA_inv, cv::DECOMP_SVD);
    cv::Mat Rm(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) Rm.at<double>(r, c) = R_wc[r][c];
    *cov_pos_world = Rm * AtA_inv * Rm.t() * (sn * sn);
  }
  return true;
}

}
