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
};

inline Correspondences buildCorrespondences(const MarkerArrayMsg& det_msg, const MapIndex& map_index)
{
  Correspondences corr;
  corr.object_points_world.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.image_points_px.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.point_marker_ids.reserve(det_msg.markers.size() * kCornersPerMarker);
  corr.used_marker_ids.reserve(det_msg.markers.size());

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
  }

  return corr;
}

// PnP and reprojection metrics

struct PnPRansacParams
{
  int iterations_count{150};
  float reprojection_error_px{4.0f};
  double confidence{0.99};
  int flags{cv::SOLVEPNP_ITERATIVE};

  int min_used_markers{1};
  int min_inlier_points{4};
};

struct PnPResult
{
  cv::Vec3d rvec_cw{};
  cv::Vec3d tvec_cw{};
  std::vector<int> inlier_indices;
  double reproj_rmse_all{std::numeric_limits<double>::quiet_NaN()};
  double reproj_rmse_inliers{std::numeric_limits<double>::quiet_NaN()};
};

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

inline bool estimatePosePnPRansac(const Correspondences& corr,
                                 const cv::Mat& K, const cv::Mat& dist,
                                 const PnPRansacParams& p,
                                 PnPResult& out)
{
  out = PnPResult{};

  const int used_markers = static_cast<int>(corr.used_marker_ids.size());
  if (used_markers < p.min_used_markers) return false;
  if (corr.object_points_world.size() < 4 || corr.image_points_px.size() < 4) return false;
  if (corr.object_points_world.size() != corr.image_points_px.size()) return false;

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
  if (static_cast<int>(inliers.size()) < p.min_inlier_points) return false;

  out.rvec_cw = rvec;
  out.tvec_cw = tvec;
  out.inlier_indices = std::move(inliers);

  out.reproj_rmse_all = reprojectionRmse(
    corr.object_points_world, corr.image_points_px, K, dist, rvec, tvec, nullptr);

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

}