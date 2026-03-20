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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"


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

class ArucoLocNode : public rclcpp::Node
{
public:
  ArucoLocNode()
  : rclcpp::Node("aruco_loc_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "Initializing ArucoLocNode");

    // Node behavior parameters
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>("markers_topic", "markers");
    declare_parameter<std::string>("map_markers_topic", "map_markers");
    declare_parameter<std::string>(
      "camera_info_topic",
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info");

    // Output
    declare_parameter<std::string>("pose_topic", "aruco_map/pose_cov");

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

    // Optional refine after RANSAC
    declare_parameter<bool>("refine_pose", false);

    // PnP / RANSAC parameters
    declare_parameter<int>("pnp.iterations", 150);
    declare_parameter<double>("pnp.reprojection_error_px", 4.0);
    declare_parameter<double>("pnp.confidence", 0.99);
    declare_parameter<int>("pnp.flags", static_cast<int>(cv::SOLVEPNP_ITERATIVE));

    declare_parameter<int>("pnp.min_used_markers", 1);
    declare_parameter<int>("pnp.min_inlier_points", 4);

    // RMSE gating threshold
    declare_parameter<double>("gating.max_inlier_rmse_px", 10.0);

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

    refine_pose_           = get_parameter("refine_pose").as_bool();
    max_inlier_rmse_px_    = get_parameter("gating.max_inlier_rmse_px").as_double();
    // PnP parameters
    pnp_params_.iterations_count      = get_parameter("pnp.iterations").as_int();
    pnp_params_.reprojection_error_px = static_cast<float>(get_parameter("pnp.reprojection_error_px").as_double());
    pnp_params_.confidence            = get_parameter("pnp.confidence").as_double();
    pnp_params_.flags                 = get_parameter("pnp.flags").as_int();
    pnp_params_.min_used_markers      = get_parameter("pnp.min_used_markers").as_int();
    pnp_params_.min_inlier_points     = get_parameter("pnp.min_inlier_points").as_int();

    // TF broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Publishers
    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);

    // Map republisher (latched)
    map_pub_ = create_publisher<MarkerArrayMsg>(
      "aruco_map/map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Subscriptions
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ArucoLocNode::cameraInfoCb, this, std::placeholders::_1));

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

    RCLCPP_INFO(get_logger(), "aruco_loc_node ready (debug image stripped)");
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

      // TF parameters
      else if (name == "publish_tf") publish_tf_ = p.as_bool();
      else if (name == "publish_static_markers_tf") publish_static_markers_tf_ = p.as_bool();
      else if (name == "detect_individual_markers") detect_individual_markers_ = p.as_bool();
      else if (name == "put_markers_count_to_covariance") put_markers_count_to_covariance_ = p.as_bool();

      // Topics (runtime change)
      else if (name == "markers_topic") {
        markers_topic_ = p.as_string();
      } else if (name == "map_markers_topic") {
        map_markers_topic_ = p.as_string();
      } else if (name == "camera_info_topic") {
        camera_info_topic_ = p.as_string();
      }       else if (name == "pose_topic") {
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

  cv::Mat estimateCovarianceFromJacobian(
      const cv::Vec3d& rvec,
      const cv::Vec3d& tvec,
      const std::vector<cv::Point3f>& object_points,
      const std::vector<cv::Point2f>& image_points,
      double pixel_error = 0.5,
      double epsilon = 1e-6)
  {
    const int num_points = static_cast<int>(object_points.size());
    if (num_points < 3) {
      return cv::Mat::zeros(6, 6, CV_64F);
    }

    cv::Mat jacobian(2 * num_points, 6, CV_64F, cv::Scalar(0));

    // [tvec_x, tvec_y, tvec_z, rvec_x, rvec_y, rvec_z]
    std::vector<double> params(6);
    params[0] = tvec[0];
    params[1] = tvec[1];
    params[2] = tvec[2];
    params[3] = rvec[0];
    params[4] = rvec[1];
    params[5] = rvec[2];

    for (int i = 0; i < 6; ++i) {
      auto params_plus  = params;
      auto params_minus = params;

      params_plus[i]  += epsilon;
      params_minus[i] -= epsilon;

      cv::Vec3d tvec_plus(params_plus[0], params_plus[1], params_plus[2]);
      cv::Vec3d rvec_plus(params_plus[3], params_plus[4], params_plus[5]);
      cv::Vec3d tvec_minus(params_minus[0], params_minus[1], params_minus[2]);
      cv::Vec3d rvec_minus(params_minus[3], params_minus[4], params_minus[5]);

      std::vector<cv::Point2f> img_points_plus, img_points_minus;
      cv::projectPoints(object_points, rvec_plus,  tvec_plus,  camera_matrix_, dist_coeffs_, img_points_plus);
      cv::projectPoints(object_points, rvec_minus, tvec_minus, camera_matrix_, dist_coeffs_, img_points_minus);

      for (int j = 0; j < num_points; ++j) {
        const double dx = (img_points_plus[j].x - img_points_minus[j].x) / (2.0 * epsilon);
        const double dy = (img_points_plus[j].y - img_points_minus[j].y) / (2.0 * epsilon);

        jacobian.at<double>(2 * j,     i) = dx;
        jacobian.at<double>(2 * j + 1, i) = dy;
      }
    }

    cv::Mat R = cv::Mat::eye(2 * num_points, 2 * num_points, CV_64F) * (pixel_error * pixel_error);

    const cv::Mat Jt  = jacobian.t();
    const cv::Mat JtJ = Jt * jacobian;

    const cv::Mat regularization = cv::Mat::eye(6, 6, CV_64F) * 1e-6;
    const cv::Mat JtJ_reg = JtJ + regularization;

    cv::Mat JtJ_inv;
    cv::invert(JtJ_reg, JtJ_inv, cv::DECOMP_SVD);

    cv::Mat covariance = JtJ_inv * Jt * R * jacobian * JtJ_inv;
    return covariance;
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
      camera_ready_ = true;
      RCLCPP_INFO(get_logger(), "Camera calibration loaded");
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

    if (detect_individual_markers_) {
      map_pub_->publish(*msg);
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
    const auto corr = loc_utils::buildCorrespondences(*det_msg, map_copy);
    if (corr.object_points_world.size() < 4) {
      return;
    }

    // Detections present on map
    int valid_markers_count = 0;
    for (const auto& marker : det_msg->markers) {
      if (map_copy.find(marker.id) != map_copy.end()) {
        valid_markers_count++;
      }
    }

    // Pose estimate (PnP)
    loc_utils::PnPResult pnp;
    if (!loc_utils::estimatePosePnPRansac(corr, camera_matrix_, dist_coeffs_, pnp_params_, pnp)) {
      return;
    }

    // solvePnP refine
    if (refine_pose_) {
      (void)loc_utils::refinePoseSolvePnP(
        corr, camera_matrix_, dist_coeffs_,
        pnp.rvec_cw, pnp.tvec_cw,
        static_cast<int>(cv::SOLVEPNP_ITERATIVE));
    }

    // Inlier RMSE gating
    if (max_inlier_rmse_px_ > 0.0 && std::isfinite(pnp.reproj_rmse_inliers) &&
        pnp.reproj_rmse_inliers > max_inlier_rmse_px_) {
      return;
    }

    // world→camera → world←camera
    const tf2::Transform T_camera_world =
      loc_utils::tfFromRvecTvec_worldToCamera(pnp.rvec_cw, pnp.tvec_cw);
    tf2::Transform T_world_camera = T_camera_world.inverse();

    // Optional known-vertical blend
    if (!known_vertical_frame_.empty()) {
      try {
        const auto vertical_tf =
          tf_buffer_.lookupTransform(world_frame, known_vertical_frame_, tf2::TimePointZero);

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

        loc_utils::applyKnownVerticalKeepYaw(q_wc, q_wcam_v, flip_vertical_, auto_flip_);
        T_world_camera.setRotation(q_wc);
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "known_vertical lookup failed: %s", e.what());
      }
    }

    // PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header.stamp = det_msg->header.stamp;
    pose_cov_msg.header.frame_id = tf_parent_frame_;
    pose_cov_msg.pose.pose = loc_utils::poseMsgFromTf(T_world_camera);

    // Covariance
    if (put_markers_count_to_covariance_) {
      pose_cov_msg.pose.covariance.fill(0.0);
      pose_cov_msg.pose.covariance[0] = static_cast<double>(valid_markers_count);
    } else {
      std::vector<cv::Point3f> object_points_vec(
        corr.object_points_world.begin(),
        corr.object_points_world.end());

      std::vector<cv::Point2f> image_points_vec(
        corr.image_points_px.begin(),
        corr.image_points_px.end());

      const double reprojection_error =
        std::isfinite(pnp.reproj_rmse_inliers) ? pnp.reproj_rmse_inliers : 1.0;

      cv::Mat pose_covariance = estimatePoseCovariance(
        pnp.rvec_cw, pnp.tvec_cw,
        object_points_vec, image_points_vec,
        valid_markers_count, reprojection_error);

      const auto cov_array = covarianceToRosArray(pose_covariance);
      std::copy(cov_array.begin(), cov_array.end(), pose_cov_msg.pose.covariance.begin());
    }

    pose_cov_pub_->publish(pose_cov_msg);

    // if (!debug_rotation_csv_.empty()) {
    //   auto now = this->now();
    //   if ((now - last_debug_log_time_).seconds() >= 1.0) {
    //     last_debug_log_time_ = now;
    //     double pose_yaw = tf2::impl::getYaw(T_world_camera.getRotation());
    //     std::ofstream f(debug_rotation_csv_, std::ios::app);
    //     if (f.is_open()) {
    //       static bool header_done = false;
    //       if (!header_done) {
    //         f << "timestamp,package,pose_x,pose_y,pose_z,pose_yaw,valid_markers_count,reproj_rmse\n";
    //         header_done = true;
    //       }
    //       double rmse = std::isfinite(pnp.reproj_rmse_inliers) ? pnp.reproj_rmse_inliers : 0.0;
    //       f << now.seconds() << ",aruco_loc,"
    //         << T_world_camera.getOrigin().x() << "," << T_world_camera.getOrigin().y() << "," << T_world_camera.getOrigin().z() << ","
    //         << pose_yaw << "," << valid_markers_count << "," << rmse << "\n";
    //     }
    //   }
    // }

    if (publish_tf_ && !tf_child_frame_.empty()) {
      geometry_msgs::msg::TransformStamped camera_map_tf;
      camera_map_tf.header.stamp = det_msg->header.stamp;
      camera_map_tf.header.frame_id = tf_parent_frame_;
      camera_map_tf.child_frame_id = tf_child_frame_;
      camera_map_tf.transform = tf2::toMsg(T_camera_world);

      tf_broadcaster_->sendTransform(camera_map_tf);
    }
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

  bool refine_pose_{false};
  double max_inlier_rmse_px_{10.0};

  // Camera intrinsics
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_ready_{false};

  // Marker map cache
  std::mutex map_mutex_;
  loc_utils::MapIndex map_index_;
  bool map_ready_{false};
  std::string map_frame_id_;

  loc_utils::PnPRansacParams pnp_params_;

  // Subscriptions and publishers
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<MarkerArrayMsg>::SharedPtr map_markers_sub_;
  rclcpp::Subscription<MarkerArrayMsg>::SharedPtr markers_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr map_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace aruco_det_loc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_det_loc::ArucoLocNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
