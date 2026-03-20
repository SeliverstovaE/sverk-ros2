#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include <atomic>


using namespace std::chrono_literals;

namespace px4_local_pose_publisher
{

class VpePublisher : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  explicit VpePublisher(rclcpp::Node & node);
  ~VpePublisher();

  void start();
  void stop();

private:
  /** Timestamp in microseconds (single time base for offboard/PX4 sync). */
  uint64_t getTimestamp();
  /** Convert microseconds to rclcpp::Time for ROS 2 message fields. */
  rclcpp::Time timeFromMicros(uint64_t us) const;

  void declareParameters();
  void setupSubscriptions();
  
  // Callbacks
  void poseCovCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void timerCallback();
  
  // Core logic
  void resetOffset();
  bool computeOffset();
  void applyOffset(tf2::Transform& transform);
  void sendFallbackData();
  
  // Conversions: ROS ENU (X east, Y north, Z up) <-> PX4 NED (X north, Y east, Z down).
  Eigen::Quaternionf convertENUToNEDQuaternion(const Eigen::Quaternionf & quat_enu);   // ENU -> NED
  Eigen::Vector3f convertENUToNEDPosition(const Eigen::Vector3f & pos_enu);         // ENU -> NED
  Eigen::Quaternionf convertNEDToENUQuaternion(const Eigen::Quaternionf & quat_ned);   // NED -> ENU
  Eigen::Vector3f convertNEDToENUPosition(const Eigen::Vector3f & pos_ned);         // NED -> ENU
  
  // TF
  tf2::Transform poseMsgToTf(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Pose tfToPoseMsg(const tf2::Transform& tf);

  // Node reference
  rclcpp::Node & node_;
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  // Data
  std::mutex data_mutex_;
  // latest_pose_: T_world_camera from pose_cov_topic_; Pose.position.* treated as ENU (ROS).
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_cov_;
  // latest_local_position_: PX4 VehicleLocalPosition (NED).
  px4_msgs::msg::VehicleLocalPosition::SharedPtr latest_local_position_;
  rclcpp::Time last_data_time_;
  bool has_valid_data_;
  std::atomic<bool> is_running_{false};

  // Precomputed PX4 measurement (updated in processingLoop()); timerCallback() sends via update().
  bool have_latest_measurement_{false};
  // position_* in NED (PX4 convention).
  px4_ros2::LocalPositionMeasurement latest_local_position_measurement_{};
  
  // Offset (map/world -> aruco_map), computed/applied in ENU (ROS TF).
  tf2::Transform offset_;  // world (map) -> aruco_map (ENU)
  bool offset_initialized_;
  bool reset_flag_;
  rclcpp::Time last_offset_reset_time_;
  /** PX4 EKF2 reset counter: incremented on offset reset (relocalization / discontinuity). */
  uint8_t reset_counter_ {0};

  // ===========================
  // Worker thread + queue
  // ===========================
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool worker_stop_{false};
  // Single-slot queue: only the newest pose is processed.
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pending_pose_cov_;
  bool pending_pose_cov_valid_{false};
  std::thread processing_thread_;

  // Incremented on each new pose in poseCovCallback(); worker checks to avoid stale commits.
  std::atomic<uint64_t> pose_seq_{0};

  void processingLoop();
  
  // Parameters
  std::string pose_topic_;
  std::string pose_cov_topic_;
  std::string vehicle_local_position_topic_;
  std::string camera_frame_id_;
  std::string base_link_frame_id_;
  std::string world_frame_id_;
  std::string aruco_map_frame_id_;
  std::string offset_frame_id_;
  
  bool use_pose_cov_;
  bool allow_offset_timeout_reset_;
  bool allow_send_vio_;
  double publish_rate_hz_;
  double offset_timeout_;
  double data_timeout_sec_;
  float fallback_variance_;
  
  // Default timeouts / variances
  const double OFFSET_TIMEOUT_DEFAULT = 3.0;
  const double DATA_TIMEOUT_DEFAULT = 0.5;
  const float FALLBACK_VARIANCE_DEFAULT = 0.5f;
};

class VpePublisherNode : public rclcpp::Node
{
public:
  VpePublisherNode();
  ~VpePublisherNode();

private:
  std::unique_ptr<VpePublisher> interface_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

} // namespace px4_local_pose_publisher
