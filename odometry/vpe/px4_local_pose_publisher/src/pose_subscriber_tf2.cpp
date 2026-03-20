#include "px4_local_pose_publisher/pose_subscriber_tf2.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <stdexcept>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <px4_ros_com/frame_transforms.h>

namespace px4_local_pose_publisher
{
  
VpePublisher::VpePublisher(rclcpp::Node & node)
: LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::Unknown),
  node_(node),
  has_valid_data_(false),
  is_running_(false),
  offset_initialized_(false),
  reset_flag_(true)
{
  RCLCPP_INFO(node_.get_logger(), "Initializing VPE Publisher (ROS 2 Humble)");
  
  declareParameters();
  setupSubscriptions();
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  
  // Timer
  timer_ = nullptr;
  last_data_time_ = timeFromMicros(getTimestamp());
  last_offset_reset_time_ = timeFromMicros(getTimestamp());
  
  // Initialize offset
  offset_.setIdentity();
  
  RCLCPP_INFO(node_.get_logger(), "VPE Publisher initialized. Waiting for data...");

  // Worker thread for heavy work (offset, TF, ENU<->NED).
  processing_thread_ = std::thread(&VpePublisher::processingLoop, this);
}

VpePublisher::~VpePublisher()
{
  stop();
}

void VpePublisher::declareParameters()
{
  // Topic parameters
  node_.declare_parameter("pose_cov_topic", "aruco_map/pose_cov");
  node_.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
  
  // Frame parameters
  node_.declare_parameter("camera_frame_id", "main_camera_optical");
  node_.declare_parameter("base_link_frame_id", "base_link");
  node_.declare_parameter("world_frame_id", "map");
  node_.declare_parameter("aruco_map_frame_id", "aruco_map_detected");
  node_.declare_parameter("offset_frame_id", "aruco_map");
  
  // Behavior parameters
  node_.declare_parameter("allow_offset_timeout_reset", false);
  node_.declare_parameter("allow_send_vio", false);
  node_.declare_parameter("publish_rate_hz", 30.0);
  node_.declare_parameter("offset_timeout", OFFSET_TIMEOUT_DEFAULT);
  node_.declare_parameter("data_timeout_sec", DATA_TIMEOUT_DEFAULT);
  node_.declare_parameter("fallback_variance", FALLBACK_VARIANCE_DEFAULT);

  // Load parameters
  pose_cov_topic_ = node_.get_parameter("pose_cov_topic").as_string();
  vehicle_local_position_topic_ = node_.get_parameter("vehicle_local_position_topic").as_string();
  
  camera_frame_id_ = node_.get_parameter("camera_frame_id").as_string();
  base_link_frame_id_ = node_.get_parameter("base_link_frame_id").as_string();
  world_frame_id_ = node_.get_parameter("world_frame_id").as_string();
  aruco_map_frame_id_ = node_.get_parameter("aruco_map_frame_id").as_string();
  offset_frame_id_ = node_.get_parameter("offset_frame_id").as_string();
  
  allow_offset_timeout_reset_ = node_.get_parameter("allow_offset_timeout_reset").as_bool();
  allow_send_vio_ = node_.get_parameter("allow_send_vio").as_bool();
  publish_rate_hz_ = node_.get_parameter("publish_rate_hz").as_double();
  offset_timeout_ = node_.get_parameter("offset_timeout").as_double();
  data_timeout_sec_ = node_.get_parameter("data_timeout_sec").as_double();
  fallback_variance_ = node_.get_parameter("fallback_variance").as_double();
}

void VpePublisher::setupSubscriptions()
{
  // Subscribe to pose topic per configuration
  pose_cov_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic_, 10,
      std::bind(&VpePublisher::poseCovCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_.get_logger(), "Subscribed to pose with covariance: %s", pose_cov_topic_.c_str());
  
  // PX4 VehicleLocalPosition
  vehicle_local_position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    vehicle_local_position_topic_, rclcpp::QoS(10).best_effort(),
    std::bind(&VpePublisher::vehicleLocalPositionCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_.get_logger(), "Subscribed to vehicle local position: %s", 
              vehicle_local_position_topic_.c_str());
}

void VpePublisher::start()
{
  if (publish_rate_hz_ <= 0) {
    throw std::runtime_error("Invalid publish rate: must be greater than 0");
  }

  timer_ = node_.create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&VpePublisher::timerCallback, this));
  
  is_running_ = true;
  RCLCPP_INFO(node_.get_logger(), "VPE Publisher started at %.1f Hz", publish_rate_hz_);
}

void VpePublisher::stop()
{
  if (timer_) {
    timer_->cancel();
    timer_ = nullptr;
  }
  is_running_ = false;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    worker_stop_ = true;
    pending_pose_cov_valid_ = false;
  }
  queue_cv_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  RCLCPP_INFO(node_.get_logger(), "VPE Publisher stopped");
}

void VpePublisher::poseCovCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Check pose quality via position covariance
  double position_variance = std::max({msg->pose.covariance[0],
                                       msg->pose.covariance[7],
                                       msg->pose.covariance[14]});

  if (position_variance > 1.0) {  // Covariance too large
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                          "High position variance: %.2f, ignoring pose", position_variance);
    return;
  }

  // Lightweight ingest only:
  // - update latest pose snapshot for fallback/timeouts,
  // - enqueue msg in a single-slot queue,
  // - heavy work runs in a separate worker thread.
  const uint64_t now_us = getTimestamp();
  const rclcpp::Time now_time = timeFromMicros(now_us);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_stamped->header = msg->header;
    pose_stamped->pose = msg->pose.pose;  // ROS Pose => ENU

    latest_pose_cov_ = msg;
    latest_pose_ = pose_stamped;
    last_data_time_ = now_time;
    has_valid_data_ = true;
  }

  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    pending_pose_cov_ = msg;              // single-slot: newest replaces oldest
    pending_pose_cov_valid_ = true;
  }
  pose_seq_.fetch_add(1, std::memory_order_relaxed);
  queue_cv_.notify_one();

  return;
}

void VpePublisher::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_local_position_ = msg;
  
  // NOTE: PX4 VehicleLocalPosition fields are NED by convention:
  // x => North, y => East, z => Down.
  RCLCPP_DEBUG(node_.get_logger(), "Received PX4 local position: NED(x=%.3f, y=%.3f, z=%.3f)", 
              msg->x, msg->y, msg->z);
}

void VpePublisher::processingLoop()
{
  while (true) {
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg;
    uint64_t start_seq{0};
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk, [this]() { return worker_stop_ || pending_pose_cov_valid_; });
      if (worker_stop_) {
        return;
      }
      pose_msg = pending_pose_cov_;
      pending_pose_cov_valid_ = false;
      start_seq = pose_seq_.load(std::memory_order_relaxed);
    }

    if (!pose_msg) {
      continue;
    }

    // Snapshot state needed for computation.
    px4_msgs::msg::VehicleLocalPosition::SharedPtr local_position_snapshot;
    tf2::Transform offset_snapshot;
    bool offset_initialized_snapshot{false};
    bool reset_flag_snapshot{false};
    rclcpp::Time last_offset_reset_time_snapshot;
    uint8_t reset_counter_snapshot{0};
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      local_position_snapshot = latest_local_position_;
      offset_snapshot = offset_;
      offset_initialized_snapshot = offset_initialized_;
      reset_flag_snapshot = reset_flag_;
      last_offset_reset_time_snapshot = last_offset_reset_time_;
      reset_counter_snapshot = reset_counter_;
    }

    const uint64_t now_us = getTimestamp();
    const rclcpp::Time current_time = timeFromMicros(now_us);

    auto is_stale = [&]() -> bool {
      return pose_seq_.load(std::memory_order_relaxed) != start_seq;
    };

    // If a newer pose arrived right after we popped this one, skip computation.
    if (is_stale()) {
      continue;
    }

    // Whether offset needs recomputation.
    bool offset_timeout_triggered = false;
    if (allow_offset_timeout_reset_ && offset_initialized_snapshot) {
      const double time_since_offset = (current_time - last_offset_reset_time_snapshot).seconds();
      offset_timeout_triggered = (time_since_offset > offset_timeout_);
    }

    uint8_t local_reset_counter = reset_counter_snapshot;
    bool need_recompute_offset = reset_flag_snapshot || !offset_initialized_snapshot || offset_timeout_triggered;

    // Timeout reset: bump reset counter; offset becomes invalid.
    if (offset_timeout_triggered) {
      if (local_reset_counter < 255) {
        ++local_reset_counter;
      }
    }

    tf2::Transform offset_used = offset_snapshot;
    bool offset_used_initialized = offset_initialized_snapshot;

    // Input pose: T_world_camera (ROS Pose -> ENU).
    tf2::Transform T_world_camera = poseMsgToTf(pose_msg->pose.pose);
    tf2::Transform T_camera_arucomap = T_world_camera.inverse();  // ENU

    // Need camera->base_link TF for base pose.
    geometry_msgs::msg::TransformStamped camera_base_tf;
    tf2::Transform T_camera_base;
    try {
      if (is_stale()) {
        continue;
      }
      camera_base_tf = tf_buffer_->lookupTransform(
        camera_frame_id_, base_link_frame_id_, tf2::TimePointZero);
      tf2::fromMsg(camera_base_tf.transform, T_camera_base);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                            "TF lookup failed (camera->base_link) in processingLoop: %s", e.what());
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        have_latest_measurement_ = false;
      }
      continue;
    }

    // Recompute offset if needed.
    bool offset_compute_success = false;
    if (need_recompute_offset) {
      if (is_stale()) {
        continue;
      }
      // world->base transform in ENU.
      tf2::Transform T_world_base_enu;
      bool have_world_base_tf = false;
      try {
        if (is_stale()) {
          continue;
        }
        geometry_msgs::msg::TransformStamped world_base_tf = tf_buffer_->lookupTransform(
          world_frame_id_, base_link_frame_id_, tf2::TimePointZero);
        tf2::fromMsg(world_base_tf.transform, T_world_base_enu);
        have_world_base_tf = true;
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                              "No TF %s -> %s for offset (fallback to local position): %s",
                              world_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
      }

      if (!have_world_base_tf) {
        if (local_position_snapshot) {
          Eigen::Vector3f pos_ned(local_position_snapshot->x,
                                   local_position_snapshot->y,
                                   local_position_snapshot->z);
          Eigen::Vector3f pos_enu = convertNEDToENUPosition(pos_ned);
          T_world_base_enu.setOrigin(tf2::Vector3(pos_enu.x(), pos_enu.y(), pos_enu.z()));
          T_world_base_enu.setRotation(tf2::Quaternion(0, 0, 0, 1));
        } else {
          // Cannot compute offset without TF or PX4 local position.
          offset_used_initialized = false;
          offset_compute_success = false;
        }
      }

      if (have_world_base_tf || local_position_snapshot) {
        // T_world_arucomap = T_world_base_enu * T_base_camera * T_camera_arucomap
        tf2::Transform T_base_camera = T_camera_base.inverse();
        tf2::Transform offset_out = T_world_base_enu * T_base_camera * T_camera_arucomap;

        offset_used = offset_out;
        offset_used_initialized = true;
        offset_compute_success = true;

        // Static TF for visualization.
        geometry_msgs::msg::TransformStamped offset_tf;
        offset_tf.header.stamp.sec = static_cast<int32_t>(now_us / 1000000);
        offset_tf.header.stamp.nanosec = static_cast<uint32_t>((now_us % 1000000) * 1000);
        offset_tf.header.frame_id = world_frame_id_;
        offset_tf.child_frame_id = offset_frame_id_;
        offset_tf.transform = tf2::toMsg(offset_used);
        static_tf_broadcaster_->sendTransform(offset_tf);
      }
    }

    // Apply offset (or visual pose directly if offset not initialized).
    if (is_stale()) {
      continue;
    }
    tf2::Transform T_world_camera_enu = T_camera_arucomap;
    if (offset_used_initialized) {
      // offset_used is T_world_arucomap; inverse(T_camera_arucomap) is T_arucomap_camera
      tf2::Transform T_arucomap_camera = T_camera_arucomap.inverse();
      T_world_camera_enu = offset_used * T_arucomap_camera;
    }

    // Base pose in world (ENU).
    if (is_stale()) {
      continue;
    }
    tf2::Transform T_world_base_enu = T_world_camera_enu * T_camera_base;

    // Convert ENU to NED for PX4.
    Eigen::Vector3f pos_enu(
      T_world_base_enu.getOrigin().x(),
      T_world_base_enu.getOrigin().y(),
      T_world_base_enu.getOrigin().z());
    Eigen::Vector3f pos_ned = convertENUToNEDPosition(pos_enu);

    tf2::Quaternion quat_enu = T_world_base_enu.getRotation();
    Eigen::Quaternionf eigen_quat_enu(quat_enu.w(), quat_enu.x(), quat_enu.y(), quat_enu.z());
    Eigen::Quaternionf eigen_quat_ned = convertENUToNEDQuaternion(eigen_quat_enu);

    // Build measurement for PX4.
    px4_ros2::LocalPositionMeasurement local_position_measurement{};
    local_position_measurement.timestamp_sample = timeFromMicros(now_us);
    local_position_measurement.reset_counter = local_reset_counter;
    local_position_measurement.position_xy = Eigen::Vector2f(pos_ned.x(), pos_ned.y());
    local_position_measurement.position_xy_variance = Eigen::Vector2f(0.01f, 0.01f);
    local_position_measurement.position_z = pos_ned.z();
    local_position_measurement.position_z_variance = 0.01f;
    local_position_measurement.attitude_quaternion = eigen_quat_ned;
    local_position_measurement.attitude_variance = Eigen::Vector3f(0.001f, 0.001f, 0.001f);

    // Do not commit stale results.
    if (is_stale()) {
      continue;
    }

    // Commit result and update offset if recomputed.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_local_position_measurement_ = local_position_measurement;
      have_latest_measurement_ = true;

      if (offset_compute_success) {
        offset_ = offset_used;
        offset_initialized_ = true;
        reset_flag_ = false;
        last_offset_reset_time_ = current_time;
        reset_counter_ = local_reset_counter;
      } else {
        // Recompute was needed but offset failed; keep invalid.
        if (need_recompute_offset) {
          offset_initialized_ = false;
          reset_flag_ = need_recompute_offset;
          reset_counter_ = local_reset_counter;
          last_offset_reset_time_ = current_time; // similar to resetOffset()
        }
      }
    }

    // RCLCPP_INFO_THROTTLE(
    RCLCPP_DEBUG(
      node_.get_logger(), *node_.get_clock(), 1000,
      "vpe proc: frames world='%s' offset='%s' offset_init_before=%d offset_recompute=%d offset_timeout=%d offset_success=%d reset_counter=%u pos_ned=[%.3f %.3f %.3f] q_ned=[%.3f %.3f %.3f %.3f] offset_origin_ENU=[%.3f %.3f %.3f]",
      world_frame_id_.c_str(), offset_frame_id_.c_str(),
      static_cast<int>(offset_initialized_snapshot),
      static_cast<int>(need_recompute_offset),
      static_cast<int>(offset_timeout_triggered),
      static_cast<int>(offset_compute_success),
      static_cast<unsigned>(local_reset_counter),
      static_cast<double>(pos_ned.x()), static_cast<double>(pos_ned.y()), static_cast<double>(pos_ned.z()),
      static_cast<double>(eigen_quat_ned.w()), static_cast<double>(eigen_quat_ned.x()),
      static_cast<double>(eigen_quat_ned.y()), static_cast<double>(eigen_quat_ned.z()),
      static_cast<double>(offset_used.getOrigin().x()),
      static_cast<double>(offset_used.getOrigin().y()),
      static_cast<double>(offset_used.getOrigin().z()));
  }
}

void VpePublisher::resetOffset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  reset_flag_ = true;
  offset_initialized_ = false;
  last_offset_reset_time_ = timeFromMicros(getTimestamp());
  if (reset_counter_ < 255) {
    ++reset_counter_;
  }
  RCLCPP_INFO(node_.get_logger(), "Offset reset requested, reset_counter=%u",
              static_cast<unsigned>(reset_counter_));
}

bool VpePublisher::computeOffset()
{
  if (!latest_pose_) {
    RCLCPP_DEBUG(node_.get_logger(), "Cannot compute offset: missing visual pose");
    return false;
  }
  
  try {
    // aruco_loc: T_world_camera; need T_camera_arucomap.
    tf2::Transform T_world_camera = poseMsgToTf(latest_pose_->pose);
    // Origin/rotation from ROS pose -> ENU.
    tf2::Transform T_camera_arucomap = T_world_camera.inverse();
    // T_camera_arucomap stays in ENU (transform inverse only).
    
    // Current world->base in ENU.
    // Prefer TF (has orientation/yaw); else PX4 local position.
    tf2::Transform T_world_base_enu;
    bool have_world_base_tf = false;
    try {
      geometry_msgs::msg::TransformStamped world_base_tf = tf_buffer_->lookupTransform(
        world_frame_id_, base_link_frame_id_, tf2::TimePointZero);
      // NOTE: used as ENU base pose (T_world_base_enu); assume ROS world->base TF is ENU
      // (X East, Y North, Z Up).
      tf2::fromMsg(world_base_tf.transform, T_world_base_enu);
      have_world_base_tf = true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                          "No TF %s -> %s for offset (will fallback to local position): %s",
                          world_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
    }

    if (!have_world_base_tf) {
      if (!latest_local_position_) {
        RCLCPP_DEBUG(node_.get_logger(),
                    "Cannot compute offset: no world->base TF and no local position");
        return false;
      }

      Eigen::Vector3f pos_ned(latest_local_position_->x,
                              latest_local_position_->y,
                              latest_local_position_->z);
      // pos_ned: NED (PX4 VehicleLocalPosition).
      Eigen::Vector3f pos_enu = convertNEDToENUPosition(pos_ned);
      // pos_enu: ENU for TF/geometry downstream.
      T_world_base_enu.setOrigin(tf2::Vector3(pos_enu.x(), pos_enu.y(), pos_enu.z()));
      T_world_base_enu.setRotation(tf2::Quaternion(0, 0, 0, 1));

      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                           "Offset fallback uses identity yaw (no world->base TF available)");
    }
    
    // TF camera→base_link
    geometry_msgs::msg::TransformStamped camera_base_tf;
    try {
      camera_base_tf = tf_buffer_->lookupTransform(
        camera_frame_id_, base_link_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(node_.get_logger(), "TF lookup failed: %s", e.what());
      return false;
    }
    
    tf2::Transform T_camera_base;
    tf2::fromMsg(camera_base_tf.transform, T_camera_base);
    
    // Offset world -> aruco_map
    // T_world_arucomap = T_world_base_enu * T_base_camera * T_camera_arucomap
    tf2::Transform T_base_camera = T_camera_base.inverse();
    tf2::Transform T_world_arucomap = T_world_base_enu * T_base_camera * T_camera_arucomap;
    
    // Store offset
    offset_ = T_world_arucomap;
    offset_initialized_ = true;
    
    // Publish static TF for visualization
    geometry_msgs::msg::TransformStamped offset_tf;
    const uint64_t now_us = getTimestamp();
    offset_tf.header.stamp.sec = static_cast<int32_t>(now_us / 1000000);
    offset_tf.header.stamp.nanosec = static_cast<uint32_t>((now_us % 1000000) * 1000);
    offset_tf.header.frame_id = world_frame_id_;
    offset_tf.child_frame_id = offset_frame_id_;
    offset_tf.transform = tf2::toMsg(offset_);
    static_tf_broadcaster_->sendTransform(offset_tf);
    
    last_offset_reset_time_ = timeFromMicros(getTimestamp());
    
    RCLCPP_INFO(node_.get_logger(), "Offset computed and published as TF: %s -> %s",
                world_frame_id_.c_str(), offset_frame_id_.c_str());
    RCLCPP_INFO(node_.get_logger(), "Offset translation: [%.2f, %.2f, %.2f]",
                // Treat offset_.origin.{x,y,z} as ENU (ROS).
                offset_.getOrigin().x(), offset_.getOrigin().y(), offset_.getOrigin().z());
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_.get_logger(), "Error computing offset: %s", e.what());
    return false;
  }
}

void VpePublisher::applyOffset(tf2::Transform& transform)
{
  if (!offset_initialized_) {
    return;
  }
  
  // Apply offset in ENU: offset_ = T_world_arucomap, transform = T_camera_arucomap;
  // T_world_camera = offset_ * inverse(transform).
  
  tf2::Transform T_arucomap_camera = transform.inverse();
  tf2::Transform T_world_camera = offset_ * T_arucomap_camera;
  
  transform = T_world_camera;
}

void VpePublisher::timerCallback()
{
  if (!is_running_.load()) {
    return;
  }

  const rclcpp::Time current_time = timeFromMicros(getTimestamp());
  
  // Take precomputed values and check data freshness.
  px4_ros2::LocalPositionMeasurement local_position_measurement{};
  bool have_measurement{false};
  bool has_valid_data{false};
  rclcpp::Time last_time;
  bool offset_initialized_snapshot{false};
  bool latest_pose_exists{false};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_time = last_data_time_;
    has_valid_data = has_valid_data_;
    have_measurement = have_latest_measurement_;
    local_position_measurement = latest_local_position_measurement_;
    offset_initialized_snapshot = offset_initialized_;
    latest_pose_exists = static_cast<bool>(latest_pose_);
  }

  const double time_since_last_data = (current_time - last_time).seconds();

  // Timeout: no fresh data
  if (time_since_last_data > data_timeout_sec_ || !has_valid_data || !latest_pose_exists || !have_measurement) {
    // RCLCPP_INFO_THROTTLE(
    RCLCPP_DEBUG(
      node_.get_logger(), *node_.get_clock(), 1000,
      "vpe timer->fallback: time_since_last_data=%.3f timeout=%.3f has_valid_data=%d latest_pose=%d have_measurement=%d offset_init=%d allow_send_vio=%d",
      time_since_last_data, data_timeout_sec_,
      static_cast<int>(has_valid_data),
      static_cast<int>(latest_pose_exists),
      static_cast<int>(have_measurement),
      static_cast<int>(offset_initialized_snapshot),
      static_cast<int>(allow_send_vio_));
    sendFallbackData();
    return;
  }

  try {
    // 1) Send precomputed NED state to PX4
    if (allow_send_vio_) {
      // position_* and attitude_quaternion filled in processingLoop() (NED).
      update(local_position_measurement);

      const uint8_t reset_counter_value = local_position_measurement.reset_counter.value_or(0);
      const auto pos_xy = local_position_measurement.position_xy.value();
      const float pos_z = local_position_measurement.position_z.value_or(0.0f);
      const auto q = local_position_measurement.attitude_quaternion.value();

      // RCLCPP_INFO_THROTTLE(
      RCLCPP_DEBUG(
        node_.get_logger(), *node_.get_clock(), 1000,
        "vpe timer->update: allow_send_vio=1 offset_init=%d reset_counter=%u pos_ned_xy=[%.3f %.3f] pos_ned_z=%.3f q_ned=[%.3f %.3f %.3f %.3f]",
        static_cast<int>(offset_initialized_snapshot),
        static_cast<unsigned>(reset_counter_value),
        static_cast<double>(pos_xy.x()), static_cast<double>(pos_xy.y()),
        static_cast<double>(pos_z),
        static_cast<double>(q.w()), static_cast<double>(q.x()),
        static_cast<double>(q.y()), static_cast<double>(q.z()));
    }
    else {
      // RCLCPP_INFO_THROTTLE(
      RCLCPP_DEBUG(
        node_.get_logger(), *node_.get_clock(), 1000,
        "vpe timer: allow_send_vio=0 -> skipping update (offset_init=%d)",
        static_cast<int>(offset_initialized_snapshot));
    }

    // 2) Refresh offset timestamp after send
    if (offset_initialized_snapshot) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_offset_reset_time_ = current_time;
      RCLCPP_DEBUG(node_.get_logger(),
        "Updated offset timestamp: sent to PX4 (pos in NED)");
    }

  } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "Navigation interface error: %s", e.what());
    sendFallbackData();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "Exception: %s", e.what());
    sendFallbackData();
  }
}

void VpePublisher::sendFallbackData()
{
  px4_ros2::LocalPositionMeasurement fallback_measurement{};
  // State snapshot under mutex (independent of concurrent offset recompute).
  tf2::Transform last_known_pose;
  bool has_last_pose = false;
  uint8_t reset_counter_snapshot = 0;
  bool offset_initialized_snapshot = false;
  tf2::Transform offset_snapshot;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    reset_counter_snapshot = reset_counter_;
    offset_initialized_snapshot = offset_initialized_;
    offset_snapshot = offset_;

    if (latest_pose_) {
      last_known_pose = poseMsgToTf(latest_pose_->pose);
      // last_known_pose: ENU T_world_camera.
      has_last_pose = true;
    }
  }

  fallback_measurement.timestamp_sample = timeFromMicros(getTimestamp());
  fallback_measurement.reset_counter = reset_counter_snapshot;
  
  // Use last known values when available
  if (has_last_pose && offset_initialized_snapshot) {
    // last_known_pose from aruco is T_world_camera; invert; then apply offset:
    tf2::Transform T_camera_arucomap = last_known_pose.inverse();
    tf2::Transform T_arucomap_camera = T_camera_arucomap.inverse();
    tf2::Transform T_world_camera_enu = offset_snapshot * T_arucomap_camera;

    // For fallback also compute base_link pose (same as processingLoop);
    // otherwise PX4 gets camera pose interpreted as body-local.
    tf2::Transform T_world_base_enu = T_world_camera_enu;
    try {
      geometry_msgs::msg::TransformStamped camera_base_tf = tf_buffer_->lookupTransform(
        camera_frame_id_, base_link_frame_id_, tf2::TimePointZero);
      tf2::Transform T_camera_base;
      tf2::fromMsg(camera_base_tf.transform, T_camera_base);
      T_world_base_enu = T_world_camera_enu * T_camera_base;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "TF lookup failed (camera->base_link) in sendFallbackData: %s", e.what());
    }

    // Convert to NED
    Eigen::Vector3f pos_enu(T_world_base_enu.getOrigin().x(),
                              T_world_base_enu.getOrigin().y(),
                              T_world_base_enu.getOrigin().z());
    Eigen::Vector3f pos_ned = convertENUToNEDPosition(pos_enu);
    
    tf2::Quaternion quat_enu = T_world_base_enu.getRotation();
    Eigen::Quaternionf eigen_quat_enu(quat_enu.w(), quat_enu.x(), quat_enu.y(), quat_enu.z());
    Eigen::Quaternionf eigen_quat_ned = convertENUToNEDQuaternion(eigen_quat_enu);
    
    fallback_measurement.position_xy = Eigen::Vector2f(pos_ned.x(), pos_ned.y());
    fallback_measurement.position_z = pos_ned.z();
    fallback_measurement.attitude_quaternion = eigen_quat_ned;
  } else {
    // No last pose: use zeros
    fallback_measurement.position_xy = Eigen::Vector2f(0.0f, 0.0f);
    fallback_measurement.position_z = 0.0f;
    fallback_measurement.attitude_quaternion = Eigen::Quaternionf::Identity();
  }
   
  // Fallback: high variance (unreliable measurement)
  fallback_measurement.position_xy_variance = Eigen::Vector2f(0.5f, 0.5f);
  fallback_measurement.position_z_variance = 0.5f;
  fallback_measurement.attitude_variance = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
  
  try {
    const uint8_t reset_counter_value = fallback_measurement.reset_counter.value_or(0);
    const auto pos_xy = fallback_measurement.position_xy.value();
    const float pos_z = fallback_measurement.position_z.value_or(0.0f);
    const auto q = fallback_measurement.attitude_quaternion.value();

    // RCLCPP_INFO_THROTTLE(
    RCLCPP_DEBUG(
      node_.get_logger(), *node_.get_clock(), 1000,
      "vpe fallback payload: allow_send_vio=%d offset_init=%d has_last_pose=%d reset_counter=%u pos_ned_xy=[%.3f %.3f] pos_ned_z=%.3f q_ned=[%.3f %.3f %.3f %.3f] var_xy=[%.3f %.3f] var_z=%.3f",
      static_cast<int>(allow_send_vio_),
      static_cast<int>(offset_initialized_snapshot),
      static_cast<int>(has_last_pose),
      static_cast<unsigned>(reset_counter_value),
      static_cast<double>(pos_xy.x()), static_cast<double>(pos_xy.y()),
      static_cast<double>(pos_z),
      static_cast<double>(q.w()), static_cast<double>(q.x()),
      static_cast<double>(q.y()), static_cast<double>(q.z()),
      static_cast<double>(fallback_measurement.position_xy_variance.value().x()),
      static_cast<double>(fallback_measurement.position_xy_variance.value().y()),
      static_cast<double>(fallback_measurement.position_z_variance.value_or(0.0f)));

    if (allow_send_vio_){
      update(fallback_measurement);
    }
    // RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
    //   "Sent fallback data with high variance (%.1f)", 
    //   static_cast<double>(fallback_variance_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
      "Failed to send fallback: %s", e.what());
  }
}

uint64_t VpePublisher::getTimestamp()
{
  return static_cast<uint64_t>(node_.now().nanoseconds() / 1000);
}

rclcpp::Time VpePublisher::timeFromMicros(uint64_t us) const
{
  return rclcpp::Time(static_cast<int64_t>(us) * 1000);
}

// Coordinate conversion utilities
Eigen::Quaternionf VpePublisher::convertENUToNEDQuaternion(const Eigen::Quaternionf & quat_enu)
{
  // quat_enu: orientation in ENU (ROS convention in this code).
  // Returns quaternion in NED for PX4 EKF / LocalPositionMeasurement.
  Eigen::Quaterniond q_enu_d(
    static_cast<double>(quat_enu.w()),
    static_cast<double>(quat_enu.x()),
    static_cast<double>(quat_enu.y()),
    static_cast<double>(quat_enu.z()));

  Eigen::Quaterniond q_ned_d =
    px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu_d);

  return Eigen::Quaternionf(
    static_cast<float>(q_ned_d.w()),
    static_cast<float>(q_ned_d.x()),
    static_cast<float>(q_ned_d.y()),
    static_cast<float>(q_ned_d.z()));
}

Eigen::Vector3f VpePublisher::convertENUToNEDPosition(const Eigen::Vector3f & pos_enu)
{
  // pos_enu in ENU; return NED for PX4.
  Eigen::Vector3d pos_enu_d(
    static_cast<double>(pos_enu.x()),
    static_cast<double>(pos_enu.y()),
    static_cast<double>(pos_enu.z()));

  Eigen::Vector3d pos_ned_d =
    px4_ros_com::frame_transforms::enu_to_ned_local_frame(pos_enu_d);

  return Eigen::Vector3f(
    static_cast<float>(pos_ned_d.x()),
    static_cast<float>(pos_ned_d.y()),
    static_cast<float>(pos_ned_d.z()));
}

Eigen::Quaternionf VpePublisher::convertNEDToENUQuaternion(const Eigen::Quaternionf & quat_ned)
{
  // quat_ned from PX4 (NED); return ENU.
  Eigen::Quaterniond q_ned_d(
    static_cast<double>(quat_ned.w()),
    static_cast<double>(quat_ned.x()),
    static_cast<double>(quat_ned.y()),
    static_cast<double>(quat_ned.z()));

  Eigen::Quaterniond q_enu_d =
    px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned_d);

  return Eigen::Quaternionf(
    static_cast<float>(q_enu_d.w()),
    static_cast<float>(q_enu_d.x()),
    static_cast<float>(q_enu_d.y()),
    static_cast<float>(q_enu_d.z()));
}

Eigen::Vector3f VpePublisher::convertNEDToENUPosition(const Eigen::Vector3f & pos_ned)
{
  // pos_ned from PX4 (NED); return ENU.
  Eigen::Vector3d pos_ned_d(
    static_cast<double>(pos_ned.x()),
    static_cast<double>(pos_ned.y()),
    static_cast<double>(pos_ned.z()));

  Eigen::Vector3d pos_enu_d =
    px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned_d);

  return Eigen::Vector3f(
    static_cast<float>(pos_enu_d.x()),
    static_cast<float>(pos_enu_d.y()),
    static_cast<float>(pos_enu_d.z()));
}

tf2::Transform VpePublisher::poseMsgToTf(const geometry_msgs::msg::Pose& pose)
{
  // No axis remapping: copy position/orientation only.
  // In this node pose.position.* is treated as ENU (ROS).
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w));
  return tf;
}

geometry_msgs::msg::Pose VpePublisher::tfToPoseMsg(const tf2::Transform& tf)
{
  // Pass-through: origin/rotation unchanged.
  // ENU tf -> ENU Pose; NED tf -> NED Pose.
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  pose.orientation.x = tf.getRotation().x();
  pose.orientation.y = tf.getRotation().y();
  pose.orientation.z = tf.getRotation().z();
  pose.orientation.w = tf.getRotation().w();
  return pose;
}

// Node
VpePublisherNode::VpePublisherNode()
: Node("vpe_publisher")
{
  // Optional verbose logging (rcutils DEBUG)
  if (false) { // set true for verbose debug
    auto ret = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error setting log severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  RCLCPP_INFO(get_logger(), "Starting VPE Publisher Node for ROS 2 Humble");
  
  interface_ = std::make_unique<VpePublisher>(*this);

  // Register with PX4
  if (!interface_->doRegister()) {
    throw std::runtime_error("Failed to register with PX4 navigation interface");
  }

  RCLCPP_INFO(get_logger(), "Successfully registered with PX4");

  // Start processing
  interface_->start();

  // Wall timer with empty callback (same pattern as sibling nodes in this package)
  shutdown_timer_ = this->create_wall_timer(
    100ms,
    [this]() {
    });
}

VpePublisherNode::~VpePublisherNode()
{
  if (interface_) {
    interface_->stop();
  }
}

} // namespace px4_local_pose_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<px4_local_pose_publisher::VpePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}