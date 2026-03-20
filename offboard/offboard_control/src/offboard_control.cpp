#include "offboard_control/offboard_control.hpp"


OffboardControlNode::OffboardControlNode(std::string px4_namespace)
: Node("offboard_control"),
  tf_buffer_(this->get_clock()),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)),
  tf_broadcaster_(this),
  static_tf_broadcaster_(this)
{
    // --- Parameters ---
    this->declare_parameter<std::string>("body_frame_id", "base_link");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("aruco_map_frame_id", "aruco_map");
    this->declare_parameter<double>("transform_timeout", 0.5);
    this->declare_parameter<double>("offboard_timeout", 10.0);
    this->declare_parameter<double>("arming_timeout", 5.0);
    this->declare_parameter<double>("default_speed", 0.5);
    this->declare_parameter<bool>("land_only_in_offboard", true);
    this->declare_parameter<bool>("check_kill_switch", true);
    this->declare_parameter<double>("local_position_timeout", 2.0);
    this->declare_parameter<double>("global_position_timeout", 10.0);
    this->declare_parameter<double>("battery_timeout", 2.0);
    this->declare_parameter<double>("manual_control_timeout", 0.0);
    this->declare_parameter<bool>("simulator", false);

    this->get_parameter("body_frame_id", body_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("aruco_map_frame_id", aruco_map_frame_id_);
    this->get_parameter("transform_timeout", transform_timeout_);
    this->get_parameter("offboard_timeout", offboard_timeout_);
    this->get_parameter("arming_timeout", arming_timeout_);
    this->get_parameter("default_speed", default_speed_);
    this->get_parameter("land_only_in_offboard", land_only_in_offboard_);
    this->get_parameter("check_kill_switch", check_kill_switch_);
    this->get_parameter("local_position_timeout", local_position_timeout_);
    this->get_parameter("global_position_timeout", global_position_timeout_);
    this->get_parameter("battery_timeout", battery_timeout_);
    this->get_parameter("manual_control_timeout", manual_control_timeout_);
    this->get_parameter("simulator", is_simulator_);

    // Load and parse reference_frames
    create_reference_frames();
    
    // --- State initialization ---
    current_local_position_.heading = 0.0;
    current_local_position_.x = 0.0;
    current_local_position_.y = 0.0;
    current_local_position_.z = 0.0;
    current_local_position_.vx = 0.0;
    current_local_position_.vy = 0.0;
    current_local_position_.vz = 0.0;
    current_local_position_.timestamp = 0.0;

    last_sended_pose = px4_msgs::msg::TrajectorySetpoint();
    service_done_ = false;
    control_rate_ = 30.0; // Hz
    lat_ = 0.0;
    lon_ = 0.0;
    alt_ = 0.0;
    battery_voltage = 0.0;
    battery_percentage  = 0.0;
    current_manual_control_timestamp_ = 0;
    current_battery_timestamp_ = 0;
    current_global_position_timestamp_ = 0;
    current_vehicle_status_timestamp_ = 0;
    
    // Create callback group for service to run in separate thread
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Initialize publishers, subscribers, services, and timer
    Init_publishers(px4_namespace);
    Init_subscribers(px4_namespace);
    Init_services();
    Init_timer();
    
    // Initialize TF frames
    init_tf_frames();
    
    // Publish static TF transforms
    publish_static_transforms();
    
    RCLCPP_INFO(this->get_logger(), "Offboard Control Node initialized with frame_id: %s", map_frame_id_.c_str());
}

OffboardControlNode::~OffboardControlNode()
{
    stopOffboardHeartbeat();
}

// --- Helpers ---
void OffboardControlNode::Init_SpeedController(
    const Pose& start_pose_ned, 
    const Pose& target_pose_any_frame,
    const std::string& frame_id,
    double speed) 
{
    // RAII buffer
    global_speed_controller_.reset();
    rclcpp::Logger logger = this->get_logger();
    try {
        // Resolve reference frame for target frame
        std::string reference_frame = get_reference_frame(frame_id);
        rclcpp::Time stamp = get_transform_lookup_time(frame_id, reference_frame);
        if (frame_id == aruco_map_frame_id_) {
            RCLCPP_INFO(logger, "Navigate in aruco_map frame (tf_static): using TF time 0 for lookup");
        }
        auto start_timestamp_micros = getTimestamp();

        // Check transform availability
        if (frame_id == aruco_map_frame_id_) {
            RCLCPP_INFO(logger, "Navigate in aruco_map frame: checking TF %s -> %s (need map->aruco_map from px4_local_pose_publisher)",
                        frame_id.c_str(), reference_frame.c_str());
        }
        if (!wait_for_transform(reference_frame, frame_id, stamp, 
                              rclcpp::Duration::from_seconds(transform_timeout_)) ||
            !wait_for_transform(map_frame_id_, reference_frame, stamp, 
                              rclcpp::Duration::from_seconds(transform_timeout_))) {
            if (frame_id == aruco_map_frame_id_) {
                RCLCPP_ERROR(logger, "Transform map <-> aruco_map not available. Check: 1) ArUco markers visible to camera, 2) px4_local_pose_publisher running and offset computed (see its logs for 'Offset computed').");
            }
            throw std::runtime_error("Transform not available between frames");
        }
        
        std::optional<builtin_interfaces::msg::Time> tf_lookup_override;
        if (frame_id == aruco_map_frame_id_) {
            builtin_interfaces::msg::Time t;
            t.sec = 0;
            t.nanosec = 0;
            tf_lookup_override = t;
        }
        global_speed_controller_ = std::make_unique<SpeedController>(
            logger, start_pose_ned, target_pose_any_frame, frame_id, map_frame_id_, speed,
            start_timestamp_micros, tf_buffer_, transform_timeout_, tf_lookup_override);
        
        if (global_speed_controller_->hasTransformationError()) {
            RCLCPP_ERROR(logger, "Initialization failed due to TF error.");
            global_speed_controller_.reset();
            // Publish target frame on error too (debug)
            publish_target_frame(target_pose_any_frame, frame_id, stamp);
        } else {
            RCLCPP_INFO(logger, "Global SpeedController initialized with frame_id: %s (reference: %s)", 
                      frame_id.c_str(), reference_frame.c_str());
            // Publish target frame
            publish_target_frame(target_pose_any_frame, frame_id, stamp);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "SpeedController initialization failed: %s", e.what());
        global_speed_controller_.reset();
    }
}

// !
void OffboardControlNode::checkManualControl() {
    if (is_simulator_ || true) return;
    if (manual_control_timeout_ > 0.0) {
        auto now = getTimestamp();
        double diff = (now - current_manual_control_timestamp_) / 1000000.0;
        if (diff > manual_control_timeout_) {
            throw std::runtime_error("Manual control timeout, RC is switched off?");
        }
    }
    if (check_kill_switch_) {
        const uint8_t SWITCH_POS_ON = 1;
        const int KILL_SWITCH_BIT = 12;
        uint8_t kill_switch = (current_manual_control_.buttons >> KILL_SWITCH_BIT) & 0b11;
        if (kill_switch == SWITCH_POS_ON) {
            stopOffboardHeartbeat();
            throw std::runtime_error("Kill switch is on");
        }
    }
}
// !

bool OffboardControlNode::offboardAndArm() {
    checkManualControl();
    // Send OffboardControlMode and setpoints
    for (int i = 0; i < 10; ++i) {
        publish_offboard_control_mode();
        publish_idle_setpoint();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    switch_to_offboard_mode();
    auto start = getTimestamp();
    while (rclcpp::ok()) {
        if (current_vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched to OFFBOARD mode.");
            break;
        }
        if ((getTimestamp() - start) / 1000000.0 > offboard_timeout_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to OFFBOARD mode: Timeout.");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    arm();
    start = getTimestamp();
    while (rclcpp::ok()) {
        if (current_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            RCLCPP_INFO(this->get_logger(), "Successfully armed.");
            return true;
        }
        if ((getTimestamp() - start) / 1000000.0 > arming_timeout_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm: Timeout.");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void OffboardControlNode::publish_setpoint() {
    publish_offboard_control_mode();
    Pose current_fcu_pose_ned(current_local_position_);
    const float nan_val = std::numeric_limits<float>::quiet_NaN();

    if (setpoint_type_ == NAVIGATE && global_speed_controller_) {
        if(global_speed_controller_->isEnd(current_fcu_pose_ned)){
            global_speed_controller_.release();
            setpoint_type_ = NONE;
            RCLCPP_INFO(this->get_logger(), "Navigation complite");
            return;
        }

        last_sended_pose = global_speed_controller_->getCurrentSetpointMSG_NED(getTimestamp());
        last_sended_pose.timestamp = getTimestamp();

        if (altitude_override_set_) {
            double ned_z;
            if (get_altitude_ned_from_frame(altitude_override_z_, altitude_override_frame_id_, ned_z)) {
                last_sended_pose.position[2] = static_cast<float>(ned_z);
            }
        }
        if (yaw_rate_override_set_) {
            last_sended_pose.yawspeed = static_cast<float>(yaw_rate_override_);
            last_sended_pose.yaw = nan_val;
        } else if (yaw_override_set_) {
            double ned_yaw;
            if (transform_yaw_to_ned(yaw_override_, yaw_override_frame_id_, ned_yaw)) {
                last_sended_pose.yaw = static_cast<float>(ned_yaw);
            }
            last_sended_pose.yawspeed = nan_val;
        }

        trajectory_setpoint_publisher_->publish(last_sended_pose);
        Pose setpoint_pose_ned(last_sended_pose.position[0], last_sended_pose.position[1], last_sended_pose.position[2], last_sended_pose.yaw);
        publish_setpoint_frame(setpoint_pose_ned, this->now());
    } else if (setpoint_type_ == POSITION) {
        geometry_msgs::msg::Point pt;
        pt.x = position_target_x_;
        pt.y = position_target_y_;
        pt.z = position_target_z_;
        geometry_msgs::msg::Point map_pt;
        if (!transform_point(pt, position_target_frame_id_, map_frame_id_, get_transform_lookup_time(position_target_frame_id_, map_frame_id_), map_pt)) {
            publish_idle_setpoint();
            return;
        }
        Pose target_enu(map_pt.x, map_pt.y, map_pt.z, position_target_yaw_);
        Pose target_ned = TransformPX4::enuToNedPose(target_enu);
        if (altitude_override_set_) {
            double ned_z;
            if (get_altitude_ned_from_frame(altitude_override_z_, altitude_override_frame_id_, ned_z)) {
                target_ned.z = ned_z;
            }
        }
        if (yaw_rate_override_set_) {
            last_sended_pose.position[0] = static_cast<float>(target_ned.x);
            last_sended_pose.position[1] = static_cast<float>(target_ned.y);
            last_sended_pose.position[2] = static_cast<float>(target_ned.z);
            last_sended_pose.yaw = nan_val;
            last_sended_pose.yawspeed = static_cast<float>(yaw_rate_override_);
        } else if (yaw_override_set_) {
            double ned_yaw;
            if (transform_yaw_to_ned(yaw_override_, yaw_override_frame_id_, ned_yaw)) {
                target_ned.yaw = ned_yaw;
            }
            last_sended_pose.position[0] = static_cast<float>(target_ned.x);
            last_sended_pose.position[1] = static_cast<float>(target_ned.y);
            last_sended_pose.position[2] = static_cast<float>(target_ned.z);
            last_sended_pose.yaw = static_cast<float>(target_ned.yaw);
            last_sended_pose.yawspeed = nan_val;
        } else {
            last_sended_pose.position[0] = static_cast<float>(target_ned.x);
            last_sended_pose.position[1] = static_cast<float>(target_ned.y);
            last_sended_pose.position[2] = static_cast<float>(target_ned.z);
            last_sended_pose.yaw = static_cast<float>(target_ned.yaw);
            last_sended_pose.yawspeed = nan_val;
        }
        last_sended_pose.timestamp = getTimestamp();
        trajectory_setpoint_publisher_->publish(last_sended_pose);
        publish_setpoint_frame(target_ned, this->now());
    } else if (setpoint_type_ == VELOCITY) {
        geometry_msgs::msg::Vector3 vel;
        vel.x = velocity_target_vx_;
        vel.y = velocity_target_vy_;
        vel.z = velocity_target_vz_;
        geometry_msgs::msg::Vector3 vel_enu;
        if (!transform_velocity(vel, velocity_target_frame_id_, map_frame_id_, get_transform_lookup_time(velocity_target_frame_id_, map_frame_id_), vel_enu)) {
            publish_idle_setpoint();
            return;
        }
        double ned_yaw;
        if (!transform_yaw_to_ned(velocity_target_yaw_, velocity_target_frame_id_, ned_yaw)) {
            ned_yaw = current_fcu_pose_ned.yaw;
        }
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.position[0] = msg.position[1] = msg.position[2] = nan_val;
        msg.velocity[0] = static_cast<float>(vel_enu.y);
        msg.velocity[1] = static_cast<float>(vel_enu.x);
        msg.velocity[2] = static_cast<float>(-vel_enu.z);
        msg.yaw = static_cast<float>(ned_yaw);
        msg.yawspeed = nan_val;
        trajectory_setpoint_publisher_->publish(msg);
    } else if (setpoint_type_ == ATTITUDE) {
        double ned_yaw;
        if (!transform_yaw_to_ned(static_cast<double>(attitude_yaw_), attitude_frame_id_, ned_yaw)) {
            ned_yaw = current_fcu_pose_ned.yaw;
        }
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(attitude_roll_, attitude_pitch_, ned_yaw);
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.q_d[0] = static_cast<float>(q_ned.w());
        msg.q_d[1] = static_cast<float>(q_ned.x());
        msg.q_d[2] = static_cast<float>(q_ned.y());
        msg.q_d[3] = static_cast<float>(q_ned.z());
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, attitude_thrust_));
        msg.yaw_sp_move_rate = std::numeric_limits<float>::quiet_NaN();
        vehicle_attitude_setpoint_publisher_->publish(msg);
    } else if (setpoint_type_ == RATES) {
        px4_msgs::msg::VehicleRatesSetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.roll = rates_roll_;
        msg.pitch = rates_pitch_;
        msg.yaw = rates_yaw_;
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, rates_thrust_));
        vehicle_rates_setpoint_publisher_->publish(msg);
    } else if (setpoint_type_ == FLIP) {
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        auto now = this->now();

        if (flip_phase_ == FlipPhase::CLIMB) {
            double elapsed = (now - flip_climb_start_time_).seconds();
            if (elapsed >= flip_climb_duration_) {
                flip_phase_ = FlipPhase::ROTATE;
                flip_rotate_start_time_ = now;
                flip_angle_done_ = 0.0;
            } else {
                px4_msgs::msg::TrajectorySetpoint msg{};
                msg.timestamp = getTimestamp();
                msg.position[0] = msg.position[1] = msg.position[2] = nan_val;
                msg.velocity[0] = 0.0f;
                msg.velocity[1] = 0.0f;
                msg.velocity[2] = static_cast<float>(-flip_vz_);
                msg.yaw = static_cast<float>(flip_initial_pose_ned_.yaw);
                msg.yawspeed = nan_val;
                trajectory_setpoint_publisher_->publish(msg);
            }
        }
        if (flip_phase_ == FlipPhase::ROTATE) {
            double dt = 1.0 / control_rate_;
            flip_angle_done_ += std::abs(flip_rate_) * dt;
            if (flip_angle_done_ >= flip_target_angle_) {
                flip_phase_ = FlipPhase::POSITION_HOLD;
            } else {
                px4_msgs::msg::VehicleRatesSetpoint msg{};
                msg.timestamp = getTimestamp();
                float rate = static_cast<float>(flip_rate_);
                msg.roll = (flip_axis_ == FlipAxis::ROLL) ? rate : 0.0f;
                msg.pitch = (flip_axis_ == FlipAxis::PITCH) ? rate : 0.0f;
                msg.yaw = (flip_axis_ == FlipAxis::YAW) ? rate : 0.0f;
                msg.thrust_body[0] = 0.0f;
                msg.thrust_body[1] = 0.0f;
                msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, flip_thrust_));
                vehicle_rates_setpoint_publisher_->publish(msg);
            }
        }
        if (flip_phase_ == FlipPhase::POSITION_HOLD) {
            last_sended_pose.position[0] = static_cast<float>(flip_initial_pose_ned_.x);
            last_sended_pose.position[1] = static_cast<float>(flip_initial_pose_ned_.y);
            last_sended_pose.position[2] = static_cast<float>(flip_initial_pose_ned_.z);
            last_sended_pose.yaw = static_cast<float>(flip_initial_pose_ned_.yaw);
            last_sended_pose.yawspeed = nan_val;
            last_sended_pose.timestamp = getTimestamp();
            trajectory_setpoint_publisher_->publish(last_sended_pose);
            setpoint_type_ = NONE;
            RCLCPP_INFO(this->get_logger(), "Flip complete, position hold set to initial pose");
        }
    } else if (setpoint_type_ == NONE) {
        publish_idle_setpoint();
    }
}

// !
void OffboardControlNode::startOffboardHeartbeat()
{
    // start send offboard
}
void OffboardControlNode::stopOffboardHeartbeat()
{
    // Stop send offboard
    setpoint_type_ = SetpointType::NONE;
    if (setpoint_timer_) {
        setpoint_timer_->cancel();
    }
    global_speed_controller_.reset();
}
// !

#pragma region TF2
void OffboardControlNode::clear_navigation_targets() {
    if (!target_transform_.child_frame_id.empty()) {
        target_transform_.header.stamp = this->now();
        target_transform_.header.frame_id = map_frame_id_;
        target_transform_.transform.translation.x = 0.0;
        target_transform_.transform.translation.y = 0.0;
        target_transform_.transform.translation.z = 0.0;
        tf2::Quaternion q_reset;
        q_reset.setRPY(0, 0, 0);
        target_transform_.transform.rotation = tf2::toMsg(q_reset);
        static_tf_broadcaster_.sendTransform(target_transform_);
    }
    
    if (!setpoint_transform_.child_frame_id.empty()) {
        setpoint_transform_.header.stamp = this->now();
        setpoint_transform_.header.frame_id = map_frame_id_;
        setpoint_transform_.transform.translation.x = 0.0;
        setpoint_transform_.transform.translation.y = 0.0;
        setpoint_transform_.transform.translation.z = 0.0;
        tf2::Quaternion q_reset2;
        q_reset2.setRPY(0, 0, 0);
        setpoint_transform_.transform.rotation = tf2::toMsg(q_reset2);
        tf_broadcaster_.sendTransform(setpoint_transform_);
    }
    
    RCLCPP_INFO(this->get_logger(), "Navigation targets cleared after landing");
}

void OffboardControlNode::create_reference_frames() {
    reference_frames_.clear();
    
    reference_frames_[body_frame_id_] = map_frame_id_;
    reference_frames_["body"] = map_frame_id_;
    reference_frames_["setpoint"] = map_frame_id_;
    reference_frames_["navigate_target"] = map_frame_id_;
    reference_frames_["aruco_map"] = map_frame_id_;
    reference_frames_["terrain"] = map_frame_id_;
    reference_frames_["setpoint"] = map_frame_id_;
    reference_frames_[map_frame_id_] = map_frame_id_;
}

std::string OffboardControlNode::get_reference_frame(const std::string& frame_id) {
    if (frame_id.empty()) return map_frame_id_;
    
    auto it = reference_frames_.find(frame_id);
    if (it != reference_frames_.end()) {
        return it->second;
    }
    return frame_id;
}

bool OffboardControlNode::validate_frame_id(const std::string& frame_id) {
    static std::unordered_set<std::string> valid_frames_cache;
   
    // Add frames from reference_frames
    for (const auto& pair : reference_frames_) {
        valid_frames_cache.insert(pair.first);
        valid_frames_cache.insert(pair.second);
    }
    
    // Check frame_id against allowed-frame cache
    if (valid_frames_cache.find(frame_id) != valid_frames_cache.end()) {
        return true;
    }
    
    // If not in cache, match pattern "aruco_*"
    const std::string aruco_prefix = "aruco_";
    if (frame_id.compare(0, aruco_prefix.length(), aruco_prefix) == 0) {
        std::string number_str = frame_id.substr(aruco_prefix.length());
        
        if (number_str.empty()) {
            return false;
        }
        
        // Fast check: all chars are digits
        for (char c : number_str) {
            if (!std::isdigit(static_cast<unsigned char>(c))) {
                return false;
            }
        }
        
        // Check numeric range
        // For values <1000, string length is enough
        if (number_str.length() > 4) {  // 1000 has 4 digits
            return false;
        }
        
        // Parse to int for stricter check
        try {
            int number = std::stoi(number_str);
            return (number >= 0 && number <= 1000);
        } catch (...) {
            return false;
        }
    }
    
    return false;
}

rclcpp::Time OffboardControlNode::get_transform_lookup_time() const {
    return is_simulator_ ? rclcpp::Time(0) : this->now();
}

rclcpp::Time OffboardControlNode::get_transform_lookup_time(const std::string& source_frame, const std::string& target_frame) const {
    if (source_frame == aruco_map_frame_id_ || target_frame == aruco_map_frame_id_) {
        return rclcpp::Time(0);
    }
    return get_transform_lookup_time();
}

bool OffboardControlNode::get_altitude_ned_from_frame(double z, const std::string& frame_id, double& out_ned_z) {
    geometry_msgs::msg::Point pt;
    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = z;
    geometry_msgs::msg::Point map_pt;
    if (!transform_point(pt, frame_id, map_frame_id_, get_transform_lookup_time(frame_id, map_frame_id_), map_pt)) {
        return false;
    }
    Eigen::Vector3d enu_vec(map_pt.x, map_pt.y, map_pt.z);
    Eigen::Vector3d ned_vec = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_vec);
    out_ned_z = ned_vec.z();
    return true;
}

bool OffboardControlNode::transform_yaw_to_ned(double yaw, const std::string& frame_id, double& out_ned_yaw) {
    double map_yaw_enu;
    if (frame_id == map_frame_id_ || frame_id == "map") {
        map_yaw_enu = yaw;
    } else {
        if (!transform_yaw(yaw, frame_id, map_frame_id_, get_transform_lookup_time(frame_id, map_frame_id_), map_yaw_enu)) {
            return false;
        }
    }
    out_ned_yaw = TransformPX4::enuToNedYaw(map_yaw_enu);
    return true;
}

void OffboardControlNode::init_tf_frames() {
    // Initialize TF frames
    body_transform_.child_frame_id = "body";
    target_transform_.child_frame_id = "navigate_target";
    setpoint_transform_.child_frame_id = "setpoint";
    terrain_transform_.child_frame_id = "terrain";
    terrain_transform_.header.frame_id = body_frame_id_;
}

bool OffboardControlNode::wait_for_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    const rclcpp::Duration& timeout) 
{
    rclcpp::Rate rate(100); // 100 Hz
    auto start_time = this->now();
    
    while (rclcpp::ok()) {
        if ((this->now() - start_time) > timeout) {
            RCLCPP_WARN(this->get_logger(), 
                "Transform timeout: %s -> %s after %.2f seconds",
                source_frame.c_str(), target_frame.c_str(), 
                timeout.seconds());
            return false;
        }
        
        try {
            if (tf_buffer_.canTransform(target_frame, source_frame, time)) {
                return true;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "Transform check exception: %s", ex.what());
        }
        
        rate.sleep();
    }
    return false;
}

bool OffboardControlNode::transform_point(
    const geometry_msgs::msg::Point& point,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    geometry_msgs::msg::Point& result)
{
    geometry_msgs::msg::PointStamped source_point, target_point;
    source_point.header.frame_id = source_frame;
    source_point.header.stamp = stamp;
    source_point.point = point;
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform point from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_point = tf_buffer_.transform(source_point, target_frame);
        result = target_point.point;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Point transform failed: %s", ex.what());
        return false;
    }
}

bool OffboardControlNode::transform_yaw(
    double yaw,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    double& result_yaw)
{
    geometry_msgs::msg::QuaternionStamped source_quat, target_quat;
    source_quat.header.frame_id = source_frame;
    source_quat.header.stamp = stamp;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    source_quat.quaternion = tf2::toMsg(q);
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform yaw from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_quat = tf_buffer_.transform(source_quat, target_frame);
        tf2::Quaternion q_out;
        tf2::fromMsg(target_quat.quaternion, q_out);
        result_yaw = tf2::impl::getYaw(q_out);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Yaw transform failed: %s", ex.what());
        return false;
    }
}

bool OffboardControlNode::transform_velocity(
    const geometry_msgs::msg::Vector3& velocity,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    geometry_msgs::msg::Vector3& result)
{
    geometry_msgs::msg::Vector3Stamped source_vel, target_vel;
    source_vel.header.frame_id = source_frame;
    source_vel.header.stamp = stamp;
    source_vel.vector = velocity;
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        // Fallback for body-referenced velocity commands: use current odometry yaw
        // to rotate body-frame XY velocity into map frame when TF is temporarily unavailable.
        const bool source_is_body = (source_frame == "body" || source_frame == body_frame_id_);
        const bool target_is_map = (target_frame == map_frame_id_ || target_frame == "map");
        if (source_is_body && target_is_map) {
            const double yaw_enu = TransformPX4::nedToEnuYaw(current_local_position_.heading);
            const double c = std::cos(yaw_enu);
            const double s = std::sin(yaw_enu);

            result.x = c * velocity.x - s * velocity.y;
            result.y = s * velocity.x + c * velocity.y;
            result.z = velocity.z;

            RCLCPP_WARN(this->get_logger(),
                "TF unavailable for %s -> %s, using odometry-yaw fallback for velocity transform",
                source_frame.c_str(), target_frame.c_str());
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform velocity from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_vel = tf_buffer_.transform(source_vel, target_frame);
        result = target_vel.vector;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Velocity transform failed: %s", ex.what());
        return false;
    }
}

void OffboardControlNode::publish_static_transforms() {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    
    // Static transform for navigation target frame
    geometry_msgs::msg::TransformStamped target_static;
    target_static.header.stamp = this->now();
    target_static.header.frame_id = map_frame_id_;
    target_static.child_frame_id = "navigate_target";
    target_static.transform.translation.x = 0.0;
    target_static.transform.translation.y = 0.0;
    target_static.transform.translation.z = 0.0;
    target_static.transform.rotation = tf2::toMsg(q);
    static_tf_broadcaster_.sendTransform(target_static);

    // Static transform for terrain frame
    geometry_msgs::msg::TransformStamped terrain_static;
    terrain_static.header.stamp = this->now();
    terrain_static.header.frame_id = body_frame_id_;
    terrain_static.child_frame_id = "terrain";
    terrain_static.transform.translation.x = 0.0;
    terrain_static.transform.translation.y = 0.0;
    terrain_static.transform.translation.z = 0.0;
    terrain_static.transform.rotation = tf2::toMsg(q);
    static_tf_broadcaster_.sendTransform(terrain_static);
    
    RCLCPP_INFO(this->get_logger(), "Published static transforms");
}

void OffboardControlNode::publish_terrain_frame(double altitude_above_ground) {
    if (terrain_transform_.child_frame_id.empty()) return;
    
    terrain_transform_.header.stamp = this->now();
    terrain_transform_.transform.translation.x = 0.0;
    terrain_transform_.transform.translation.y = 0.0;
    terrain_transform_.transform.translation.z = -altitude_above_ground;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    terrain_transform_.transform.rotation = tf2::toMsg(q);
    
    static_tf_broadcaster_.sendTransform(terrain_transform_);
}

void OffboardControlNode::publish_odometry_transform(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
{
    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
        RCLCPP_WARN(this->get_logger(), "Odometry frame not in NED, skipping transform");
        return;
    }

    auto timestamp = this->now();
    Eigen::Vector3d ned_pos(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d enu_pos = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_pos);
    
    Eigen::Quaterniond q_ned_to_body(
        msg->q[0],  // w
        msg->q[1],  // x
        msg->q[2],  // y
        msg->q[3]   // z
    );
    
    geometry_msgs::msg::TransformStamped body_transform;
    body_transform.header.stamp = timestamp;
    body_transform.header.frame_id = map_frame_id_;
    body_transform.child_frame_id = "body";
    
    body_transform.transform.translation.x = enu_pos.x();
    body_transform.transform.translation.y = enu_pos.y();
    body_transform.transform.translation.z = enu_pos.z();
    
    Eigen::Quaterniond q_enu_to_body = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned_to_body);
    double yaw_only = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_enu_to_body);
    
    tf2::Quaternion q_yaw_only;
    q_yaw_only.setRPY(0, 0, yaw_only);
    
    body_transform.transform.rotation = tf2::toMsg(q_yaw_only);
    tf_broadcaster_.sendTransform(body_transform);
    
    geometry_msgs::msg::TransformStamped base_link_transform;
    base_link_transform.header.stamp = timestamp;
    base_link_transform.header.frame_id = map_frame_id_;
    base_link_transform.child_frame_id = body_frame_id_;
    
    base_link_transform.transform.translation.x = enu_pos.x();
    base_link_transform.transform.translation.y = enu_pos.y();
    base_link_transform.transform.translation.z = enu_pos.z();
    
    Eigen::Quaterniond q_aircraft_to_ned = q_ned_to_body;
    Eigen::Quaterniond q_base_link_to_enu = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_aircraft_to_ned);
    Eigen::Quaterniond q_enu_to_base_link = q_base_link_to_enu;
    
    base_link_transform.transform.rotation.x = q_enu_to_base_link.x();
    base_link_transform.transform.rotation.y = q_enu_to_base_link.y();
    base_link_transform.transform.rotation.z = q_enu_to_base_link.z();
    base_link_transform.transform.rotation.w = q_enu_to_base_link.w();
    
    tf_broadcaster_.sendTransform(base_link_transform);
    current_odom_transform_ = base_link_transform;
}

void OffboardControlNode::publish_target_frame(
    const Pose& target_pose, 
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
    if (target_transform_.child_frame_id.empty()) return;
    
    try {
        // Resolve reference frame for target frame
        std::string reference_frame = get_reference_frame(frame_id);
        
        // Transform target position into reference frame
        geometry_msgs::msg::PointStamped target_point;
        target_point.header.frame_id = frame_id;
        target_point.header.stamp = stamp;
        target_point.point.x = target_pose.x;
        target_point.point.y = target_pose.y;
        target_point.point.z = target_pose.z;
        
        geometry_msgs::msg::PointStamped transformed_point = 
            tf_buffer_.transform(target_point, reference_frame);
        
        // Publish transform
        target_transform_.header.frame_id = reference_frame;
        target_transform_.header.stamp = stamp;
        target_transform_.transform.translation.x = transformed_point.point.x;
        target_transform_.transform.translation.y = transformed_point.point.y;
        target_transform_.transform.translation.z = transformed_point.point.z;
        
        // Orientation: use current yaw or transform
        tf2::Quaternion q;
        q.setRPY(0, 0, target_pose.yaw);
        target_transform_.transform.rotation = tf2::toMsg(q);
        
        static_tf_broadcaster_.sendTransform(target_transform_);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to publish target frame: %s", ex.what());
    }
}

void OffboardControlNode::publish_setpoint_frame(const Pose& setpoint_pose_ned, const rclcpp::Time& stamp) {
    if (setpoint_transform_.child_frame_id.empty()) return;
    
    setpoint_transform_.header.frame_id = map_frame_id_;
    setpoint_transform_.header.stamp = stamp;
    
    // NED to ENU for TF
    Pose setpoint_enu = TransformPX4::nedToEnuPose(setpoint_pose_ned);
    setpoint_transform_.transform.translation.x = setpoint_enu.x;
    setpoint_transform_.transform.translation.y = setpoint_enu.y;
    setpoint_transform_.transform.translation.z = setpoint_enu.z;
    
    // Do not publish NaN orientation — TF rejects it and breaks the tree
    double yaw_enu = setpoint_enu.yaw;
    if (!std::isfinite(yaw_enu)) {
        yaw_enu = TransformPX4::nedToEnuYaw(current_local_position_.heading);
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_enu);
    setpoint_transform_.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_.sendTransform(setpoint_transform_);
}
#pragma endregion TF2

#pragma region PX4 command
void OffboardControlNode::publish_idle_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    // Hold current position
    if (last_sended_pose.timestamp != 0){
        msg.position[0] = last_sended_pose.position[0];
        msg.position[1] = last_sended_pose.position[1];
        msg.position[2] = last_sended_pose.position[2];
        msg.yaw = last_sended_pose.yaw;
    }
    else{
        msg.position[0] = current_local_position_.x;
        msg.position[1] = current_local_position_.y;
        msg.position[2] = current_local_position_.z;
        msg.yaw = current_local_position_.heading;
    }
    msg.timestamp = getTimestamp();
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControlNode::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    bool is_flip = (setpoint_type_ == FLIP);
    msg.position = (setpoint_type_ == NAVIGATE || setpoint_type_ == POSITION || setpoint_type_ == NONE) ||
                   (is_flip && flip_phase_ == FlipPhase::POSITION_HOLD);
    msg.velocity = (setpoint_type_ == VELOCITY) || (is_flip && flip_phase_ == FlipPhase::CLIMB);
    msg.acceleration = false;
    msg.attitude = (setpoint_type_ == ATTITUDE);
    msg.body_rate = (setpoint_type_ == RATES) || (is_flip && flip_phase_ == FlipPhase::ROTATE);
    msg.timestamp = getTimestamp();
    offboard_mode_publisher_->publish(msg);
}

void OffboardControlNode::switch_to_offboard_mode(){
    RCLCPP_INFO(this->get_logger(), "Requesting switch to Offboard mode");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControlNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = getTimestamp();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    request->request = msg;
    service_done_ = false;
    auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControlNode::response_callback, this,
                           std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Sent vehicle command %d", msg.command);
}

bool OffboardControlNode::land() {
    stopOffboardHeartbeat();
    RCLCPP_INFO(this->get_logger(), "Sending land command...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    return true;
}

void OffboardControlNode::arm()
{
    RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void OffboardControlNode::disarm()
{
    RCLCPP_INFO(this->get_logger(), "DisArming vehicle...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}
#pragma endregion PX4 command

#pragma region Service
void OffboardControlNode::navigate_callback(const offboard_interfaces::srv::Navigate::Request::SharedPtr request,
                           const offboard_interfaces::srv::Navigate::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received navigate request to frame: %s", 
               request->frame_id.c_str());
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        RCLCPP_WARN(this->get_logger(), "Navigate failed: %s", response->message.c_str());
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        // Frame check
        if (!validate_frame_id(request->frame_id)) {
            throw std::runtime_error("Invalid frame_id: " + request->frame_id);
        }
        // Checks
        if (std::isinf(request->x) || std::isinf(request->y) || std::isinf(request->z)) {
            throw std::runtime_error("x, y, z arguments cannot be Inf");
        }
        if (std::isinf(request->speed)) {
            throw std::runtime_error("speed argument cannot be Inf");
        }
        if (std::isfinite(request->x) != std::isfinite(request->y)) {
            throw std::runtime_error("x and y can be set only together");
        }
        if (request->speed < 0) {
            throw std::runtime_error("Navigate speed must be positive");
        }
        if (std::isnan(request->x) && std::isnan(request->y) && std::isnan(request->z)) {
            throw std::runtime_error("At least one of x, y, z must be finite");
        }
        Pose current_fcu_pose_ned(current_local_position_);
        Pose user_target_enu(request->x, request->y, request->z, request->yaw);
        std::string effective_frame_id = request->frame_id;
        Pose effective_target = user_target_enu;

        // Body frame: compute target in map from the same pose snapshot as start to avoid
        // yaw mismatch (TF body orientation vs current_local_position_.heading race).
        // Target yaw in map = current heading (ENU) + user yaw (relative to body).
        if (request->frame_id == "body") {
            geometry_msgs::msg::Point body_point;
            body_point.x = request->x;
            body_point.y = request->y;
            body_point.z = request->z;
            geometry_msgs::msg::Point map_point;
            if (!transform_point(body_point, "body", map_frame_id_, get_transform_lookup_time(), map_point)) {
                throw std::runtime_error("Failed to transform body target to map frame");
            }
            double current_heading_enu = TransformPX4::nedToEnuYaw(current_fcu_pose_ned.yaw);
            effective_target = Pose(map_point.x, map_point.y, map_point.z, current_heading_enu + request->yaw);
            effective_frame_id = map_frame_id_;
        }

        Init_SpeedController(current_fcu_pose_ned, effective_target, effective_frame_id, request->speed);
        RCLCPP_INFO(this->get_logger(), "Starting from FCU NED (%.2f, %.2f, %.2f, yaw=%.2f) to User target in %s frame (%.2f, %.2f, %.2f, yaw=%.2f) at pos_speed %.2f m/s with instant yaw alignment.",
                current_fcu_pose_ned.x, current_fcu_pose_ned.y, current_fcu_pose_ned.z, current_fcu_pose_ned.yaw,
                effective_frame_id.c_str(),
                effective_target.x, effective_target.y, effective_target.z, effective_target.yaw,
                request->speed);
        
        setpoint_type_ = NAVIGATE;
        last_sended_pose = px4_msgs::msg::TrajectorySetpoint();
        Init_timer();
        wait_armed_ = request->auto_arm;
        if (request->auto_arm) {
            if (!offboardAndArm()) {
                throw std::runtime_error("Auto-arm/offboard failed");
            }
            wait_armed_ = false;
        } else {
            if (current_vehicle_status_.nav_state != 
                px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
            }
            if (current_vehicle_status_.arming_state != 
                px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                throw std::runtime_error("Copter is not armed, use auto_arm?");
            }
        }
        response->success = true;
        response->message = "Navigate started";
    } catch (const std::exception& e) {
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Navigate failed: %s", e.what());
        if (setpoint_timer_) {
            setpoint_timer_->cancel();
        }
        setpoint_type_ = NONE;
        global_speed_controller_.reset();
    }
    busy_ = false;
}

void OffboardControlNode::land_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    const std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  
    RCLCPP_INFO(this->get_logger(), "Received land request.");
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        RCLCPP_WARN(this->get_logger(), "Land failed: %s", response->message.c_str());
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        if (land_only_in_offboard_) {
            if (current_vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                throw std::runtime_error("Copter is not in OFFBOARD mode");
            }
        }
        if (!land()) {
            throw std::runtime_error("Failed to initiate land command");
        }
        setpoint_type_ = NONE;
        if (setpoint_timer_) {
            setpoint_timer_->cancel();
        }
        global_speed_controller_.reset();

        clear_navigation_targets();
        response->success = true;
        response->message = "Land command sent";
    } catch (const std::exception& e) {
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Land failed: %s", e.what());
    }
    busy_ = false;
}

void OffboardControlNode::get_telemetry_callback(const offboard_interfaces::srv::GetTelemetry::Request::SharedPtr request,
                            const offboard_interfaces::srv::GetTelemetry::Response::SharedPtr response) {
    auto now = getTimestamp();

    // Choose frame for telemetry output
    std::string telemetry_frame = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
    if (!validate_frame_id(telemetry_frame)) {
        RCLCPP_WARN(this->get_logger(),
                    "GetTelemetry: invalid frame_id '%s', falling back to '%s'",
                    telemetry_frame.c_str(), map_frame_id_.c_str());
        telemetry_frame = map_frame_id_;
    }

    // State checks
    if (current_vehicle_status_timestamp_ == 0) {
        response->connected = false;
    } else {
        response->connected = true;
        response->armed = (current_vehicle_status_.arming_state == 
                            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        switch (current_vehicle_status_.nav_state) {
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                response->mode = "OFFBOARD"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                response->mode = "AUTO.LAND"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                response->mode = "AUTO.LOITER"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
                response->mode = "AUTO.MISSION"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
                response->mode = "AUTO.RTL"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                response->mode = "AUTO.TAKEOFF"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
                response->mode = "MANUAL"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:
                response->mode = "ACRO"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
                response->mode = "ALTCTL"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
                response->mode = "POSCTL"; break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:
                response->mode = "STABILIZE"; break;
            default:
                response->mode = "UNKNOWN"; break;
        }
    }

    const bool local_pose_fresh =
        (now - current_local_position_.timestamp) / 1000000.0 <= local_position_timeout_;

    if (local_pose_fresh) {
        // Body pose in requested telemetry_frame
        geometry_msgs::msg::Point body_origin;
        body_origin.x = 0.0;
        body_origin.y = 0.0;
        body_origin.z = 0.0;

        geometry_msgs::msg::Point pose_in_frame;
        rclcpp::Time tf_time = get_transform_lookup_time("body", telemetry_frame);

        if (transform_point(body_origin, "body", telemetry_frame, tf_time, pose_in_frame)) {
            response->x = static_cast<float>(pose_in_frame.x);
            response->y = static_cast<float>(pose_in_frame.y);
            response->z = static_cast<float>(pose_in_frame.z);
        } else {
            // If transform unavailable, return NaN position
            response->x = std::numeric_limits<float>::quiet_NaN();
            response->y = std::numeric_limits<float>::quiet_NaN();
            response->z = std::numeric_limits<float>::quiet_NaN();
        }

        // Drone yaw: body +X orientation in requested frame
        double yaw_in_frame = 0.0;
        if (transform_yaw(0.0, "body", telemetry_frame, tf_time, yaw_in_frame)) {
            response->yaw = static_cast<float>(yaw_in_frame);
        } else {
            response->yaw = std::numeric_limits<float>::quiet_NaN();
        }

        // Roll/pitch still not computed — zeros
        response->roll = 0.0f;
        response->pitch = 0.0f;
    } else {
        response->x = std::numeric_limits<float>::quiet_NaN();
        response->y = std::numeric_limits<float>::quiet_NaN();
        response->z = std::numeric_limits<float>::quiet_NaN();
        response->roll = std::numeric_limits<float>::quiet_NaN();
        response->pitch = std::numeric_limits<float>::quiet_NaN();
        response->yaw = std::numeric_limits<float>::quiet_NaN();
    }

    // Velocity in requested frame (TF map -> telemetry_frame)
    if (local_pose_fresh) {
        CurrentPose enu_vel = TransformPX4::nedToEnuVel(current_local_position_);
        geometry_msgs::msg::Vector3 vel_map;
        vel_map.x = enu_vel.vx;
        vel_map.y = enu_vel.vy;
        vel_map.z = enu_vel.vz;

        geometry_msgs::msg::Vector3 vel_in_frame;
        rclcpp::Time tf_time = get_transform_lookup_time(map_frame_id_, telemetry_frame);
        if (transform_velocity(vel_map, map_frame_id_, telemetry_frame, tf_time, vel_in_frame)) {
            response->vx = static_cast<float>(vel_in_frame.x);
            response->vy = static_cast<float>(vel_in_frame.y);
            response->vz = static_cast<float>(vel_in_frame.z);
        } else {
            response->vx = std::numeric_limits<float>::quiet_NaN();
            response->vy = std::numeric_limits<float>::quiet_NaN();
            response->vz = std::numeric_limits<float>::quiet_NaN();
        }
    } else {
        response->vx = std::numeric_limits<float>::quiet_NaN();
        response->vy = std::numeric_limits<float>::quiet_NaN();
        response->vz = std::numeric_limits<float>::quiet_NaN();
    }
    // Global position
    if ((now - current_global_position_timestamp_) / 1000000.0 <= global_position_timeout_) {
        response->lat = lat_;
        response->lon = lon_;
        response->alt = alt_;
    } else {
        response->lat = std::numeric_limits<float>::quiet_NaN();
        response->lon = std::numeric_limits<float>::quiet_NaN();
        response->alt = std::numeric_limits<float>::quiet_NaN();
    }
    // Battery
    if ((now - current_battery_timestamp_) / 1000000.0 <= battery_timeout_) {
        response->voltage = battery_voltage;
        response->cell_voltage = battery_voltage;
    } else {
        response->voltage = std::numeric_limits<float>::quiet_NaN();
        response->cell_voltage = std::numeric_limits<float>::quiet_NaN();
    }

    // Actual frame for x/y/z/vx/vy/vz/yaw
    response->frame_id = telemetry_frame;
}

void OffboardControlNode::set_altitude_callback(const offboard_interfaces::srv::SetAltitude::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetAltitude::Response::SharedPtr response) {
    try {
        if (!validate_frame_id(request->frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + request->frame_id;
            return;
        }
        altitude_override_z_ = static_cast<double>(request->z);
        altitude_override_frame_id_ = request->frame_id;
        altitude_override_set_ = true;
        response->success = true;
        response->message = "Altitude override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_yaw_callback(const offboard_interfaces::srv::SetYaw::Request::SharedPtr request,
                                           const offboard_interfaces::srv::SetYaw::Response::SharedPtr response) {
    try {
        if (std::isnan(request->yaw)) {
            yaw_override_set_ = false;
            yaw_rate_override_set_ = false;
            response->success = true;
            response->message = "Yaw and yaw rate override cleared";
            return;
        }
        if (!validate_frame_id(request->frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + request->frame_id;
            return;
        }
        yaw_override_ = static_cast<double>(request->yaw);
        yaw_override_frame_id_ = request->frame_id;
        yaw_override_set_ = true;
        yaw_rate_override_set_ = false;
        response->success = true;
        response->message = "Yaw override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_yaw_rate_callback(const offboard_interfaces::srv::SetYawRate::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetYawRate::Response::SharedPtr response) {
    try {
        yaw_rate_override_ = static_cast<double>(request->yaw_rate);
        yaw_rate_override_set_ = true;
        yaw_override_set_ = false;
        response->success = true;
        response->message = "Yaw rate override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_position_callback(const offboard_interfaces::srv::SetPosition::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetPosition::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        position_target_x_ = std::isfinite(request->x) ? request->x : 0.0;
        position_target_y_ = std::isfinite(request->y) ? request->y : 0.0;
        position_target_z_ = std::isfinite(request->z) ? request->z : 0.0;
        position_target_yaw_ = std::isfinite(request->yaw) ? request->yaw : 0.0;
        position_target_frame_id_ = frame_id;
        setpoint_type_ = POSITION;
        Init_timer();
        response->success = true;
        response->message = "Set position started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_velocity_callback(const offboard_interfaces::srv::SetVelocity::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetVelocity::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        velocity_target_vx_ = request->vx;
        velocity_target_vy_ = request->vy;
        velocity_target_vz_ = request->vz;
        velocity_target_yaw_ = std::isfinite(request->yaw) ? request->yaw : 0.0;
        velocity_target_frame_id_ = frame_id;
        setpoint_type_ = VELOCITY;
        Init_timer();
        response->success = true;
        response->message = "Set velocity started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_attitude_callback(const offboard_interfaces::srv::SetAttitude::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetAttitude::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        attitude_roll_ = request->roll;
        attitude_pitch_ = request->pitch;
        attitude_yaw_ = request->yaw;
        attitude_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
        attitude_frame_id_ = frame_id;
        setpoint_type_ = ATTITUDE;
        Init_timer();
        response->success = true;
        response->message = "Set attitude started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_rates_callback(const offboard_interfaces::srv::SetRates::Request::SharedPtr request,
                                             const offboard_interfaces::srv::SetRates::Response::SharedPtr response) {
    try {
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        rates_roll_ = request->roll_rate;
        rates_pitch_ = request->pitch_rate;
        rates_yaw_ = request->yaw_rate;
        rates_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
        setpoint_type_ = RATES;
        Init_timer();
        response->success = true;
        response->message = "Set rates started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::flip_callback(const offboard_interfaces::srv::Flip::Request::SharedPtr request,
                                        const offboard_interfaces::srv::Flip::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received flip request, axis: %s", request->axis.c_str());
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        if (request->axis == "roll") {
            flip_axis_ = FlipAxis::ROLL;
        } else if (request->axis == "pitch") {
            flip_axis_ = FlipAxis::PITCH;
        } else if (request->axis == "yaw") {
            flip_axis_ = FlipAxis::YAW;
        } else {
            throw std::runtime_error("axis must be 'roll', 'pitch' or 'yaw', got: " + request->axis);
        }
        if (request->climb_duration < 0.0f || request->rate == 0.0f || request->target_angle <= 0.0f) {
            throw std::runtime_error("climb_duration must be >= 0, rate != 0, target_angle > 0");
        }
        if (request->auto_arm && !offboardAndArm()) {
            throw std::runtime_error("Auto-arm/offboard failed");
        }
        if (!request->auto_arm) {
            if (current_vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
            }
            if (current_vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                throw std::runtime_error("Copter is not armed, use auto_arm?");
            }
        }
        flip_initial_pose_ned_ = Pose(current_local_position_);
        flip_vz_ = request->vz;
        flip_climb_duration_ = request->climb_duration;
        flip_rate_ = request->rate;
        flip_target_angle_ = static_cast<double>(request->target_angle);
        flip_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
        flip_phase_ = FlipPhase::CLIMB;
        flip_climb_start_time_ = this->now();
        flip_angle_done_ = 0.0;
        setpoint_type_ = FLIP;
        Init_timer();
        response->success = true;
        response->message = "Flip started";
        RCLCPP_INFO(this->get_logger(), "Flip: initial NED (%.2f, %.2f, %.2f), axis=%s, rate=%.2f, target=%.2f rad",
                    flip_initial_pose_ned_.x, flip_initial_pose_ned_.y, flip_initial_pose_ned_.z,
                    request->axis.c_str(), flip_rate_, flip_target_angle_);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Flip failed: %s", e.what());
    }
    busy_ = false;
}

#pragma endregion Service

#pragma region Inits
void OffboardControlNode::Init_publishers(std::string px4_namespace)
{
    vehicle_command_client_ = create_client<px4_msgs::srv::VehicleCommand>(
        px4_namespace+"vehicle_command"
    );
    while (!vehicle_command_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    offboard_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_namespace + "in/offboard_control_mode", QoSConfigurator::commands());
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_namespace + "in/trajectory_setpoint", QoSConfigurator::commands());
    vehicle_attitude_setpoint_publisher_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        px4_namespace + "in/vehicle_attitude_setpoint", QoSConfigurator::commands());
    vehicle_rates_setpoint_publisher_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        px4_namespace + "in/vehicle_rates_setpoint", QoSConfigurator::commands());
}

void OffboardControlNode::Init_subscribers(std::string px4_namespace)
{
    if (is_simulator_){
        vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            px4_namespace + "out/vehicle_status_v1",
            QoSConfigurator::system_status(),
            std::bind(&OffboardControlNode::vehicleStatusCallback, this, std::placeholders::_1));
    }
    else{
        vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            px4_namespace + "out/vehicle_status",
            QoSConfigurator::system_status(),
            std::bind(&OffboardControlNode::vehicleStatusCallback, this, std::placeholders::_1));
    }
    vehicle_global_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        px4_namespace + "out/vehicle_global_position",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::vehicleGlobalPosCallback, this, std::placeholders::_1));
    vehicle_odom_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        px4_namespace + "out/vehicle_odometry",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::vehicleOdomCallback, this, std::placeholders::_1));
    battery_status_subscriber_ = create_subscription<px4_msgs::msg::BatteryStatus>(
        px4_namespace + "out/battery_status",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::batteryCallback, this, std::placeholders::_1));
    manual_control_subscriber_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        px4_namespace + "out/manual_control_setpoint",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::manualControlCallback, this, std::placeholders::_1));
}

void OffboardControlNode::Init_services()
{
    navigate_service_ = create_service<offboard_interfaces::srv::Navigate>(
        "navigate",
        std::bind(&OffboardControlNode::navigate_callback, this, 
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    land_service_ = create_service<std_srvs::srv::Trigger>(
        "land",
        std::bind(&OffboardControlNode::land_callback, this,
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    get_telemetry_service_ = create_service<offboard_interfaces::srv::GetTelemetry>(
        "get_telemetry",
        std::bind(&OffboardControlNode::get_telemetry_callback, this,
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    set_altitude_service_ = create_service<offboard_interfaces::srv::SetAltitude>(
        "set_altitude",
        std::bind(&OffboardControlNode::set_altitude_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_yaw_service_ = create_service<offboard_interfaces::srv::SetYaw>(
        "set_yaw",
        std::bind(&OffboardControlNode::set_yaw_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_yaw_rate_service_ = create_service<offboard_interfaces::srv::SetYawRate>(
        "set_yaw_rate",
        std::bind(&OffboardControlNode::set_yaw_rate_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_position_service_ = create_service<offboard_interfaces::srv::SetPosition>(
        "set_position",
        std::bind(&OffboardControlNode::set_position_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_velocity_service_ = create_service<offboard_interfaces::srv::SetVelocity>(
        "set_velocity",
        std::bind(&OffboardControlNode::set_velocity_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_attitude_service_ = create_service<offboard_interfaces::srv::SetAttitude>(
        "set_attitude",
        std::bind(&OffboardControlNode::set_attitude_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_rates_service_ = create_service<offboard_interfaces::srv::SetRates>(
        "set_rates",
        std::bind(&OffboardControlNode::set_rates_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    flip_service_ = create_service<offboard_interfaces::srv::Flip>(
        "flip",
        std::bind(&OffboardControlNode::flip_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
}

void OffboardControlNode::Init_timer()
{
    if (setpoint_timer_ && !setpoint_timer_->is_canceled()) {
        setpoint_timer_->cancel();
    }
    const std::chrono::nanoseconds timer_period{static_cast<int64_t>(1e9/control_rate_)};
    setpoint_timer_ = create_wall_timer(
        timer_period,
        std::bind(&OffboardControlNode::publish_setpoint, this));
}
#pragma endregion Inits

#pragma region Callback
void OffboardControlNode::batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg){  
    battery_percentage = msg->remaining * 100.0;
    battery_voltage = msg->voltage_v;
    current_battery_timestamp_ = getTimestamp();
}

void OffboardControlNode::manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg){
    current_manual_control_ = *msg;
    current_manual_control_timestamp_ = getTimestamp();
}

void OffboardControlNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    current_vehicle_status_ = *msg;
    current_vehicle_status_timestamp_ = getTimestamp();
    if (!prev_vehicle_status_msg_) {
        RCLCPP_INFO(this->get_logger(), "Initial vehicle status received");
    } else {
        // Check arming state changes
        if (prev_vehicle_status_msg_->arming_state != msg->arming_state) {
            if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed");
            } else if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
            }
        }
        // Check navigation state changes
        if (prev_vehicle_status_msg_->nav_state != msg->nav_state) {
            switch (msg->nav_state) {
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in takeoff mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in land mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in hold mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in offboard mode");
                    break;
            }
        }
    }
    prev_vehicle_status_msg_ = msg;
}

void OffboardControlNode::vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt;
    current_global_position_timestamp_ = getTimestamp();
}

double OffboardControlNode::normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

void OffboardControlNode::vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_local_position_.x = msg->position[0];
    current_local_position_.y = msg->position[1];
    current_local_position_.z = msg->position[2];
    
    Eigen::Quaterniond q_ned_to_body(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    double raw_heading_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned_to_body);
    if (is_simulator_ && !initial_heading_set_) {
        initial_heading_ned_ = raw_heading_ned;
        initial_heading_set_ = true;
    }
    current_local_position_.heading = raw_heading_ned;
    
    current_local_position_.vx = msg->velocity[0];
    current_local_position_.vy = msg->velocity[1];
    current_local_position_.vz = msg->velocity[2];
    current_local_position_.timestamp = getTimestamp();

    publish_odometry_transform(msg);
    publish_terrain_frame(-msg->position[2]);

    double log_yaw = raw_heading_ned;
    if (is_simulator_ && initial_heading_set_) {
        log_yaw = normalizeYaw(raw_heading_ned - initial_heading_ned_);
    }
    RCLCPP_DEBUG(this->get_logger(), "Drone pose in NED (%.2f, %.2f, %.2f, yaw=%.2f)", current_local_position_.x, current_local_position_.y, current_local_position_.z, log_yaw);
}
#pragma endregion Callback

#pragma region Utils
uint64_t OffboardControlNode::getTimestamp() {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

void OffboardControlNode::response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto reply = future.get()->reply;
        service_result_ = reply.result;
        switch (service_result_)
        {
        case reply.VEHICLE_CMD_RESULT_ACCEPTED:
            RCLCPP_INFO(this->get_logger(), "command accepted");
            break;
        case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
            break;
        case reply.VEHICLE_CMD_RESULT_DENIED:
            RCLCPP_WARN(this->get_logger(), "command denied");
            break;
        case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
            RCLCPP_WARN(this->get_logger(), "command unsupported");
            break;
        case reply.VEHICLE_CMD_RESULT_FAILED:
            RCLCPP_WARN(this->get_logger(), "command failed");
            break;
        case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
            RCLCPP_WARN(this->get_logger(), "command in progress");
            break;
        case reply.VEHICLE_CMD_RESULT_CANCELLED:
            RCLCPP_WARN(this->get_logger(), "command cancelled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "command reply unknown");
            break;
        }
        service_done_ = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}
#pragma endregion Utils

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlNode>("/fmu/");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}