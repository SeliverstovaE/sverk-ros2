#ifndef OFFBOARD_CONTROL_HPP_
#define OFFBOARD_CONTROL_HPP_
#include "rclcpp/rclcpp.hpp"
#include "offboard_interfaces/srv/navigate.hpp"
#include "offboard_interfaces/srv/get_telemetry.hpp"
#include "offboard_interfaces/srv/set_altitude.hpp"
#include "offboard_interfaces/srv/set_yaw.hpp"
#include "offboard_interfaces/srv/set_yaw_rate.hpp"
#include "offboard_interfaces/srv/set_position.hpp"
#include "offboard_interfaces/srv/set_velocity.hpp"
#include "offboard_interfaces/srv/set_attitude.hpp"
#include "offboard_interfaces/srv/set_rates.hpp"
#include "offboard_interfaces/srv/flip.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <map>
#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <vector>


using namespace std::chrono_literals;
using namespace px4_ros_com::frame_transforms;

class QoSConfigurator {
public:
    static rclcpp::QoS sensor_data() {
        return rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
    }
    static rclcpp::QoS system_status() {
        return rclcpp::QoS(rclcpp::KeepLast(5))
            .best_effort()
            .transient_local();
    }
    static rclcpp::QoS commands() {
        return rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
    }
};

struct CurrentPose {
    double x, y, z;
    double vx, vy, vz;
    double heading;
    uint64_t timestamp;
};

struct Pose {
    double x, y, z;
    double yaw;
    Pose(double x = 0.0, double y = 0.0, double z = 0.0, double yaw = 0.0)
        : x(x), y(y), z(z), yaw(yaw) {}
    Pose(CurrentPose pose)
        : x(pose.x), y(pose.y), z(pose.z), yaw(pose.heading) {}
};

enum SetpointType {
    NONE,
    NAVIGATE,
    NAVIGATE_GLOBAL,
    POSITION,
    VELOCITY,
    ATTITUDE,
    RATES,
    FLIP,
    ALTITUDE,
    YAW,
    YAW_RATE,
};

enum class FlipAxis { ROLL, PITCH, YAW };

enum class FlipPhase { CLIMB, ROTATE, POSITION_HOLD };

class TransformPX4 {
public:
    using QuaternionArray = std::array<float, 4>;
    
    static double interpolateYaw(double start_yaw, double end_yaw, double ratio) {
        double diff = std::atan2(std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));
        double interpolated_diff = diff * ratio;
        double result_yaw = start_yaw + interpolated_diff;
        return std::atan2(std::sin(result_yaw), std::cos(result_yaw));
    }

    static double getDistance(const Pose& from, const Pose& to) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double dz = to.z - from.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static Pose enuToNedPose(const Pose& enu_pose) {
        Eigen::Vector3d enu_vec(enu_pose.x, enu_pose.y, enu_pose.z);
        Eigen::Vector3d ned_vec = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_vec);
        return Pose(ned_vec.x(), ned_vec.y(), ned_vec.z(), enuToNedYaw(enu_pose.yaw));
    }

    static Pose nedToEnuPose(const Pose& ned_pose) {
        Eigen::Vector3d ned_vec(ned_pose.x, ned_pose.y, ned_pose.z);
        Eigen::Vector3d enu_vec = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_vec);
        return Pose(enu_vec.x(), enu_vec.y(), enu_vec.z(), nedToEnuYaw(ned_pose.yaw));
    }

    static double nedToEnuYaw(const double& ned_yaw) {
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(0.0, 0.0, ned_yaw);
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        return px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_enu);
    }

    static QuaternionArray nedToEnuQuan(const QuaternionArray& ned_quan) {
        // Array to Eigen quaternion
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(ned_quan);
        
        // NED to ENU quaternion
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned);
        
        // Back to array
        QuaternionArray enu_quan;
        px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(q_enu, enu_quan);
        
        return enu_quan;
    }

    static QuaternionArray enuToNedQuan(const QuaternionArray& enu_quan) {
        // Array to Eigen quaternion
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(enu_quan);
        
        // ENU to NED quaternion
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        
        // Back to array
        QuaternionArray ned_quan;
        px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(q_ned, ned_quan);
        
        return ned_quan;
    }

    static double enuToNedYaw(const double& enu_yaw) {
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(0.0, 0.0, enu_yaw);
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        return px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned);
    }

    static CurrentPose enuToNedVel(const CurrentPose& enu_vel) {
        CurrentPose ned_vel;
        ned_vel.timestamp = enu_vel.timestamp;
        ned_vel.heading = enu_vel.heading;
        ned_vel.x = enu_vel.x;
        ned_vel.y = enu_vel.y;
        ned_vel.z = enu_vel.z;
        ned_vel.vx = enu_vel.vy;
        ned_vel.vy = enu_vel.vx;
        ned_vel.vz = -enu_vel.vz;
        return ned_vel;
    }

    static CurrentPose nedToEnuVel(const CurrentPose& ned_vel) {
        CurrentPose enu_vel;
        enu_vel.timestamp = ned_vel.timestamp;
        enu_vel.heading = ned_vel.heading;
        enu_vel.x = ned_vel.x;
        enu_vel.y = ned_vel.y;
        enu_vel.z = ned_vel.z;
        enu_vel.vx = ned_vel.vy;
        enu_vel.vy = ned_vel.vx;
        enu_vel.vz = -ned_vel.vz;
        return enu_vel;
    }
};

class SpeedController {
public:
    SpeedController(
        rclcpp::Logger logger,
        const Pose& start_pose_ned,
        const Pose& target_pose_any_frame,
        const std::string& frame_id,
        const std::string& map_frame_id,
        double speed,
        long long start_timestamp_micros,
        tf2_ros::Buffer& tf_buffer,
        double transform_timeout,
        std::optional<builtin_interfaces::msg::Time> tf_lookup_time_override = std::nullopt)
        : logger_(logger),
          start_pose_ned_(start_pose_ned),
          target_speed_(speed),
          start_timestamp_micros_(start_timestamp_micros),
          tf_buffer_(&tf_buffer),
          transform_timeout_(transform_timeout),
          current_frame_id_(frame_id),
          map_frame_id_(map_frame_id),
          tf_lookup_time_override_(tf_lookup_time_override)
    {
        // Transform target pose into map frame
        Pose target_pose_in_map_frame;
        if (!transform_to_map_frame(target_pose_any_frame, frame_id, target_pose_in_map_frame)) {
            RCLCPP_ERROR(logger_, "Failed to transform target pose from frame %s to map frame", frame_id.c_str());
            transformation_error_ = true;
            return;
        }
        // ENU to NED for internal math
        target_pose_ned_ = TransformPX4::enuToNedPose(target_pose_in_map_frame);
        distance_ = TransformPX4::getDistance(start_pose_ned_, target_pose_ned_);
        double signed_yaw_diff = std::atan2(std::sin(target_pose_ned_.yaw - start_pose_ned_.yaw),
                                           std::cos(target_pose_ned_.yaw - start_pose_ned_.yaw));
        yaw_difference_ = std::abs(signed_yaw_diff);
        if (distance_ == 0.0) {
            total_pos_time_micros_ = 0;
            RCLCPP_INFO(logger_, "SpeedController: Start and target positions are the same. Position time is 0 microseconds.");
        } else if (speed <= 0.0) {
            total_pos_time_micros_ = LLONG_MAX;
            RCLCPP_ERROR(logger_, "SpeedController: Target speed must be positive. Got %f.", speed);
        } else {
            double total_pos_time_seconds = distance_ / speed;
            total_pos_time_micros_ = static_cast<long long>(total_pos_time_seconds * 1e6);
        }
        // Navigate: yaw applied immediately (no angular ramp).
        total_yaw_time_micros_ = 0;
    }

    bool transform_to_map_frame(
        const Pose& source_pose,
        const std::string& source_frame,
        Pose& target_pose) 
    {
        if (source_frame == map_frame_id_) {
            target_pose = source_pose;
            return true;
        }
        geometry_msgs::msg::PoseStamped source_pose_msg;
        source_pose_msg.header.frame_id = source_frame;
        source_pose_msg.header.stamp = tf_lookup_time_override_ ? *tf_lookup_time_override_ : this->now();
        source_pose_msg.pose.position.x = source_pose.x;
        source_pose_msg.pose.position.y = source_pose.y;
        source_pose_msg.pose.position.z = source_pose.z;
        tf2::Quaternion q;
        q.setRPY(0, 0, source_pose.yaw);
        source_pose_msg.pose.orientation = tf2::toMsg(q);
        try {
            geometry_msgs::msg::PoseStamped target_pose_msg = 
                tf_buffer_->transform(source_pose_msg, map_frame_id_, 
                                    tf2::durationFromSec(transform_timeout_));
            target_pose.x = target_pose_msg.pose.position.x;
            target_pose.y = target_pose_msg.pose.position.y;
            target_pose.z = target_pose_msg.pose.position.z;
            tf2::Quaternion q_out;
            tf2::fromMsg(target_pose_msg.pose.orientation, q_out);
            target_pose.yaw = tf2::impl::getYaw(q_out);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
            transformation_error_ = true;
            return false;
        }
    }

    px4_msgs::msg::TrajectorySetpoint getCurrentSetpointMSG_NED(long long current_timestamp_micros) const {
        Pose setpoint_NED = getCurrentSetpoint_NED(current_timestamp_micros);
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = current_timestamp_micros;
        msg.position[0] = setpoint_NED.x;
        msg.position[1] = setpoint_NED.y;
        msg.position[2] = setpoint_NED.z;
        msg.yaw = setpoint_NED.yaw;
        msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
        return msg;
    }

    Pose getCurrentSetpoint_NED(long long current_timestamp_micros) const {
        Pose result_pose_ned = start_pose_ned_;
        if (total_pos_time_micros_ == LLONG_MAX) {
            RCLCPP_ERROR(logger_, "SpeedController: Error in setup (pos speed <= 0), returning start pos.");
        } else if (total_pos_time_micros_ > 0) {
            long long elapsed_pos_time_micros = current_timestamp_micros - start_timestamp_micros_;
            double pos_ratio = std::min(static_cast<double>(elapsed_pos_time_micros) / static_cast<double>(total_pos_time_micros_), 1.0);
            result_pose_ned.x = start_pose_ned_.x + (target_pose_ned_.x - start_pose_ned_.x) * pos_ratio;
            result_pose_ned.y = start_pose_ned_.y + (target_pose_ned_.y - start_pose_ned_.y) * pos_ratio;
            result_pose_ned.z = start_pose_ned_.z + (target_pose_ned_.z - start_pose_ned_.z) * pos_ratio;
        } else {
            result_pose_ned.x = target_pose_ned_.x;
            result_pose_ned.y = target_pose_ned_.y;
            result_pose_ned.z = target_pose_ned_.z;
        }
        result_pose_ned.yaw = target_pose_ned_.yaw;
        return result_pose_ned;
    }

    bool isEnd(const Pose& current_drone_pose_ned, double position_tolerance = 0.08, double yaw_tolerance = 0.08) const {
        if (transformation_error_) {
            return true;
        }
        // Complete only at target (position + yaw), not by timer — avoids early exit/overshoot;
        // slow drone may take longer than the nominal motion time.
        double distance_to_target = TransformPX4::getDistance(current_drone_pose_ned, target_pose_ned_);
        bool position_reached = distance_to_target <= position_tolerance;
        double yaw_diff = current_drone_pose_ned.yaw - target_pose_ned_.yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
        bool yaw_reached = std::abs(yaw_diff) <= yaw_tolerance;
        return position_reached && yaw_reached;
    }

    bool isFinished(long long current_timestamp_micros) const {
        if (total_pos_time_micros_ == LLONG_MAX || total_yaw_time_micros_ == LLONG_MAX) {
             return true;
        }
        long long max_required_time = std::max(total_pos_time_micros_, total_yaw_time_micros_);
        return (current_timestamp_micros - start_timestamp_micros_) >= max_required_time;
    }

    bool hasTransformationError() const {
        return transformation_error_;
    }

    builtin_interfaces::msg::Time now() const {
        auto now = std::chrono::system_clock::now();
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto epoch = now_ns.time_since_epoch();
        uint64_t nanoseconds = epoch.count();
        builtin_interfaces::msg::Time time_msg;
        time_msg.sec = static_cast<int32_t>(nanoseconds / 1000000000);
        time_msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000);
        return time_msg;
    }

private:
    rclcpp::Logger logger_;
    Pose start_pose_ned_;
    Pose target_pose_ned_;
    double target_speed_;
    long long start_timestamp_micros_;
    double distance_;
    double yaw_difference_;
    long long total_pos_time_micros_;
    long long total_yaw_time_micros_;
    tf2_ros::Buffer* tf_buffer_;
    double transform_timeout_;
    std::string current_frame_id_;
    std::string map_frame_id_;
    std::optional<builtin_interfaces::msg::Time> tf_lookup_time_override_;
    mutable bool transformation_error_ = false;
};

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode(std::string px4_namespace);
    ~OffboardControlNode();
private:
    // --- TF ---
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped current_odom_transform_;
    geometry_msgs::msg::TransformStamped body_transform_;
    geometry_msgs::msg::TransformStamped target_transform_;
    geometry_msgs::msg::TransformStamped setpoint_transform_;
    geometry_msgs::msg::TransformStamped terrain_transform_;
    
    // --- Reference frames ---
    std::map<std::string, std::string> reference_frames_;

    // --- Service servers ---
    rclcpp::Service<offboard_interfaces::srv::Navigate>::SharedPtr navigate_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
    rclcpp::Service<offboard_interfaces::srv::GetTelemetry>::SharedPtr get_telemetry_service_;
    rclcpp::Service<offboard_interfaces::srv::SetAltitude>::SharedPtr set_altitude_service_;
    rclcpp::Service<offboard_interfaces::srv::SetYaw>::SharedPtr set_yaw_service_;
    rclcpp::Service<offboard_interfaces::srv::SetYawRate>::SharedPtr set_yaw_rate_service_;
    rclcpp::Service<offboard_interfaces::srv::SetPosition>::SharedPtr set_position_service_;
    rclcpp::Service<offboard_interfaces::srv::SetVelocity>::SharedPtr set_velocity_service_;
    rclcpp::Service<offboard_interfaces::srv::SetAttitude>::SharedPtr set_attitude_service_;
    rclcpp::Service<offboard_interfaces::srv::SetRates>::SharedPtr set_rates_service_;
    rclcpp::Service<offboard_interfaces::srv::Flip>::SharedPtr flip_service_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    // --- Subscribers ---
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_pos_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_subscriber_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    // --- Frame parameters ---
    std::string body_frame_id_;
    std::string map_frame_id_;
    std::string aruco_map_frame_id_;

    // --- Parameters ---
    double transform_timeout_;
    double offboard_timeout_;
    double arming_timeout_;
    double default_speed_;
    bool land_only_in_offboard_;
    bool check_kill_switch_;
    double local_position_timeout_;
    double global_position_timeout_;
    double battery_timeout_;
    double manual_control_timeout_;
    bool is_simulator_;
    double control_rate_;

    // --- State ---
    px4_msgs::msg::VehicleStatus current_vehicle_status_;
    uint64_t current_vehicle_status_timestamp_;
    float battery_voltage;
    float battery_percentage;
    uint64_t current_battery_timestamp_;
    double lat_{0.0}, lon_{0.0}, alt_{0.0};
    uint64_t current_global_position_timestamp_;
    px4_msgs::msg::ManualControlSetpoint current_manual_control_;
    uint64_t current_manual_control_timestamp_;
    SetpointType setpoint_type_ = SetpointType::NONE;
    CurrentPose current_local_position_;
    px4_msgs::msg::TrajectorySetpoint last_sended_pose;

    // Overrides for NAVIGATE / POSITION: altitude, yaw, yaw_rate
    bool altitude_override_set_ = false;
    double altitude_override_z_ = 0.0;
    std::string altitude_override_frame_id_;
    bool yaw_override_set_ = false;
    double yaw_override_ = 0.0;
    std::string yaw_override_frame_id_;
    bool yaw_rate_override_set_ = false;
    double yaw_rate_override_ = 0.0;

    // Direct setpoint state for POSITION / VELOCITY
    double position_target_x_ = 0.0, position_target_y_ = 0.0, position_target_z_ = 0.0, position_target_yaw_ = 0.0;
    std::string position_target_frame_id_;
    double velocity_target_vx_ = 0.0, velocity_target_vy_ = 0.0, velocity_target_vz_ = 0.0, velocity_target_yaw_ = 0.0;
    std::string velocity_target_frame_id_;

    // Attitude / rates setpoint state
    float attitude_roll_ = 0.0f, attitude_pitch_ = 0.0f, attitude_yaw_ = 0.0f, attitude_thrust_ = 0.5f;
    std::string attitude_frame_id_;
    float rates_roll_ = 0.0f, rates_pitch_ = 0.0f, rates_yaw_ = 0.0f, rates_thrust_ = 0.5f;

    // --- Flip state ---
    FlipPhase flip_phase_ = FlipPhase::CLIMB;
    Pose flip_initial_pose_ned_;
    FlipAxis flip_axis_ = FlipAxis::ROLL;
    float flip_vz_ = 1.0f;
    float flip_climb_duration_ = 0.0f;
    float flip_rate_ = 0.0f;
    float flip_target_angle_ = 0.0f;
    float flip_thrust_ = 0.5f;
    rclcpp::Time flip_climb_start_time_;
    rclcpp::Time flip_rotate_start_time_;
    double flip_angle_done_ = 0.0;

    // --- Speed controller ---
    std::unique_ptr<SpeedController> global_speed_controller_;

    // --- Flags ---
    bool busy_ = false;
    bool wait_armed_ = false;

    // Reference yaw at first odometry (so reported/display yaw is 0 at startup)
    bool initial_heading_set_ = false;
    double initial_heading_ned_ = 0.0;
    static double normalizeYaw(double yaw);

    // Callback groups for concurrent execution
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    std::shared_ptr<px4_msgs::msg::VehicleStatus> prev_vehicle_status_msg_;

    // --- TF init ---
    void create_reference_frames();
    void init_tf_frames();

    // --- TF helpers ---
    bool wait_for_transform(
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time,
        const rclcpp::Duration& timeout);
    
    std::string get_reference_frame(const std::string& frame_id);
    
    // --- Generic transforms ---
    bool transform_point(
        const geometry_msgs::msg::Point& point,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        geometry_msgs::msg::Point& result);
    
    bool transform_yaw(
        double yaw,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        double& result_yaw);
    
    bool transform_velocity(
        const geometry_msgs::msg::Vector3& velocity,
        const std::string& source_frame,
        const std::string& target_frame,
        const rclcpp::Time& stamp,
        geometry_msgs::msg::Vector3& result);

    // --- Publish TF ---
    void clear_navigation_targets();
    void publish_static_transforms();
    void publish_setpoint_frame(const Pose& setpoint_pose_ned, const rclcpp::Time& stamp);
    void publish_terrain_frame(double altitude_above_ground);
    void publish_odometry_transform(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publish_target_frame(
        const Pose& target_pose, 
        const std::string& frame_id,
        const rclcpp::Time& stamp);

    // --- Helpers ---
    void Init_publishers(std::string px4_namespace);
    void Init_subscribers(std::string px4_namespace);
    void Init_services();
    void Init_timer();
    void Init_SpeedController(
        const Pose& start_pose_ned, 
        const Pose& target_pose_any_frame,
        const std::string& frame_id,
        double speed);
    bool validate_frame_id(const std::string& frame_id);
    bool get_altitude_ned_from_frame(double z, const std::string& frame_id, double& out_ned_z);
    bool transform_yaw_to_ned(double yaw, const std::string& frame_id, double& out_ned_yaw);
    /** Time for TF lookups: latest (0) in simulator to avoid extrapolation; now() on real drone. */
    rclcpp::Time get_transform_lookup_time() const;
    /** When source or target is aruco_map (tf_static), use time 0 so static transform is found. */
    rclcpp::Time get_transform_lookup_time(const std::string& source_frame, const std::string& target_frame) const;

    // --- Control ---
    void checkManualControl();
    bool offboardAndArm();

    // --- PX4 commands ---
    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void switch_to_offboard_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    bool land();
    void publish_idle_setpoint();

    // --- Service callbacks ---
    void navigate_callback(
        const offboard_interfaces::srv::Navigate::Request::SharedPtr request,
                           const offboard_interfaces::srv::Navigate::Response::SharedPtr response);
    void land_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
                       const std_srvs::srv::Trigger::Response::SharedPtr response);
    void get_telemetry_callback(
        const offboard_interfaces::srv::GetTelemetry::Request::SharedPtr request,
                                const offboard_interfaces::srv::GetTelemetry::Response::SharedPtr response);
    void set_altitude_callback(const offboard_interfaces::srv::SetAltitude::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetAltitude::Response::SharedPtr response);
    void set_yaw_callback(const offboard_interfaces::srv::SetYaw::Request::SharedPtr request,
                          const offboard_interfaces::srv::SetYaw::Response::SharedPtr response);
    void set_yaw_rate_callback(const offboard_interfaces::srv::SetYawRate::Request::SharedPtr request,
                               const offboard_interfaces::srv::SetYawRate::Response::SharedPtr response);
    void set_position_callback(const offboard_interfaces::srv::SetPosition::Request::SharedPtr request,
                               const offboard_interfaces::srv::SetPosition::Response::SharedPtr response);
    void set_velocity_callback(const offboard_interfaces::srv::SetVelocity::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetVelocity::Response::SharedPtr response);
    void set_attitude_callback(const offboard_interfaces::srv::SetAttitude::Request::SharedPtr request,
                              const offboard_interfaces::srv::SetAttitude::Response::SharedPtr response);
    void set_rates_callback(const offboard_interfaces::srv::SetRates::Request::SharedPtr request,
                           const offboard_interfaces::srv::SetRates::Response::SharedPtr response);
    void flip_callback(const offboard_interfaces::srv::Flip::Request::SharedPtr request,
                       const offboard_interfaces::srv::Flip::Response::SharedPtr response);

    // --- Topic callbacks ---
    void batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    // --- Main loop helpers ---
    void publish_setpoint();
    void stopOffboardHeartbeat();
    void startOffboardHeartbeat();

    // --- Utils ---
    uint64_t getTimestamp();
    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    bool service_done_;
    uint8_t service_result_;
};

#endif // OFFBOARD_CONTROL_HPP_