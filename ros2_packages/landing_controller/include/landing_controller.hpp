#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/qos.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

class LandingController : public rclcpp::Node {
public: 
    LandingController();

private:
    enum class State {
        APPROACH, // P-controlled lateral + descent, tracking the marker
        LAND,     // fixed descent rate, still offboard, still centering
        LANDED
    };

    State current_state_ {State::APPROACH};

    static constexpr float kLandAltitudeThreshold {2.0f}; // meters - latest_z_ below this triggers LAND
    static constexpr float kLandDescentVelocity {0.3f}; // m/s, fixed - replaces decaying P term once in LAND

    float current_altitude_ {0.0f};

    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_pose_stamp_ {static_cast<int64_t>(0), RCL_ROS_TIME};

    float current_yaw_ {0.0};
    float latest_x_ {0}, latest_y_ {0}, latest_z_ {0};
    bool have_pose_ {false};
    bool have_local_position_ {false};
    bool is_landed_ {false};

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    void control_loop();

    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void update_state();
    void send_disarm_command();
};