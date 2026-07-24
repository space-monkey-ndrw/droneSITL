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
        LAND,     // fixed descent rate, still offboard
        LANDED
    };

    State current_state_ {State::APPROACH};

    static constexpr float kLandAltitudeThreshold_ {2.5f}; // meters - latest_z_ below this triggers LAND
    static constexpr float kLandDescentVelocity_ {0.3f}; // m/s, fixed - replaces decaying P term once in LAND

    float current_altitude_ {0.0f};

    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_pose_stamp_ {static_cast<int64_t>(0), RCL_ROS_TIME};

    float current_yaw_ {0.0f};
    float vehicle_x_ {0.0f}, vehicle_y_ {0.0f};
    float latest_x_ {0.0f}, latest_y_ {0.0f}, latest_z_ {0.0f};
    float filtered_x_ {0.0f}, filtered_y_ {0.0f}, filtered_z_ {0.0f};
    float integral_x_ {0.0f}, integral_y_ {0.0f};
    float previous_error_x_ {0.0f}, previous_error_y_ {0.0f};
    float filtered_derivative_x_ {0.0f}, filtered_derivative_y_ {0.0f};
    bool filter_initialized_ {false};
    bool derivative_initialized_ {false};
    bool derivative_filter_initialized_ {false};
    bool have_pose_ {false};
    bool have_local_position_ {false};
    bool is_landed_ {false};

    static constexpr float kPoseFilterTau_ {0.1f}; // seconds - smoothing time constant
    static constexpr float kMaxLateralVelocity_ {1.0f};
    static constexpr float kMaxDescentVelocity_ {10.0f};
    static constexpr float kDerivativeFilterTau_ {0.1f}; // seconds — placeholder, tune by testing

    // proportional gain
    static constexpr float K_lateral_ {0.1f}; // .1 has almost no zigzag, .2 has some, .5 has a lot
    static constexpr float K_descent_ {1.0f};

    static constexpr float Ki_lateral_ {0.001f};  // placeholder — tuned later in step 3
    rclcpp::Time last_control_time_ {static_cast<int64_t>(0), RCL_ROS_TIME};
    static constexpr float kIntegralMax_ {0.003f}; // placeholder cap — tune in step 3 (gain-tuning pass)

    static constexpr float Kd_lateral_ {0.01f};

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    void control_loop();

    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void update_state();
    void send_disarm_command();
};