#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/qos.hpp"

class LandingController : public rclcpp::Node {
public: 
    LandingController();

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_pose_stamp_;

    float current_yaw_ {0.0};
    float latest_x_ {0}, latest_y_ {0}, latest_z_ {0};
    bool have_pose_ {false};

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    void control_loop();
};