#include "landing_controller.hpp"
#include <cmath>

LandingController::LandingController() : Node {"landing_controller"} {
    RCLCPP_INFO(this->get_logger(), "Landing controller initialized");

    rclcpp::QoS qos_profile {10};
    qos_profile.best_effort();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10, std::bind(&LandingController::pose_callback, this, std::placeholders::_1));

    local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos_profile, std::bind(&LandingController::local_position_callback, this, std::placeholders::_1));

    trajectory_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);

    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LandingController::control_loop, this));
}

void LandingController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // get time stamp on pose arrival
    last_pose_stamp_ = rclcpp::Time(msg->header.stamp);
    RCLCPP_INFO(this->get_logger(), "last_pose_stamp_ = %.3f, this->now() = %.3f", last_pose_stamp_.seconds(), this->now().seconds());

    // extract position
    float x_dist { static_cast<float>(msg->pose.position.x) };
    float y_dist { static_cast<float>(msg->pose.position.y) };
    float z_dist { static_cast<float>(msg->pose.position.z) };

    latest_x_ = x_dist;
    latest_y_ = y_dist;
    latest_z_ = z_dist;
    have_pose_ = true;

    // for now, just log it
    RCLCPP_INFO(this->get_logger(), "Distance: x=%.3f, y=%.3f, z=%.3f", x_dist, y_dist, z_dist);
}

void LandingController::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_yaw_ = msg->heading;
}

void LandingController::control_loop() {
    if (have_pose_) {
        // check age of pose
        rclcpp::Duration pose_age = this->now() - last_pose_stamp_;
        const rclcpp::Duration max_pose_age = rclcpp::Duration::from_seconds(0.15);
        // TODO - tune this. 0.15 above is 150 ms, 3x our loop period to ensure genuinely stale.

        RCLCPP_INFO(this->get_logger(), "Pose age %.3f s", pose_age.seconds());

        if (pose_age > max_pose_age) {
            RCLCPP_WARN(this->get_logger(), "Pose stale (%.3f s old) - holding position", pose_age.seconds());
            // TODO: decide behavior - hold, publish zero velocity, or trigger search
            // currently, early return will stop publishing & lets PX4 offboard safety timeout engage
            return;
        }

        // proportional gain
        float K_lateral {0.5}; // todo - tune this
        float K_descent {0.3};

        // calculate velocities (camera frame x, y offset -> body frame velocity)
        float vx = -latest_y_ * K_lateral; // y offset -> x velocity (forward/back)
        float vy = latest_x_ * K_lateral; // x offset -> y velocity (left/right)
        float vz = latest_z_ * K_descent; // z distance -> descent velocity

        // add rotation
        float vx_ned = vx * cos(current_yaw_) - vy * sin(current_yaw_);
        float vy_ned = vx * sin(current_yaw_) + vy * cos(current_yaw_);

        // create trajectory setpoint message
        auto trajectory_msg = px4_msgs::msg::TrajectorySetpoint {};
        trajectory_msg.velocity[0] = vx_ned;
        trajectory_msg.velocity[1] = vy_ned;
        trajectory_msg.velocity[2] = vz;
        trajectory_msg.timestamp = this->now().nanoseconds() / 1000; // microseconds

        // TrajectorySetpoint needs unused fields to be NaN
        trajectory_msg.position = {NAN, NAN, NAN};
        trajectory_msg.acceleration = {NAN, NAN, NAN};
        trajectory_msg.yaw = NAN;
        trajectory_msg.yawspeed = NAN;

        // create offboard control mode message
        auto offboard_msg = px4_msgs::msg::OffboardControlMode {};
        offboard_msg.position = false;
        offboard_msg.velocity = true; // we're sending velocity commands
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;

        offboard_msg.timestamp = this->now().nanoseconds() / 1000; // microseconds

        // publish both messages
        offboard_control_mode_publisher_->publish(offboard_msg);
        trajectory_publisher_->publish(trajectory_msg);

        RCLCPP_INFO(this->get_logger(), "Published: vx = %.3f, vy = %.3f, vz = %.3f", vx, vy, vz);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingController>());
    rclcpp::shutdown();
    return 0;
}
