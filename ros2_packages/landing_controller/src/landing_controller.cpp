#include "landing_controller.hpp"
#include <algorithm>
#include <cmath>

LandingController::LandingController() : Node {"landing_controller"} {
    RCLCPP_INFO(this->get_logger(), "Landing controller initialized");
    RCLCPP_INFO(this->get_logger(), "K_descent_: %.1f", K_descent_);

    rclcpp::QoS qos_profile {10};
    qos_profile.best_effort();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10, std::bind(&LandingController::pose_callback, this, std::placeholders::_1));

    local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos_profile, std::bind(&LandingController::local_position_callback, this, std::placeholders::_1));

    trajectory_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);

    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LandingController::control_loop, this));

    land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos_profile, std::bind(&LandingController::land_detected_callback, this, std::placeholders::_1));

    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
}

void LandingController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // extract position
    float x_dist { static_cast<float>(msg->pose.position.x) };
    float y_dist { static_cast<float>(msg->pose.position.y) };
    float z_dist { static_cast<float>(msg->pose.position.z) };

    // sanity checks - reject NaN/Inf
    if (!std::isfinite(x_dist) || !std::isfinite(y_dist) || !std::isfinite(z_dist)) {
        RCLCPP_WARN(this->get_logger(), "Rejected pose: non-finite value (x=%.3f, y=%.3f, z=%.3f)", x_dist, y_dist, z_dist);
        return;
    }

    if (z_dist < 0.0f) {
        RCLCPP_WARN(this->get_logger(), "Rejected pose: negative z distance (%.3f)", z_dist);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "pose passed sanity checks");

    if(!filter_initialized_) {
        filtered_x_ = x_dist;
        filtered_y_ = y_dist;
        filtered_z_ = z_dist;
        filter_initialized_ = true;
    } else {
        double dt = (rclcpp::Time(msg->header.stamp) - last_pose_stamp_).seconds();
        if (dt > 0.0) {
            float alpha = static_cast<float>(dt / (kPoseFilterTau_ + dt));
            filtered_x_ = alpha * x_dist + (1.0f - alpha) * filtered_x_;
            filtered_y_ = alpha * y_dist + (1.0f - alpha) * filtered_y_;
            filtered_z_ = alpha * z_dist + (1.0f - alpha) * filtered_z_;
        }
    }

    RCLCPP_INFO(this->get_logger(), "raw x=%.3f y=%.3f z=%.3f | filtered: x=%.3f y=%.3f z=%.3f", x_dist, y_dist, z_dist, filtered_x_, filtered_y_, filtered_z_);

    // get time stamp after sanity checks (rejected poses are not 'fresh')
    last_pose_stamp_ = rclcpp::Time(msg->header.stamp);
    RCLCPP_DEBUG(this->get_logger(), "last_pose_stamp_ = %.3f, this->now() = %.3f", last_pose_stamp_.seconds(), this->now().seconds());

    latest_x_ = x_dist;
    latest_y_ = y_dist;
    latest_z_ = z_dist;
    have_pose_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Distance: x=%.3f, y=%.3f, z=%.3f", x_dist, y_dist, z_dist);
}

void LandingController::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_yaw_ = msg->heading;
    current_altitude_ = -msg->z;  // NED - +z is below starting positiong, -z is above
    have_local_position_ = true;
    vehicle_x_ = msg->x;
    vehicle_y_ = msg->y;
}

void LandingController::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    is_landed_ = msg->landed;
}

void LandingController::update_state() {
    switch (current_state_) {
    case State::APPROACH:
        if (have_local_position_ && current_altitude_ < kLandAltitudeThreshold_) {
            RCLCPP_INFO(this->get_logger(), "Altitude threshold crossed (%.3f < %.1f) - transitioning to LAND", current_altitude_, kLandAltitudeThreshold_);
            current_state_ = State::LAND;

            integral_x_ = 0.0f;
            integral_y_ = 0.0f;
        }
        break;
    
    case State::LAND:
        if (is_landed_) {
            RCLCPP_INFO(this->get_logger(), "transitioning to LANDED");
            current_state_ = State::LANDED;
            send_disarm_command();
        }
        break;

    case State::LANDED:
        break; // end
    }
}

void LandingController::send_disarm_command() {
    auto cmd = px4_msgs::msg::VehicleCommand {};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = 0.0; // disarm
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.from_external = true;
    cmd.timestamp = this->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(cmd);
}

void LandingController::control_loop() {
    update_state();

    if (current_state_ == State::LANDED) {
        return; // disarmed - stop publishing offboard setpoints
    }
    
    rclcpp::Time now = this->now();
    float dt = 0.0f;
    if (last_control_time_.nanoseconds() != 0) {
        dt = (now - last_control_time_).seconds();
    }
    last_control_time_ = now;

    // create & publish offboard control mode message, regadless of pose freshness
    auto offboard_msg = px4_msgs::msg::OffboardControlMode {};
    offboard_msg.position = false;
    offboard_msg.velocity = true; // we're sending velocity commands
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_msg.timestamp = this->now().nanoseconds() / 1000; // microseconds
    offboard_control_mode_publisher_->publish(offboard_msg);

    float vx_ned = 0.0f, vy_ned = 0.0f, vz = 0.0f;

    if (have_pose_) {
        // check age of pose
        auto pose_age = this->now() - last_pose_stamp_; // rclcpp::Time - rclcpp::Time returns as an rclcpp::Duration
        const auto max_pose_age = rclcpp::Duration::from_seconds(0.15);
        // Note - 0.15 above is 150 ms, 3x our loop period to ensure genuinely stale.
        // pose processing times are around .03 ms, plenty of headroom

        RCLCPP_DEBUG(this->get_logger(), "Pose age %.3f s", pose_age.seconds());

        if (pose_age > max_pose_age) {
            RCLCPP_DEBUG(this->get_logger(), "Pose stale (%.3f s old) - holding position", pose_age.seconds());
            // TODO: decide behavior - hold, publish zero velocity, or trigger search

            if (current_state_ == State::LAND) {
                vx_ned = 0.0f;
                vy_ned = 0.0f;
                vz = kLandDescentVelocity_; // keep descending if marker is lost since too close to see
            } else {
                RCLCPP_WARN(this->get_logger(), "Holding position - stale pose in APPROACH");
                return;
            }
        } else if (current_state_ == State::APPROACH) {
            // integral accumulation = running total of error over time
            integral_x_ += filtered_x_ * dt;
            integral_y_ += filtered_y_ * dt;

            // anti-windup - clamp accumulated integral
            integral_y_ = std::clamp(integral_y_, -kIntegralMax_, kIntegralMax_);
            integral_x_ = std::clamp(integral_x_, -kIntegralMax_, kIntegralMax_);

            float derivative_x = 0.0f, derivative_y = 0.0f;
            if (derivative_initialized_ && dt > 0.0f) {
                derivative_x = (filtered_x_ - previous_error_x_) / dt;
                derivative_y = (filtered_y_ - previous_error_y_) / dt;
            }

            previous_error_x_ = filtered_x_;
            previous_error_y_ = filtered_y_;
            derivative_initialized_ = true;

            if (!derivative_filter_initialized_) {
                filtered_derivative_x_ = derivative_x;
                filtered_derivative_y_ = derivative_y;
                derivative_filter_initialized_ = true;
            } else if (dt > 0.0f) {
                float alpha = dt / (kDerivativeFilterTau_ + dt);
                filtered_derivative_x_ = alpha * derivative_x + (1.0f - alpha) * filtered_derivative_x_;
                filtered_derivative_y_ = alpha * derivative_y + (1.0f - alpha) * filtered_derivative_y_;
            }

            // calculate velocities (camera frame x, y offset -> body frame velocity)
            float vx = -filtered_y_ * K_lateral_ - integral_y_ * Ki_lateral_ - filtered_derivative_y_ * Kd_lateral_; // y offset -> x velocity (forward/back)
            float vy = filtered_x_ * K_lateral_ + integral_x_ * Ki_lateral_ + filtered_derivative_x_ * Kd_lateral_; // x offset -> y velocity (left/right)
            vz = (current_state_ == State::LAND) ? kLandDescentVelocity_ : (filtered_z_ * K_descent_);

            float p_contrib_x = filtered_x_ * K_lateral_;
            float i_contrib_x = integral_x_ * Ki_lateral_;
            float d_contrib_x = derivative_x * Kd_lateral_;
            // RCLCPP_INFO(this->get_logger(), "vy=%.3f | P=%.3f I=%.3f D=%.3f", vy, p_contrib_x, i_contrib_x, d_contrib_x);

            float p_contrib_y = -filtered_y_ * K_lateral_;
            float i_contrib_y = -integral_y_ * Ki_lateral_;
            float d_contrib_y = -derivative_y * Kd_lateral_;
            // RCLCPP_INFO(this->get_logger(), "vx=%.3f | P=%.3f I=%.3f D=%.3f", vx, p_contrib_y, i_contrib_y, d_contrib_y);

            // add rotation
            vx_ned = vx * cos(current_yaw_) - vy * sin(current_yaw_);
            vy_ned = vx * sin(current_yaw_) + vy * cos(current_yaw_);
            RCLCPP_INFO(this->get_logger(), "vx_ned=%.3f vy_ned=%.3f | vehicle x=%.3f y=%.3f | yaw=%.4f", vx_ned, vy_ned, vehicle_x_, vehicle_y_, current_yaw_);
        } else {  // have pose, pose is fresh, not in APPROACH state
            vx_ned = 0.0f;
            vy_ned = 0.0f;
            vz = kLandDescentVelocity_; // keep descending if marker is lost since too close to see
        }
    }

    vx_ned = std::clamp(vx_ned, -kMaxLateralVelocity_, kMaxLateralVelocity_);
    vy_ned = std::clamp(vy_ned, -kMaxLateralVelocity_, kMaxLateralVelocity_);
    vz = std::clamp(vz, 0.0f, kMaxDescentVelocity_);

    RCLCPP_DEBUG(this->get_logger(), "clamped: vx_ned = %.3f, vy_ned = %.3f, vz = %.3f", vx_ned, vy_ned, vz);

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

    trajectory_publisher_->publish(trajectory_msg);

    RCLCPP_INFO(this->get_logger(), "Published: vx_ned = %.3f, vy_ned = %.3f, vz = %.3f", vx_ned, vy_ned, vz);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingController>());
    rclcpp::shutdown();
    return 0;
}
