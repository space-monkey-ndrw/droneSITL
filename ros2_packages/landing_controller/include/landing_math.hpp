#pragma once

#include <cmath>
#include <algorithm>

// Pure math helpers extracted from LandingController so they can be unit
// tested without spinning up ROS/PX4/Gazebo. No rclcpp, no message types —
// just floats in, floats out.

struct Vec3 {
    float x {0.0f};
    float y {0.0f};
    float z {0.0f};
};

// Extracts roll and pitch (NOT yaw) from a PX4 VehicleAttitude-style
// quaternion, given as separate components in (w, x, y, z) order.
// Yaw is intentionally omitted — see write-up: the existing PID output
// rotation already handles yaw, so re-applying it here would double-correct.
inline void quat_to_roll_pitch(float w, float x, float y, float z, float& roll, float& pitch) {
    roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    float sin_pitch = 2.0f * (w * y - z * x);
    sin_pitch = std::clamp(sin_pitch, -1.0f, 1.0f); // guard asin domain near gimbal lock
    pitch = std::asin(sin_pitch);
}

// Rotates a raw /aruco_pose tvec (camera frame: +x right, +y down, +z
// forward/boresight) into a tilt-corrected measurement, expressed back in
// the same camera-style axes so downstream filtering/PID code is unchanged.
//
// Three explicit steps, each independently testable:
//   1. camera -> body (FRD): fixed mount rotation (image-up = body-forward)
//   2. body -> level-body: undo roll/pitch tilt
//   3. level-body -> camera-style axes: inverse of step 1
//
// When roll == pitch == 0.0f, this is the identity transform.
inline Vec3 derotate_camera_to_level(const Vec3& cam, float roll, float pitch) {
    // step 1: camera frame -> body frame
    // camera: +x right, +y down, +z forward (out of camera lens, towards ground)
    // body: image-up = body-forward (+x body is -y image), image-right = body-right (+y body is +x image)
    float x_body = -cam.y;
    float y_body =  cam.x;
    float z_body =  cam.z;

    // step 2: body FRD -> level-body frame (remove roll/pitch tilt)
    float sr = std::sin(roll),  cr = std::cos(roll);
    float sp = std::sin(pitch), cp = std::cos(pitch);

    // Rx (roll) applied first
    float x1 = x_body;
    float y1 = y_body * cr - z_body * sr;
    float z1 = y_body * sr + z_body * cr;

    // then Ry (pitch)
    float x_level =  x1 * cp + z1 * sp;
    float y_level =  y1;
    float z_level = -x1 * sp + z1 * cp;

    // step 3: level-body frame -> back to camera frame (keeps downstream PID unchanged)
    Vec3 out;
    out.x =  y_level;
    out.y = -x_level;
    out.z =  z_level;
    return out;
}

// Archimedean spiral velocity feedforward, in NED (frame-independent of
// vehicle heading — this is an inertial pattern, not a body-frame one).
//   r(theta) = growth_rate * theta,   theta = omega * elapsed
// Radius is capped at max_radius; once capped, dr/dt becomes 0 and the
// drone circles at fixed radius instead of continuing to expand outward.
inline Vec3 spiral_velocity(float elapsed, float omega, float growth_rate, float max_radius) {
    float theta = omega * elapsed;
    float r = std::min(growth_rate * theta, max_radius);

    bool still_growing = (r < max_radius);
    float dr_dt = still_growing ? (growth_rate * omega) : 0.0f;

    Vec3 out;
    out.x = dr_dt * std::cos(theta) - r * std::sin(theta) * omega;
    out.y = dr_dt * std::sin(theta) + r * std::cos(theta) * omega;
    out.z = 0.0f;
    return out;
}