#include <gtest/gtest.h>
#include "landing_math.hpp"

// ---------- quat_to_roll_pitch ----------

TEST(QuatToRollPitch, IdentityQuatGivesZero) {
    float roll, pitch;
    quat_to_roll_pitch(1.0f, 0.0f, 0.0f, 0.0f, roll, pitch);
    EXPECT_NEAR(roll, 0.0f, 1e-5);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
}

TEST(QuatToRollPitch, PureRollRotationRecovered) {
    float theta = 0.1f;
    // rotation of theta about body x-axis -> q = (cos(theta/2), sin(theta/2), 0, 0)
    float roll, pitch;
    quat_to_roll_pitch(std::cos(theta / 2), std::sin(theta / 2), 0.0f, 0.0f, roll, pitch);
    EXPECT_NEAR(roll, theta, 1e-4);
    EXPECT_NEAR(pitch, 0.0f, 1e-5);
}

TEST(QuatToRollPitch, PurePitchRotationRecovered) {
    float theta = 0.1f;
    // rotation of theta about body y-axis -> q = (cos(theta/2), 0, sin(theta/2), 0)
    float roll, pitch;
    quat_to_roll_pitch(std::cos(theta / 2), 0.0f, std::sin(theta / 2), 0.0f, roll, pitch);
    EXPECT_NEAR(pitch, theta, 1e-4);
    EXPECT_NEAR(roll, 0.0f, 1e-5);
}

TEST(QuatToRollPitch, ClampsNearGimbalLock) {
    // deliberately pathological input that would push asin() out of domain
    // without clamping -- should not NaN or throw.
    float roll, pitch;
    quat_to_roll_pitch(0.001f, 0.0f, 10.0f, 0.0f, roll, pitch);
    EXPECT_FALSE(std::isnan(pitch));
}

// ---------- derotate_camera_to_level ----------

TEST(Derotation, IdentityWhenLevel) {
    Vec3 in {1.0f, 2.0f, 3.0f};
    Vec3 out = derotate_camera_to_level(in, 0.0f, 0.0f);
    EXPECT_NEAR(out.x, in.x, 1e-5);
    EXPECT_NEAR(out.y, in.y, 1e-5);
    EXPECT_NEAR(out.z, in.z, 1e-5);
}

TEST(Derotation, RollOnlyShiftsXNotY) {
    // marker dead-center in image (no lateral offset), drone rolled 0.1 rad
    Vec3 out = derotate_camera_to_level({0.0f, 0.0f, 5.0f}, 0.1f, 0.0f);
    EXPECT_NEAR(out.x, -0.4992f, 1e-3);  // regression value, confirmed by running the math
    EXPECT_NEAR(out.y, 0.0f, 1e-4);
    EXPECT_GT(out.z, 0.0f);              // still in front of / below the camera
}

TEST(Derotation, PitchOnlyShiftsYNotX) {
    Vec3 out = derotate_camera_to_level({0.0f, 0.0f, 5.0f}, 0.0f, 0.1f);
    EXPECT_NEAR(out.x, 0.0f, 1e-4);
    EXPECT_NEAR(out.y, -0.4992f, 1e-3); // regression value, confirmed by running the math
    EXPECT_GT(out.z, 0.0f);
}

TEST(Derotation, SignFlipsWithOppositeTilt) {
    // if tilting +0.1 rad shifts x negative, tilting -0.1 rad should shift it
    // positive by (approximately) the same magnitude -- catches sign errors
    // introduced by refactoring without needing an exact expected value.
    Vec3 pos_roll = derotate_camera_to_level({0.0f, 0.0f, 5.0f}, 0.1f, 0.0f);
    Vec3 neg_roll = derotate_camera_to_level({0.0f, 0.0f, 5.0f}, -0.1f, 0.0f);
    EXPECT_NEAR(pos_roll.x, -neg_roll.x, 1e-3);
}

// ---------- spiral_velocity ----------

TEST(Spiral, StartsAtNonzeroOutwardVelocity) {
    // at t=0, theta=0, so vx should equal dr/dt (pure outward radial motion)
    // and vy should be 0 (no tangential component yet)
    Vec3 v = spiral_velocity(0.0f, 0.4f, 0.3f, 10.0f);
    EXPECT_NEAR(v.y, 0.0f, 1e-4);
    EXPECT_GT(v.x, 0.0f);
}

TEST(Spiral, SpeedStaysBoundedAfterRadiusCap) {
    // once radius is capped, speed should settle to a constant (pure
    // circular motion at max_radius * omega), not keep growing forever
    float omega = 0.4f, growth = 0.3f, max_r = 10.0f;
    Vec3 v_far = spiral_velocity(1000.0f, omega, growth, max_r);
    Vec3 v_farther = spiral_velocity(2000.0f, omega, growth, max_r);

    float speed_far = std::sqrt(v_far.x * v_far.x + v_far.y * v_far.y);
    float speed_farther = std::sqrt(v_farther.x * v_farther.x + v_farther.y * v_farther.y);

    EXPECT_NEAR(speed_far, max_r * omega, 1e-3);
    EXPECT_NEAR(speed_farther, max_r * omega, 1e-3);
}

TEST(Spiral, SpeedGrowsMonotonicallyBeforeCap) {
    // before the radius cap kicks in, the tangential term (r * omega) should
    // keep increasing with elapsed time, so overall speed should grow too.
    float omega = 0.4f, growth = 0.3f, max_r = 100.0f; // high cap so we stay in growth phase

    float prev_speed = 0.0f;
    for (float t = 0.5f; t <= 5.0f; t += 0.5f) {
        Vec3 v = spiral_velocity(t, omega, growth, max_r);
        float speed = std::sqrt(v.x * v.x + v.y * v.y);
        EXPECT_GT(speed, prev_speed) << "speed did not increase at t=" << t;
        prev_speed = speed;
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}