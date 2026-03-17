// mujoco-game types tests.
// Tests pure data structures that don't require MuJoCo runtime.

#include "types.h"
#include "test_harness.h"

// ---- types.h tests ----

TEST(constants) {
    CHECK(kMaxJoints == 64);
    CHECK(kMaxContacts == 8);
}

TEST(motor_command_defaults) {
    MotorCommand cmd{};
    CHECK(cmd.forward_velocity == 0.0f);
    CHECK(cmd.angular_velocity == 0.0f);
    CHECK(cmd.approach_drive == 0.0f);
    CHECK(cmd.freeze == 0.0f);
}

TEST(sensory_reading_defaults) {
    SensoryReading sr{};
    CHECK(sr.channel == 0);
    CHECK(sr.activation == 0.0f);
    CHECK(sr.raw_value == 0.0f);
}

TEST(body_state_defaults) {
    BodyState bs{};
    CHECK(bs.n_joints == 0);
    CHECK(bs.n_contacts == 0);
    CHECK(bs.step == 0);
    CHECK(bs.sim_time == 0.0f);
    CHECK(bs.heading == 0.0f);
    for (int i = 0; i < kMaxJoints; ++i) {
        CHECK(bs.joint_angles[i] == 0.0f);
        CHECK(bs.joint_velocities[i] == 0.0f);
    }
    for (int i = 0; i < kMaxContacts; ++i)
        CHECK(bs.contacts[i] == 0.0f);
}

TEST(body_state_joint_storage) {
    BodyState bs{};
    bs.n_joints = 20;
    for (int i = 0; i < 20; ++i) {
        bs.joint_angles[i] = static_cast<float>(i) * 0.1f;
        bs.joint_velocities[i] = static_cast<float>(i) * -0.05f;
    }
    CHECK(std::fabs(bs.joint_angles[10] - 1.0f) < 1e-6f);
    CHECK(std::fabs(bs.joint_velocities[10] - (-0.5f)) < 1e-6f);
}

TEST(body_state_contacts) {
    BodyState bs{};
    bs.n_contacts = 4;
    bs.contacts[0] = 1.0f;  // LF on ground
    bs.contacts[1] = 0.0f;  // RF in air
    bs.contacts[2] = 0.5f;  // LH partial
    bs.contacts[3] = 1.0f;  // RH on ground
    CHECK(bs.contacts[0] == 1.0f);
    CHECK(bs.contacts[1] == 0.0f);
    CHECK(std::fabs(bs.contacts[2] - 0.5f) < 1e-6f);
}

TEST(body_state_velocity) {
    BodyState bs{};
    bs.body_velocity[0] = 80.0f;   // forward mm/s
    bs.body_velocity[1] = -5.0f;   // lateral mm/s
    bs.body_velocity[2] = 0.1f;    // yaw rad/s
    CHECK(bs.body_velocity[0] == 80.0f);
    CHECK(bs.body_velocity[1] == -5.0f);
    CHECK(std::fabs(bs.body_velocity[2] - 0.1f) < 1e-6f);
}

int main() {
    printf("mujoco-game types tests\n");
    return RunAllTests();
}
