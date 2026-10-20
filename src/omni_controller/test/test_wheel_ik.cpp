#include "omni_controller/wheel_ik.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <map>

using namespace omni_controller;

// ─── MecanumIK Tests ────────────────────────────────────────────────────────

class MecanumIKTest: public ::testing::Test {
protected:
    void SetUp() override
    {
        ik = std::make_unique<MecanumIK>();
        WheelIKConfig config;
        config.wheel_rad = 0.05;
        config.driveshaft_x = 0.235;
        config.driveshaft_y = 0.188;
        config.mecanum_angle = 135.0;
        std::vector<std::string> names = {"LF", "LH", "RF", "RH"};
        ik->configure(config, names);
    }
    std::unique_ptr<MecanumIK> ik;
};

TEST_F(MecanumIKTest, NumWheels) { EXPECT_EQ(ik->num_wheels(), 4u); }

TEST_F(MecanumIKTest, ZeroInputGivesZeroOutput)
{
    auto vels = ik->inverse(0.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 4u);
    for (auto v : vels)
        EXPECT_NEAR(v, 0.0, 1e-10);
}

TEST_F(MecanumIKTest, ForwardXOnly)
{
    double vx = 1.0;
    auto wheel_vels = ik->inverse(vx, 0.0, 0.0);
    ASSERT_EQ(wheel_vels.size(), 4u);

    // RF and RH should be positive, LF and LH negative (or vice versa based on sign convention)
    // But all should have the same magnitude
    double mag = std::abs(wheel_vels[0]);
    for (auto v : wheel_vels)
        EXPECT_NEAR(std::abs(v), mag, 1e-10);

    // LF (-) and LH (-), RF (+) and RH (+)
    EXPECT_LT(wheel_vels[0], 0.0); // LF: -1/wr * vx
    EXPECT_LT(wheel_vels[1], 0.0); // LH: -1/wr * vx
    EXPECT_GT(wheel_vels[2], 0.0); // RF: +1/wr * vx
    EXPECT_GT(wheel_vels[3], 0.0); // RH: +1/wr * vx
}

TEST_F(MecanumIKTest, IKFKRoundTrip)
{
    // Test that FK(IK(twist)) ≈ twist
    double vx = 0.5, vy = 0.3, omega = 0.2;
    auto wheel_vels = ik->inverse(vx, vy, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], vy, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(MecanumIKTest, IKFKRoundTripPureRotation)
{
    double omega = 1.5;
    auto wheel_vels = ik->inverse(0.0, 0.0, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], 0.0, 1e-10);
    EXPECT_NEAR(recovered[1], 0.0, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(MecanumIKTest, IKFKRoundTripLateralOnly)
{
    double vy = 0.8;
    auto wheel_vels = ik->inverse(0.0, vy, 0.0);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], 0.0, 1e-10);
    EXPECT_NEAR(recovered[1], vy, 1e-10);
    EXPECT_NEAR(recovered[2], 0.0, 1e-10);
}

TEST_F(MecanumIKTest, WrongWheelCountThrows)
{
    MecanumIK bad_ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B"};
    EXPECT_THROW(bad_ik.configure(config, names), std::invalid_argument);
}

// ─── DifferentialIK Tests ───────────────────────────────────────────────────

class DifferentialIKTest: public ::testing::Test {
protected:
    void SetUp() override
    {
        ik = std::make_unique<DifferentialIK>();
        WheelIKConfig config;
        config.wheel_rad = 0.1;
        config.track_width = 0.5;
        std::vector<std::string> names = {"LEFT", "RIGHT"};
        ik->configure(config, names);
    }
    std::unique_ptr<DifferentialIK> ik;
};

TEST_F(DifferentialIKTest, NumWheels) { EXPECT_EQ(ik->num_wheels(), 2u); }

TEST_F(DifferentialIKTest, ZeroInputGivesZeroOutput)
{
    auto vels = ik->inverse(0.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 2u);
    EXPECT_NEAR(vels[0], 0.0, 1e-10);
    EXPECT_NEAR(vels[1], 0.0, 1e-10);
}

TEST_F(DifferentialIKTest, ForwardOnly)
{
    double vx = 1.0;
    auto vels = ik->inverse(vx, 0.0, 0.0);
    // Same ground speed: vx / wheel_rad = 1.0 / 0.1 = 10, right wheel mirrored
    EXPECT_NEAR(vels[0], 10.0, 1e-10);
    EXPECT_NEAR(vels[1], -10.0, 1e-10);
}

TEST_F(DifferentialIKTest, PureRotation)
{
    double omega = 2.0;
    auto vels = ik->inverse(0.0, 0.0, omega);
    // left ground speed = -omega * track/2 = -0.5 → -5.0
    // right ground speed = +0.5, mirrored wheel → -5.0
    EXPECT_NEAR(vels[0], -5.0, 1e-10);
    EXPECT_NEAR(vels[1], -5.0, 1e-10);
}

TEST_F(DifferentialIKTest, IKFKRoundTrip)
{
    double vx = 0.5, omega = 0.3;
    auto wheel_vels = ik->inverse(vx, 0.0, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], 0.0, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(DifferentialIKTest, LateralMotionIgnored)
{
    // vy is ignored in differential drive
    auto vels_no_vy = ik->inverse(1.0, 0.0, 0.0);
    auto vels_with_vy = ik->inverse(1.0, 999.0, 0.0);
    EXPECT_NEAR(vels_no_vy[0], vels_with_vy[0], 1e-10);
    EXPECT_NEAR(vels_no_vy[1], vels_with_vy[1], 1e-10);
}

TEST_F(DifferentialIKTest, WrongWheelCountThrows)
{
    DifferentialIK bad_ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B", "C"};
    EXPECT_THROW(bad_ik.configure(config, names), std::invalid_argument);
}

TEST_F(DifferentialIKTest, SingleWheelPerSideWorks)
{
    // DifferentialIK always takes exactly 2 logical wheels.
    // With multi-wheel-per-side, the controller passes one name per group.
    DifferentialIK fresh;
    WheelIKConfig config;
    config.wheel_rad = 0.1;
    config.track_width = 0.5;
    std::vector<std::string> names = {"left_front", "right_front"};
    EXPECT_NO_THROW(fresh.configure(config, names));

    auto vels = fresh.inverse(1.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 2u);
    EXPECT_NEAR(vels[0], 10.0, 1e-10);
    EXPECT_NEAR(vels[1], -10.0, 1e-10);
}

// ─── TraquadIK Tests ────────────────────────────────────────────────────────

WheelIKConfig traquad_config(double yaw_slip_gain)
{
    WheelIKConfig config;
    config.wheel_rad = 0.028;
    config.track_width = 0.395;
    config.wheel_sides = {1, 1, -1, -1};    // LF, LH, RF, RH
    config.shoulder_signs = {1, -1, -1, 1}; // σ = side · (front +1, hind −1)
    config.shoulder_x = 0.235;
    config.leg_length = 0.19;
    config.yaw_slip_gain = yaw_slip_gain;
    return config;
}

const std::vector<std::string> kTraquadWheels = {"LF_WHEEL", "LH_WHEEL", "RF_WHEEL", "RH_WHEEL"};

TEST(TraquadIK, NumWheels)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.0), kTraquadWheels);
    EXPECT_EQ(ik.num_wheels(), 4u);
}

TEST(TraquadIK, NoSlipGainEqualsDifferential)
{
    // γ = 0: the shoulder angles do not matter
    TraquadIK ik;
    ik.configure(traquad_config(0.0), kTraquadWheels);
    ik.set_leg_state({1.2, -1.2, -1.2, 1.2});
    DifferentialIK diff;
    WheelIKConfig config;
    config.wheel_rad = 0.028;
    config.track_width = 0.395;
    diff.configure(config, {"LEFT", "RIGHT"});

    double vx = 0.2, omega = 0.5;
    auto vels = ik.inverse(vx, 0.0, omega);
    auto ref = diff.inverse(vx, 0.0, omega);
    EXPECT_NEAR(vels[0], ref[0], 1e-10);
    EXPECT_NEAR(vels[1], ref[0], 1e-10);
    EXPECT_NEAR(vels[2], ref[1], 1e-10);
    EXPECT_NEAR(vels[3], ref[1], 1e-10);
}

TEST(TraquadIK, EffectiveTrackGrowsWithTrackSpread)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.2), kTraquadWheels);
    double h = 0.395 / 2.0;

    ik.set_leg_state({1.2, -1.2, -1.2, 1.2});
    double x12 = 0.235 + 0.19 * std::cos(1.2);
    EXPECT_NEAR(ik.effective_half_track(), h * (1.0 + 0.2 * (x12 / h) * (x12 / h)), 1e-12);
    double y12 = ik.effective_half_track();

    ik.set_leg_state({1.6, -1.6, -1.6, 1.6});
    EXPECT_LT(ik.effective_half_track(), y12); // tracks closer together → less slip

    // Pure rotation: every wheel turns at −y_eff·ωz / r
    auto vels = ik.inverse(0.0, 0.0, 0.5);
    for (double v : vels)
        EXPECT_NEAR(v, -ik.effective_half_track() * 0.5 / 0.028, 1e-10);
}

TEST(TraquadIK, ForwardOnlyIgnoresShoulders)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.3), kTraquadWheels);
    ik.set_leg_state({1.0, -1.4, -1.1, 1.3});
    auto vels = ik.inverse(0.2, 0.0, 0.0);
    EXPECT_NEAR(vels[0], 0.2 / 0.028, 1e-10);
    EXPECT_NEAR(vels[1], 0.2 / 0.028, 1e-10);
    EXPECT_NEAR(vels[2], -0.2 / 0.028, 1e-10);
    EXPECT_NEAR(vels[3], -0.2 / 0.028, 1e-10);
}

TEST(TraquadIK, IKFKRoundTrip)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.25), kTraquadWheels);
    ik.set_leg_state({1.0, -1.3, -1.1, 1.2});
    double vx = 0.15, omega = -0.4;
    auto recovered = ik.forward(ik.inverse(vx, 0.0, omega));
    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], 0.0, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST(TraquadIK, InvalidConfigThrows)
{
    TraquadIK ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B"};
    config.wheel_sides = {1, -1};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument); // no shoulder_signs
    config.shoulder_signs = {1, -1};
    EXPECT_NO_THROW(ik.configure(config, names));
    config.yaw_slip_gain = -0.1;
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument);
    config.yaw_slip_gain = 0.0;
    config.wheel_sides = {1, 1};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument); // one side only
    config.wheel_sides = {1, 0};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument);
}

TEST(TraquadIK, WrongLegStateSizeThrows)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.0), kTraquadWheels);
    EXPECT_THROW(ik.set_leg_state({0.0}), std::invalid_argument);
}

TEST(DifferentialMultiWheelConcept, IKOutputFansOutToGroups)
{
    // Demonstrates the multi-wheel pattern:
    // IK computes 2 velocities, controller duplicates to N wheels per side.
    DifferentialIK ik;
    WheelIKConfig config;
    config.wheel_rad = 0.1;
    config.track_width = 0.5;
    // Controller passes one representative name per group
    ik.configure(config, {"LEFT", "RIGHT"});

    double vx = 0.5, omega = 1.0;
    auto vels = ik.inverse(vx, 0.0, omega);

    // Simulate 2 wheels per side: each gets the same velocity
    std::vector<std::vector<std::string>> groups = {
        {"left_front", "left_rear"}, {"right_front", "right_rear"}
    };

    // Fan out: each joint in group g gets vels[g]
    std::map<std::string, double> joint_cmds;
    for (size_t g = 0; g < groups.size(); g++)
        for (const auto& jnt : groups[g])
            joint_cmds[jnt] = vels[g];

    // All left wheels get the same velocity
    EXPECT_DOUBLE_EQ(joint_cmds["left_front"], joint_cmds["left_rear"]);
    // All right wheels get the same velocity
    EXPECT_DOUBLE_EQ(joint_cmds["right_front"], joint_cmds["right_rear"]);
    // Left != right (since omega != 0)
    EXPECT_NE(joint_cmds["left_front"], joint_cmds["right_front"]);

    // Forward kinematics: average each group, recover twist
    double avg_left = (joint_cmds["left_front"] + joint_cmds["left_rear"]) / 2.0;
    double avg_right = (joint_cmds["right_front"] + joint_cmds["right_rear"]) / 2.0;
    auto recovered = ik.forward({avg_left, avg_right});

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

// ─── Factory Tests ──────────────────────────────────────────────────────────

TEST(WheelIKFactory, CreateMecanum)
{
    auto ik = create_wheel_ik("mecanum");
    ASSERT_NE(ik, nullptr);
    EXPECT_EQ(ik->num_wheels(), 4u);
}

TEST(WheelIKFactory, CreateDifferential)
{
    auto ik = create_wheel_ik("differential");
    ASSERT_NE(ik, nullptr);
    EXPECT_EQ(ik->num_wheels(), 2u);
}

TEST(WheelIKFactory, CreateTraquad)
{
    auto ik = create_wheel_ik("traquad");
    ASSERT_NE(ik, nullptr);
    EXPECT_NE(dynamic_cast<TraquadIK*>(ik.get()), nullptr);
}

TEST(WheelIKFactory, CreateNoneReturnsNull)
{
    auto ik = create_wheel_ik("none");
    EXPECT_EQ(ik, nullptr);
}

TEST(WheelIKFactory, UnknownTypeThrows)
{
    EXPECT_THROW(create_wheel_ik("swerve"), std::invalid_argument);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
