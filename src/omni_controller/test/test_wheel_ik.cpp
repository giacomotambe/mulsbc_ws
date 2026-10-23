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

WheelIKConfig traquad_config(double y0 = 0.0, double k_delta = 0.0, double drift = 0.0)
{
    WheelIKConfig config;
    config.wheel_rad = 0.045;
    config.track_width = 0.4;
    config.wheel_sides = {1, 1, -1, -1}; // LF, LH, RF, RH
    config.yaw_half_track = y0;
    config.yaw_posture_gain = k_delta;
    config.spin_forward_drift = drift;
    return config;
}

WheelIKConfig deficit_config()
{
    WheelIKConfig config = traquad_config(0.556, 0.302, 0.066);
    config.deficit_compensation = true;
    config.deficit_base = 1.215;
    config.deficit_speed = 0.025;
    config.deficit_turn = -0.221;
    config.deficit_side = 0.939;
    config.deficit_fade = 1.0;
    return config;
}

const std::vector<std::string> kTraquadWheels = {"LF_WHEEL", "LH_WHEEL", "RF_WHEEL", "RH_WHEEL"};
const std::vector<double> kStand = {1.6, -1.6, -1.6, 1.6};

TEST(TraquadIK, NumWheels)
{
    TraquadIK ik;
    ik.configure(traquad_config(), kTraquadWheels);
    EXPECT_EQ(ik.num_wheels(), 4u);
}

TEST(TraquadIK, DefaultsEqualDifferential)
{
    // y0 = kδ = d = 0, no deficit: the shoulder angles do not matter
    TraquadIK ik;
    ik.configure(traquad_config(), kTraquadWheels);
    ik.set_leg_state({1.9, -1.3, -1.3, 1.9});
    DifferentialIK diff;
    WheelIKConfig config;
    config.wheel_rad = 0.045;
    config.track_width = 0.4;
    diff.configure(config, {"LEFT", "RIGHT"});

    double vx = 0.2, omega = 0.5;
    auto vels = ik.inverse(vx, 0.0, omega);
    auto ref = diff.inverse(vx, 0.0, omega);
    EXPECT_NEAR(vels[0], ref[0], 1e-10);
    EXPECT_NEAR(vels[1], ref[0], 1e-10);
    EXPECT_NEAR(vels[2], ref[1], 1e-10);
    EXPECT_NEAR(vels[3], ref[1], 1e-10);
}

TEST(TraquadIK, YawPostureLowersEffectiveTrack)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.556, 0.302), kTraquadWheels);
    ik.set_leg_state(kStand); // δ = 0: the stand pose cancels
    EXPECT_NEAR(ik.effective_half_track(), 0.556, 1e-12);

    // R2 posture, both directions: δ = ±0.3 (q = σ·1.6 + δ)
    ik.set_leg_state({1.9, -1.3, -1.3, 1.9});
    EXPECT_NEAR(ik.effective_half_track(), 0.556 - 0.302 * 0.3, 1e-12);
    ik.set_leg_state({1.3, -1.9, -1.9, 1.3});
    EXPECT_NEAR(ik.effective_half_track(), 0.556 - 0.302 * 0.3, 1e-12);

    // never below the geometric half track
    ik.set_leg_state({3.0, 1.4, 1.4, 3.0});
    EXPECT_NEAR(ik.effective_half_track(), 0.2, 1e-12);

    // Pure rotation: every wheel turns at −y_eff·ωz / r
    ik.set_leg_state(kStand);
    for (double v : ik.inverse(0.0, 0.0, 0.5))
        EXPECT_NEAR(v, -0.556 * 0.5 / 0.045, 1e-10);
}

TEST(TraquadIK, SpinForwardDrift)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.556, 0.302, 0.066), kTraquadWheels);
    ik.set_leg_state(kStand);
    // turning in place both ways: the tracks back off by d·|ωz| on both sides
    for (double omega : {0.8, -0.8}) {
        auto vels = ik.inverse(0.0, 0.0, omega);
        double v_left = vels[0] * 0.045, v_right = -vels[2] * 0.045;
        EXPECT_NEAR((v_left + v_right) / 2.0, -0.066 * 0.8, 1e-10);
    }
    // straight: no effect
    auto vels = ik.inverse(0.3, 0.0, 0.0);
    EXPECT_NEAR(vels[0], 0.3 / 0.045, 1e-10);
    EXPECT_NEAR(vels[2], -0.3 / 0.045, 1e-10);
}

TEST(TraquadIK, IKFKRoundTrip)
{
    TraquadIK ik;
    ik.configure(traquad_config(0.556, 0.302, 0.066), kTraquadWheels);
    ik.set_leg_state({1.85, -1.35, -1.35, 1.85});
    for (double omega : {-0.4, 0.0, 0.7}) {
        double vx = 0.15;
        auto recovered = ik.forward(ik.inverse(vx, 0.0, omega));
        EXPECT_NEAR(recovered[0], vx, 1e-10);
        EXPECT_NEAR(recovered[1], 0.0, 1e-10);
        EXPECT_NEAR(recovered[2], omega, 1e-10);
    }
}

TEST(TraquadIK, DeficitCompensationOffByDefault)
{
    TraquadIK ik, ref;
    WheelIKConfig config = deficit_config();
    config.deficit_compensation = false;
    ik.configure(config, kTraquadWheels);
    ref.configure(traquad_config(0.556, 0.302, 0.066), kTraquadWheels);
    ik.set_leg_state(kStand);
    ref.set_leg_state(kStand);
    auto a = ik.inverse(0.3, 0.0, 0.4), b = ref.inverse(0.3, 0.0, 0.4);
    for (size_t i = 0; i < 4; i++)
        EXPECT_NEAR(a[i], b[i], 1e-12);
}

TEST(TraquadIK, DeficitCompensation)
{
    // no forward drift here: turning in place gives the same |ω| and load 1 on both sides
    TraquadIK ik, kin;
    WheelIKConfig config = deficit_config();
    config.spin_forward_drift = 0.0;
    ik.configure(config, kTraquadWheels);
    config.deficit_compensation = false;
    kin.configure(config, kTraquadWheels);
    ik.set_leg_state(kStand);
    kin.set_leg_state(kStand);
    // forward-signed track speed added by the compensation
    auto added = [&](double vx, double omega, size_t i) {
        double side = i < 2 ? 1.0 : -1.0;
        return side * (ik.inverse(vx, 0.0, omega)[i] - kin.inverse(vx, 0.0, omega)[i]);
    };

    // straight (load 0), both directions: + base + speed·|ω| along the motion
    double w = 0.3 / 0.045;
    EXPECT_NEAR(added(0.3, 0.0, 0), 1.215 + 0.025 * w, 1e-10);
    EXPECT_NEAR(added(0.3, 0.0, 2), 1.215 + 0.025 * w, 1e-10);
    EXPECT_NEAR(added(-0.3, 0.0, 0), -(1.215 + 0.025 * w), 1e-10);

    // turning in place (load 1): both tracks are outer, the largest deficit
    double ws = 0.556 * 0.8 / 0.045;
    double spin = 1.215 + 0.025 * ws - 0.221 + 0.939;
    EXPECT_NEAR(added(0.0, 0.8, 0), -spin, 1e-10); // left track backwards
    EXPECT_NEAR(added(0.0, 0.8, 2), spin, 1e-10);  // right track forwards

    // arc to the left: the right (outer) track gets more than the left (inner) one
    EXPECT_GT(added(0.3, 0.3, 2), added(0.3, 0.3, 0));
    EXPECT_GT(added(0.3, 0.3, 0), 0.0);

    // faded to 0 at standstill, no step near zero
    for (double v : ik.inverse(0.0, 0.0, 0.0))
        EXPECT_NEAR(v, 0.0, 1e-12);
    EXPECT_NEAR(added(0.0225, 0.0, 0), 0.5 * (1.215 + 0.025 * 0.5), 1e-10); // |ω| 0.5 = fade / 2

    // forward() works on the measured speeds: no compensation
    auto fk = ik.forward(kin.inverse(0.3, 0.0, 0.4));
    EXPECT_NEAR(fk[0], 0.3, 1e-10);
    EXPECT_NEAR(fk[2], 0.4, 1e-10);
}

TEST(TraquadIK, InvalidConfigThrows)
{
    TraquadIK ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B"};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument); // no wheel_sides
    config.wheel_sides = {1, -1};
    EXPECT_NO_THROW(ik.configure(config, names));
    config.yaw_posture_gain = -0.1;
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument);
    config.yaw_posture_gain = 0.0;
    config.deficit_fade = 0.0;
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument);
    config.deficit_fade = 1.0;
    config.wheel_sides = {1, 1};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument); // one side only
    config.wheel_sides = {1, 0};
    EXPECT_THROW(ik.configure(config, names), std::invalid_argument);
}

TEST(TraquadIK, WrongLegStateSizeThrows)
{
    TraquadIK ik;
    ik.configure(traquad_config(), kTraquadWheels);
    EXPECT_THROW(ik.set_leg_state({0.0}), std::invalid_argument);
}
