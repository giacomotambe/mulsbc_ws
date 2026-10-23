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

WheelIKConfig traquad_config(double gamma = 0.0, double k_delta = 0.0, double creep_gain = 0.0)
{
    WheelIKConfig config;
    config.wheel_rad = 0.045;
    config.track_width = 0.4;
    config.wheel_sides = {1, 1, -1, -1};    // LF, LH, RF, RH
    config.shoulder_signs = {1, -1, -1, 1}; // σ = side · (front +1, hind −1)
    config.shoulder_x = 0.235;
    config.leg_length = 0.19;
    config.yaw_slip_gain = gamma;
    config.yaw_posture_gain = k_delta;
    config.creep_gain = creep_gain;
    config.creep_x0 = 0.171;
    config.creep_posture = 0.09;
    return config;
}

WheelIKConfig deficit_config()
{
    WheelIKConfig config = traquad_config(1.35, 0.29, 1.6);
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

std::vector<double> stand_at(double alpha) { return {alpha, -alpha, -alpha, alpha}; }

double xbar(double alpha) { return 0.235 + 0.19 * std::cos(alpha); }

TEST(TraquadIK, NumWheels)
{
    TraquadIK ik;
    ik.configure(traquad_config(), kTraquadWheels);
    EXPECT_EQ(ik.num_wheels(), 4u);
}

TEST(TraquadIK, DefaultsEqualDifferential)
{
    // γ = kδ = 0, no creep, no deficit: the shoulder angles do not matter
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

TEST(TraquadIK, EffectiveTrackFollowsTheShoulders)
{
    TraquadIK ik;
    ik.configure(traquad_config(1.35, 0.29), kTraquadWheels);
    double h = 0.2;
    auto y_model = [h](double alpha, double delta) {
        double r = xbar(alpha) / h;
        return h * (1.0 + 1.35 * r * r) - 0.29 * std::abs(delta);
    };

    // tracks spread out → more skid → larger effective half track (real: 0.96 / 0.56 / 0.40 m)
    ik.set_leg_state(stand_at(1.05));
    double y105 = ik.effective_half_track();
    EXPECT_NEAR(y105, y_model(1.05, 0.0), 1e-12);
    ik.set_leg_state(kStand);
    double y16 = ik.effective_half_track();
    EXPECT_NEAR(y16, y_model(1.6, 0.0), 1e-12);
    ik.set_leg_state(stand_at(1.85));
    EXPECT_LT(ik.effective_half_track(), y16);
    EXPECT_GT(y105, y16);

    // R2 posture, both directions: X̄ about unchanged, y_eff lowered by kδ·|δ|
    for (double delta : {0.3, -0.3}) {
        ik.set_leg_state({1.6 + delta, -1.6 + delta, -1.6 + delta, 1.6 + delta});
        double r = (0.235 + 0.19 * 0.5 * (std::cos(1.6 + delta) + std::cos(1.6 - delta))) / h;
        EXPECT_NEAR(ik.effective_half_track(), h * (1.0 + 1.35 * r * r) - 0.29 * 0.3, 1e-12);
        EXPECT_LT(ik.effective_half_track(), y16);
    }

    // never below the geometric half track
    ik.set_leg_state({3.0, 1.4, 1.4, 3.0});
    EXPECT_NEAR(ik.effective_half_track(), h, 1e-12);

    // Pure rotation: every wheel turns at −y_eff·ωz / r
    ik.set_leg_state(kStand);
    for (double v : ik.inverse(0.0, 0.0, 0.5))
        EXPECT_NEAR(v, -y16 * 0.5 / 0.045, 1e-10);
}

TEST(TraquadIK, Creep)
{
    TraquadIK ik;
    ik.configure(traquad_config(1.35, 0.29, 1.6), kTraquadWheels);
    // d = 1.6·(X̄ − 0.171) − 0.09·|δ|, larger with the tracks spread out, never negative
    ik.set_leg_state(stand_at(1.05));
    EXPECT_NEAR(ik.creep(), 1.6 * (xbar(1.05) - 0.171), 1e-12);
    ik.set_leg_state(stand_at(1.6));
    EXPECT_NEAR(ik.creep(), 1.6 * (xbar(1.6) - 0.171), 1e-12);
    ik.set_leg_state({1.9, -1.3, -1.3, 1.9}); // R2: less creep than standing
    EXPECT_LT(ik.creep(), 1.6 * (xbar(1.6) - 0.171));
    ik.set_leg_state(stand_at(2.2));
    EXPECT_NEAR(ik.creep(), 0.0, 1e-12);

    // turning in place both ways: the tracks back off by d·|ωz| on both sides
    ik.set_leg_state(kStand);
    double d = ik.creep();
    for (double omega : {0.8, -0.8}) {
        auto vels = ik.inverse(0.0, 0.0, omega);
        double v_left = vels[0] * 0.045, v_right = -vels[2] * 0.045;
        EXPECT_NEAR((v_left + v_right) / 2.0, -d * 0.8, 1e-10);
    }
    // straight: no effect
    auto vels = ik.inverse(0.3, 0.0, 0.0);
    EXPECT_NEAR(vels[0], 0.3 / 0.045, 1e-10);
    EXPECT_NEAR(vels[2], -0.3 / 0.045, 1e-10);
}

TEST(TraquadIK, IKFKRoundTrip)
{
    TraquadIK ik;
    ik.configure(traquad_config(1.35, 0.29, 1.6), kTraquadWheels);
    for (const auto& q : {stand_at(1.05), stand_at(1.85), std::vector<double>{1.85, -1.35, -1.35, 1.85}}) {
        ik.set_leg_state(q);
        for (double omega : {-0.4, 0.0, 0.7}) {
            double vx = 0.15;
            auto recovered = ik.forward(ik.inverse(vx, 0.0, omega));
            EXPECT_NEAR(recovered[0], vx, 1e-10);
            EXPECT_NEAR(recovered[1], 0.0, 1e-10);
            EXPECT_NEAR(recovered[2], omega, 1e-10);
        }
    }
}

TEST(TraquadIK, DeficitCompensationOffByDefault)
{
    TraquadIK ik, ref;
    WheelIKConfig config = deficit_config();
    config.deficit_compensation = false;
    ik.configure(config, kTraquadWheels);
    ref.configure(traquad_config(1.35, 0.29, 1.6), kTraquadWheels);
    ik.set_leg_state(kStand);
    ref.set_leg_state(kStand);
    auto a = ik.inverse(0.3, 0.0, 0.4), b = ref.inverse(0.3, 0.0, 0.4);
    for (size_t i = 0; i < 4; i++)
        EXPECT_NEAR(a[i], b[i], 1e-12);
}

TEST(TraquadIK, DeficitCompensation)
{
    // no creep here: turning in place gives the same |ω| and load 1 on both sides
    TraquadIK ik, kin;
    WheelIKConfig config = deficit_config();
    config.creep_gain = 0.0;
    ik.configure(config, kTraquadWheels);
    config.deficit_compensation = false;
    kin.configure(config, kTraquadWheels);
    ik.set_leg_state(kStand);
    kin.set_leg_state(kStand);
    double y = ik.effective_half_track();
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
    double ws = y * 0.8 / 0.045;
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

TEST(TraquadIK, InnerTrackKeptMoving)
{
    WheelIKConfig config = traquad_config(1.35, 0.29, 0.0);
    config.min_track_speed = 1.5;
    config.min_track_hysteresis = 0.5;
    TraquadIK ik, kin;
    ik.configure(config, kTraquadWheels);
    config.min_track_speed = 0.0;
    kin.configure(config, kTraquadWheels);
    ik.set_leg_state(kStand);
    kin.set_leg_state(kStand);
    double y = ik.effective_half_track(), r = 0.045, w_min = 1.5;
    auto belts = [r](const std::vector<double>& w) { return std::array<double, 2>{w[0] * r, -w[2] * r}; };

    // arc to the left with radius y_eff: the kinematics stops the inner (left) track
    double omega = 0.7, vx = y * omega;
    EXPECT_NEAR(kin.inverse(vx, 0.0, omega)[0], 0.0, 1e-9);
    auto b = belts(ik.inverse(vx, 0.0, omega));
    EXPECT_NEAR(b[0], w_min * r, 1e-9);                         // inner track held forward
    EXPECT_NEAR((b[1] - b[0]) / (2.0 * y), omega, 1e-9);        // same yaw rate
    EXPECT_NEAR(ik.forward(ik.inverse(vx, 0.0, omega))[2], omega, 1e-9);

    // slightly tighter: the kinematics asks a little backward, the direction is kept
    b = belts(ik.inverse(vx - 0.3 * r, 0.0, omega));
    EXPECT_NEAR(b[0], w_min * r, 1e-9);
    // clearly backward (past the hysteresis): it reverses
    b = belts(ik.inverse(vx - 1.0 * r, 0.0, omega));
    EXPECT_NEAR(b[0], -w_min * r, 1e-9);
    // and stays backward slightly above zero
    b = belts(ik.inverse(vx + 0.3 * r, 0.0, omega));
    EXPECT_NEAR(b[0], -w_min * r, 1e-9);

    // outside the band, turning in place and straight: unchanged
    for (auto [v, w] : std::vector<std::pair<double, double>>{{0.5, 0.3}, {0.0, 0.7}, {0.4, 0.0}, {-0.5, 0.3}}) {
        auto a1 = ik.inverse(v, 0.0, w), a2 = kin.inverse(v, 0.0, w);
        for (size_t i = 0; i < 4; i++)
            EXPECT_NEAR(a1[i], a2[i], 1e-9);
    }
    // backward arc at radius y_eff: the inner track is held in the direction of travel
    TraquadIK back;
    WheelIKConfig cb = traquad_config(1.35, 0.29, 0.0);
    cb.min_track_speed = 1.5;
    back.configure(cb, kTraquadWheels);
    back.set_leg_state(kStand);
    b = belts(back.inverse(-y * 0.7, 0.0, 0.7));   // backward, turning left: the right track is inner
    EXPECT_NEAR(b[1], -w_min * r, 1e-9);
    EXPECT_NEAR((b[1] - b[0]) / (2.0 * y), 0.7, 1e-9);
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
