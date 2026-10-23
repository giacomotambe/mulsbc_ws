#ifndef OMNI_CONTROLLER_WHEEL_IK_HPP
#define OMNI_CONTROLLER_WHEEL_IK_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace omni_controller {

struct WheelIKConfig {
    double wheel_rad = 0.05;
    double driveshaft_x = 0.235;
    double driveshaft_y = 0.188;
    double mecanum_angle = 135.0; // degrees
    double track_width = 0.4;

    // Traquad only (see TraquadIK)
    std::vector<int> wheel_sides;     // per wheel: +1 left, −1 right
    std::vector<int> shoulder_signs;  // per wheel: σ of its leg, α = σ·q_HFE (α > 0 = leg down)
    double shoulder_x = 0.235;        // |x| of the HFE axes from the base center [m]
    double leg_length = 0.19;         // HFE → ankle [m]
    double yaw_slip_gain = 0.0;       // γ: effective half track growth with the track spread X̄
    double yaw_posture_gain = 0.0;    // kδ: y_eff decrease per rad of common HFE offset [m/rad]
    // Forward creep while turning, d = max(gain·(X̄ − x0) − posture·|δ|, 0) [m/rad]
    double creep_gain = 0.0;          // [1/rad]
    double creep_x0 = 0.0;            // [m]
    double creep_posture = 0.0;       // [m/rad²]
    // Arcs: the inner track never stands still while the robot drives and turns (a standing
    // track pivots and the ground can roll it over its passive ankle). Below this speed it is
    // held at ±min_track_speed and the outer track takes the difference (same yaw rate)
    double min_track_speed = 0.0;     // [rad/s] 0 = off
    double min_track_hysteresis = 0.5; // [rad/s] the inner track reverses only past ∓ this
    // Track deficit compensation (B): Δ = base + speed·|ω| + load·(turn ± side) [rad/s]
    bool deficit_compensation = false;
    double deficit_base = 0.0;
    double deficit_speed = 0.0;
    double deficit_turn = 0.0;
    double deficit_side = 0.0;        // + outer track of the turn, − inner one
    double deficit_fade = 1.0;        // [rad/s] Δ scaled by |ω| / fade below it
};

class WheelIK {
public:
    virtual ~WheelIK() = default;

    virtual void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) = 0;

    /// Compute wheel velocities from base twist (vx, vy, omega)
    virtual std::vector<double> inverse(double vx, double vy, double omega) const = 0;

    /// Compute base twist (vx, vy, omega) from wheel velocities
    virtual std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const = 0;

    virtual size_t num_wheels() const = 0;

    /// Shoulder (HFE) angle of the leg carrying each wheel. Ignored by default.
    virtual void set_leg_state(const std::vector<double>& /*q*/) {}
};

/// 4-wheel mecanum IK (ported from omni_vel_controller)
class MecanumIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return 4; }

private:
    double base2wheel_[4][3] = {};
    double odom_[3][4] = {};
};

/// 2-wheel differential IK
class DifferentialIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return 2; }

private:
    double wheel_rad_ = 0.05;
    double track_width_ = 0.4;
};

/// Traquad IK: skid-steer on tracked legs, one velocity per wheel, with the effective track
/// width and the creep measured with the Qualisys (2026-10-22: joystick runs, R2 and the
/// t1-t3 missions at shoulder angles 1.05-1.85).
///
/// A. Kinematics. The tracks skid when turning, so the robot turns as a differential drive
/// with an effective half track y_eff. It grows with the longitudinal spread of the tracks X̄
/// (the farther apart front and hind tracks are, the more they skid) and the yaw posture of
/// the hips lowers it (common offset δ of the four HFE joints: R2, the compliant IK mode and
/// the passive yield). The robot also creeps forward while turning, more with the tracks
/// spread out:
///
///   X̄     = mean_i(shoulder_x + leg_length·cos α_i),   α = σ·q_HFE,   δ = mean_i q_HFE,i
///   y_eff = max(h·(1 + γ·(X̄/h)²) − kδ·|δ|, h),        h = track_width/2
///   d     = max(gain·(X̄ − x0) − posture·|δ|, 0)
///   vx'   = vx − d·|ωz|
///   V_L   = vx' − y_eff·ωz,   V_R = vx' + y_eff·ωz,   ω_i = κ_i·V_side / r
///
/// with κ = wheel side (+1 left, −1 right), all from the measured shoulder angles. forward()
/// inverts it from the measured speeds. With γ = kδ = 0 and no creep this equals
/// DifferentialIK.
///
/// Arcs with the inner track at about zero (radius ≈ y_eff): with min_track_speed > 0, while
/// the robot drives and turns, the inner track is held at ±min_track_speed (direction kept until
/// the kinematics asks for the other one past the hysteresis) and the outer track changes by
/// the same amount, so the yaw rate stays exact and the forward speed changes a little.
///
/// B. Track deficit (optional). The track drives have a velocity loop with Kd only: each
/// track runs slower than commanded by Δ, larger when the tracks skid. Each command is raised
/// along its direction by
///
///   Δ = base + speed·|ω| + load·(turn ± side),   load = 1 − |ΣV| / (|ΣV| + |V_R − V_L|)
///
/// (+ for the outer track of the turn, − for the inner one; load 0 straight, 1 turning in
/// place), faded to 0 below |ω| = fade. forward() does not include it (measured speeds).
class TraquadIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return config_.wheel_sides.size(); }

    void set_leg_state(const std::vector<double>& q) override;

    /// Effective half track width for the current shoulder angles [m]
    double effective_half_track() const { return y_eff_; }
    /// Forward creep per rad/s of yaw rate for the current shoulder angles [m/rad]
    double creep() const { return creep_; }

private:
    double deficit(double omega, double load, bool outer) const;
    void keep_inner_track_moving(double vx, double omega, double& v_left, double& v_right) const;

    WheelIKConfig config_;
    double half_track_ = 0.2;
    double y_eff_ = 0.2;
    double creep_ = 0.0;
    mutable int inner_sign_ = 0;   // direction held by the inner track in the band (0 = outside)
};

/// Factory: create WheelIK by name ("mecanum", "differential", "traquad")
/// Returns nullptr for "none"
std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type);

} // namespace omni_controller

#endif // OMNI_CONTROLLER_WHEEL_IK_HPP
