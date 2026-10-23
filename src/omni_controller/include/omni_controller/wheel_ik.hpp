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
    double yaw_half_track = 0.0;      // y0: effective half track standing [m] (0 = track_width/2)
    double yaw_posture_gain = 0.0;    // kδ: y_eff decrease per rad of common HFE offset [m/rad]
    double spin_forward_drift = 0.0;  // d: forward drift per rad/s of yaw rate [m/rad]
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
/// width and drift measured with the Qualisys (study of 2026-10-22, three runs, standing).
///
/// A. Kinematics. The tracks skid when turning, so the robot turns as a differential drive
/// with an effective half track y_eff, which the yaw posture of the hips lowers (common
/// offset δ of the four HFE joints, the same for R2, the compliant IK mode and the passive
/// yield), and it creeps forward while turning:
///
///   y_eff = max(y0 − kδ·|δ|, track_width/2),   δ = mean_i q_HFE,i (measured)
///   vx'   = vx − d·|ωz|
///   V_L   = vx' − y_eff·ωz,   V_R = vx' + y_eff·ωz,   ω_i = κ_i·V_side / r
///
/// with κ = wheel side (+1 left, −1 right). forward() inverts it from the measured speeds.
/// With y0 = kδ = d = 0 this equals DifferentialIK.
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

    /// Effective half track width for the current yaw posture [m]
    double effective_half_track() const { return y_eff_; }

private:
    double deficit(double omega, double load, bool outer) const;

    WheelIKConfig config_;
    double half_track_ = 0.2;
    double y0_ = 0.2;
    double y_eff_ = 0.2;
};

/// Factory: create WheelIK by name ("mecanum", "differential", "traquad")
/// Returns nullptr for "none"
std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type);

} // namespace omni_controller

#endif // OMNI_CONTROLLER_WHEEL_IK_HPP
