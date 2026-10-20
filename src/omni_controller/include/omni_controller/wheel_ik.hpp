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

    // Differential only (tracked legs, see DifferentialIK).
    // Per-wheel side: +1 left, −1 right. Empty → exactly two wheels [left, right].
    std::vector<int> wheel_sides;
    // Per-wheel sign σ of the leg carrying the track (α = σ·q_HFE, α > 0 = leg towards
    // the ground), 0 = no leg. Empty → no shoulder compensation.
    std::vector<int> shoulder_signs;
    double leg_length = 0.0;        // HFE → ankle [m]; 0 disables shoulder compensation
    double shoulder_coupling = 1.0; // c: 1 = wheel encoder relative to the leg, 0 = to the body
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

    /// Shoulder (HFE) angle and velocity of the leg carrying each wheel. Ignored by default.
    virtual void set_leg_state(const std::vector<double>& /*q*/, const std::vector<double>& /*qd*/) {}
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

/// Differential (skid-steer) IK, one velocity per wheel.
///
/// Tracked legs (traquad): each track hangs from a passive ankle at the end of a leg
/// rotating about the HFE axis and lies flat on the ground. With α = σ·q_HFE:
///
///   ω_i = (κ_i·vx − (track_width/2)·ωz) / r − (leg_length·sin α_i / r + c)·q̇_i
///
/// κ_i = wheel side. With still shoulders (q̇ = 0) this is plain differential drive.
class DifferentialIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return sides_.size(); }

    void set_leg_state(const std::vector<double>& q, const std::vector<double>& qd) override;

private:
    /// Shoulder-motion term of wheel i [rad/s]: (leg_length·sin α / r + c)·q̇
    double shoulder_term(size_t i) const;

    double wheel_rad_ = 0.05;
    double track_width_ = 0.4;
    std::vector<int> sides_ = {1, -1};
    std::vector<int> shoulder_signs_ = {0, 0};
    double leg_length_ = 0.0;
    double shoulder_coupling_ = 1.0;
    std::vector<double> q_ = {0.0, 0.0}, qd_ = {0.0, 0.0};
};

/// Factory: create WheelIK by name ("mecanum", "differential")
/// Returns nullptr for "none"
std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type);

} // namespace omni_controller

#endif // OMNI_CONTROLLER_WHEEL_IK_HPP
