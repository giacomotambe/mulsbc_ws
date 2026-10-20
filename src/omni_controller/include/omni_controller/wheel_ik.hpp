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
    std::vector<int> wheel_sides;    // per wheel: +1 left, −1 right
    std::vector<int> shoulder_signs; // per wheel: σ of its leg, α = σ·q_HFE (α > 0 = leg down)
    double shoulder_x = 0.235;       // |x| of the HFE axes from the base center [m]
    double leg_length = 0.19;        // HFE → ankle [m]
    double yaw_slip_gain = 0.0;      // γ: effective track width growth with the track spread
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

/// Traquad IK: skid-steer on tracked legs, one velocity per wheel.
///
/// Each track hangs from a passive ankle at the end of a leg rotating about the HFE axis
/// and lies flat on the ground. The shoulder angles move the tracks only along x, so
/// without slip they do not change the wheel speeds. They change how much the tracks skid
/// when turning: the farther apart front and hind tracks are, the less the robot turns.
/// This is compensated with an effective half track width
///
///   y_eff = (track_width/2)·(1 + γ·(X̄/(track_width/2))²),   X̄ = mean_i(shoulder_x + leg_length·cos α_i)
///   ω_i   = (κ_i·vx − y_eff·ωz) / r
///
/// with α = σ·q_HFE and κ = wheel side. With γ = 0 this equals DifferentialIK.
class TraquadIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return sides_.size(); }

    void set_leg_state(const std::vector<double>& q) override;

    /// Effective half track width for the current shoulder angles [m]
    double effective_half_track() const { return y_eff_; }

private:
    double wheel_rad_ = 0.05;
    double half_track_ = 0.2;
    std::vector<int> sides_;
    std::vector<int> shoulder_signs_;
    double shoulder_x_ = 0.235;
    double leg_length_ = 0.19;
    double yaw_slip_gain_ = 0.0;
    double y_eff_ = 0.2;
};

/// Factory: create WheelIK by name ("mecanum", "differential", "traquad")
/// Returns nullptr for "none"
std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type);

} // namespace omni_controller

#endif // OMNI_CONTROLLER_WHEEL_IK_HPP
