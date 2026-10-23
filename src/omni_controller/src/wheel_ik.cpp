#include "omni_controller/wheel_ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace omni_controller {

// ─── MecanumIK ──────────────────────────────────────────────────────────────

void MecanumIK::configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names)
{
    if (wheel_names.size() != 4) {
        throw std::invalid_argument(
            "MecanumIK requires exactly 4 wheels, got " + std::to_string(wheel_names.size())
        );
    }

    double wr = config.wheel_rad;
    double ds_x = config.driveshaft_x;
    double ds_y = config.driveshaft_y;
    double ma = config.mecanum_angle * (M_PI / 180.0);

    double cot_ma = 1.0 / std::tan(ma);
    double rot_term = ds_x - ds_y * cot_ma;

    // Inverse kinematics: [vx, vy, omega] → wheel velocities
    // Wheel order: LF, LH, RF, RH
    base2wheel_[0][0] = -1.0 / wr; // LF
    base2wheel_[0][1] = 1.0 / wr;
    base2wheel_[0][2] = rot_term / wr;

    base2wheel_[1][0] = -1.0 / wr; // LH
    base2wheel_[1][1] = -1.0 / wr;
    base2wheel_[1][2] = rot_term / wr;

    base2wheel_[2][0] = 1.0 / wr; // RF
    base2wheel_[2][1] = 1.0 / wr;
    base2wheel_[2][2] = rot_term / wr;

    base2wheel_[3][0] = 1.0 / wr; // RH
    base2wheel_[3][1] = -1.0 / wr;
    base2wheel_[3][2] = rot_term / wr;

    // Forward kinematics: wheel velocities → [vx, vy, omega]
    //                      LF       LH       RF       RH
    odom_[0][0] = -wr / 4.0;
    odom_[0][1] = -wr / 4.0;
    odom_[0][2] = wr / 4.0;
    odom_[0][3] = wr / 4.0;

    odom_[1][0] = wr / 4.0;
    odom_[1][1] = -wr / 4.0;
    odom_[1][2] = wr / 4.0;
    odom_[1][3] = -wr / 4.0;

    odom_[2][0] = wr / (4.0 * rot_term);
    odom_[2][1] = wr / (4.0 * rot_term);
    odom_[2][2] = wr / (4.0 * rot_term);
    odom_[2][3] = wr / (4.0 * rot_term);
}

std::vector<double> MecanumIK::inverse(double vx, double vy, double omega) const
{
    std::vector<double> wheel_vels(4);
    double base_vel[3] = {vx, vy, omega};
    for (int i = 0; i < 4; i++) {
        double w_v = 0.0;
        for (int j = 0; j < 3; j++) {
            w_v += base2wheel_[i][j] * base_vel[j];
        }
        wheel_vels[i] = w_v;
    }
    return wheel_vels;
}

std::array<double, 3> MecanumIK::forward(const std::vector<double>& wheel_velocities) const
{
    std::array<double, 3> odom_vel = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            odom_vel[i] += odom_[i][j] * wheel_velocities[j];
        }
    }
    return odom_vel;
}

// ─── DifferentialIK ─────────────────────────────────────────────────────────

void DifferentialIK::configure(
    const WheelIKConfig& config, const std::vector<std::string>& wheel_names
)
{
    if (wheel_names.size() != 2) {
        throw std::invalid_argument(
            "DifferentialIK requires exactly 2 wheels, got " + std::to_string(wheel_names.size())
        );
    }
    wheel_rad_ = config.wheel_rad;
    track_width_ = config.track_width;
}

std::vector<double> DifferentialIK::inverse(double vx, double /*vy*/, double omega) const
{
    // wheel_names order: [left, right]
    double v_left = (vx - omega * track_width_ / 2.0) / wheel_rad_;
    double v_right = -(vx + omega * track_width_ / 2.0) / wheel_rad_;
    return {v_left, v_right};
}

std::array<double, 3> DifferentialIK::forward(const std::vector<double>& wheel_velocities) const
{
    double v_left = wheel_velocities[0] * wheel_rad_;
    double v_right = -wheel_velocities[1] * wheel_rad_;
    double vx = (v_left + v_right) / 2.0;
    double omega = (v_right - v_left) / track_width_;
    return {vx, 0.0, omega};
}

// ─── TraquadIK ──────────────────────────────────────────────────────────────

void TraquadIK::configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names)
{
    size_t n = wheel_names.size();
    if (config.wheel_sides.size() != n || config.shoulder_signs.size() != n) {
        throw std::invalid_argument(
            "TraquadIK: wheel_sides and shoulder_signs must have one entry per wheel (" +
            std::to_string(n) + ")"
        );
    }
    bool has_left = false, has_right = false;
    for (size_t i = 0; i < n; i++) {
        int side = config.wheel_sides[i];
        int sigma = config.shoulder_signs[i];
        if ((side != 1 && side != -1) || (sigma != 1 && sigma != -1))
            throw std::invalid_argument("TraquadIK: wheel_sides and shoulder_signs must be +1 or -1");
        has_left |= side == 1;
        has_right |= side == -1;
    }
    if (!has_left || !has_right)
        throw std::invalid_argument("TraquadIK needs at least one wheel per side");
    if (config.yaw_slip_gain < 0.0 || config.yaw_posture_gain < 0.0)
        throw std::invalid_argument("TraquadIK: yaw_slip_gain and yaw_posture_gain must be >= 0");
    if (config.deficit_fade <= 0.0)
        throw std::invalid_argument("TraquadIK: deficit_fade must be > 0");
    if (config.min_track_speed < 0.0 || config.min_track_hysteresis < 0.0)
        throw std::invalid_argument("TraquadIK: min_track_speed and min_track_hysteresis must be >= 0");

    config_ = config;
    half_track_ = config.track_width / 2.0;
    set_leg_state(std::vector<double>(n, 0.0));
}

void TraquadIK::set_leg_state(const std::vector<double>& q)
{
    if (q.size() != config_.wheel_sides.size())
        throw std::invalid_argument("TraquadIK: leg state size must match wheel count");
    // Mean longitudinal distance of the tracks from the base center, and the yaw posture:
    // common offset of the HFE joints (the stand pose cancels, σ sums to 0)
    double x_mean = 0.0, delta = 0.0;
    for (size_t i = 0; i < q.size(); i++) {
        x_mean += config_.shoulder_x + config_.leg_length * std::cos(config_.shoulder_signs[i] * q[i]);
        delta += q[i];
    }
    x_mean /= static_cast<double>(q.size());
    delta = std::abs(delta) / static_cast<double>(q.size());
    double ratio = x_mean / half_track_;
    // never below the geometric half track (no skid at all), no backward creep
    y_eff_ = std::max(
        half_track_ * (1.0 + config_.yaw_slip_gain * ratio * ratio) - config_.yaw_posture_gain * delta,
        half_track_
    );
    creep_ = std::max(
        config_.creep_gain * (x_mean - config_.creep_x0) - config_.creep_posture * delta, 0.0
    );
}

double TraquadIK::deficit(double omega, double load, bool outer) const
{
    double d = config_.deficit_base + config_.deficit_speed * std::abs(omega) +
               load * (config_.deficit_turn + (outer ? 1.0 : -1.0) * config_.deficit_side);
    // fade near zero: no step of the command when a track reverses
    return std::max(d, 0.0) * std::min(std::abs(omega) / config_.deficit_fade, 1.0);
}

std::vector<double> TraquadIK::inverse(double vx, double /*vy*/, double omega) const
{
    // Belt speeds of the two sides (forward positive), forward drift while turning removed
    double v = vx - creep_ * std::abs(omega);
    double v_left = v - y_eff_ * omega;
    double v_right = v + y_eff_ * omega;
    if (config_.min_track_speed > 0.0)
        keep_inner_track_moving(vx, omega, v_left, v_right);
    double sum = std::abs(v_left + v_right), diff = std::abs(v_right - v_left);
    double load = sum + diff > 1e-9 ? 1.0 - sum / (sum + diff) : 0.0;

    // Right wheels spin the other way for forward motion (side = −1)
    const auto& sides = config_.wheel_sides;
    std::vector<double> wheel_vels(sides.size());
    for (size_t i = 0; i < sides.size(); i++) {
        double own = sides[i] > 0 ? v_left : v_right;
        double other = sides[i] > 0 ? v_right : v_left;
        double w = own / config_.wheel_rad;
        if (config_.deficit_compensation) {
            bool outer = (own - other) * own > 0.0;
            w += std::copysign(deficit(w, load, outer), w);
        }
        wheel_vels[i] = sides[i] * w;
    }
    return wheel_vels;
}

void TraquadIK::keep_inner_track_moving(double vx, double omega, double& v_left, double& v_right) const
{
    const double v_min = config_.min_track_speed * config_.wheel_rad;
    // arcs only: turning in place or creeping along is left alone
    if (std::abs(vx) < v_min || std::abs(omega) < 1e-3) {
        inner_sign_ = 0;
        return;
    }
    bool left_inner = std::abs(v_left) < std::abs(v_right);
    double& inner = left_inner ? v_left : v_right;
    double& outer = left_inner ? v_right : v_left;
    if (std::abs(inner) >= v_min) {
        inner_sign_ = 0;
        return;
    }
    // direction: the one the kinematics asks when entering the band (the direction of travel
    // when it asks about zero), kept until it asks clearly for the other one (no chattering)
    const double h = config_.min_track_hysteresis * config_.wheel_rad;
    if (inner_sign_ == 0)
        inner_sign_ = std::abs(inner) > h ? (inner > 0.0 ? 1 : -1) : (vx > 0.0 ? 1 : -1);
    else if (inner > h)
        inner_sign_ = 1;
    else if (inner < -h)
        inner_sign_ = -1;
    const double target = inner_sign_ * v_min;
    outer += target - inner;   // V_R − V_L unchanged: same yaw rate
    inner = target;
}

std::array<double, 3> TraquadIK::forward(const std::vector<double>& wheel_velocities) const
{
    // Ground speed of each track, averaged per side
    double v_left = 0.0, v_right = 0.0;
    int n_left = 0, n_right = 0;
    const auto& sides = config_.wheel_sides;
    for (size_t i = 0; i < sides.size(); i++) {
        double v = sides[i] * wheel_velocities[i] * config_.wheel_rad;
        if (sides[i] > 0) {
            v_left += v;
            n_left++;
        } else {
            v_right += v;
            n_right++;
        }
    }
    v_left /= n_left;
    v_right /= n_right;
    double omega = (v_right - v_left) / (2.0 * y_eff_);
    double vx = (v_left + v_right) / 2.0 + creep_ * std::abs(omega);
    return {vx, 0.0, omega};
}

// ─── Factory ────────────────────────────────────────────────────────────────

std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type)
{
    if (type == "mecanum") {
        return std::make_unique<MecanumIK>();
    } else if (type == "differential") {
        return std::make_unique<DifferentialIK>();
    } else if (type == "traquad") {
        return std::make_unique<TraquadIK>();
    } else if (type == "none") {
        return nullptr;
    }
    throw std::invalid_argument(
        "Unknown wheel IK type: '" + type +
        "'. Use 'mecanum', 'differential', 'traquad', or 'none'."
    );
}

} // namespace omni_controller
