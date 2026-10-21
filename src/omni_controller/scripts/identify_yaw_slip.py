#!/usr/bin/env python3
"""Identify traquad.yaw_slip_gain (γ) of the traquad wheel IK.

While the robot turns, the real yaw rate (IMU) is lower than the one the track speeds
would give without slip. The effective half track width that explains it is

    y_true = (v_right − v_left) / (2·ωz_imu)

with the track ground speeds taken from the measured wheel velocities. The IK models it as

    y_eff = h·(1 + γ·(X̄/h)²),   h = track_width/2,   X̄ = mean(shoulder_x + leg_length·cos α_i)

so γ is the least-squares fit of  y_true/h − 1 = γ·(X̄/h)²  over all turning samples.

Usage (robot standing, traquad.yaw_slip_gain: 0.0, joystick as the only twist source):
    python3 identify_yaw_slip.py [--imu-topic /imu_sensor/imu] [--wheel-rad 0.028] ...
Turn in place / along arcs with the joystick, then Ctrl+C to print the fit.
"""

import argparse
import csv
import math
import signal
import statistics
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from pi3hat_moteus_int_msgs.msg import JointsStates

LEGS = ['LF', 'LH', 'RF', 'RH']
SIDE = {'LF': 1, 'LH': 1, 'RF': -1, 'RH': -1}   # κ: +1 left, −1 right
SIGMA = {'LF': 1, 'LH': -1, 'RF': -1, 'RH': 1}  # α = σ·q_HFE


class YawSlipIdentifier(Node):
    def __init__(self, args):
        super().__init__('identify_yaw_slip')
        self.args = args
        self.h = args.track_width / 2.0
        self.wz_imu = None
        self.n_imu = 0
        self.n_joints = 0
        self.n_slow = 0
        self.samples = []  # (alpha_mean, x_mean, y_true, wz_imu, dv)
        self.create_subscription(Imu, args.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointsStates, args.joints_topic, self.joints_cb, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Listening on {args.joints_topic} and {args.imu_topic}. '
            'Turn with the joystick, Ctrl+C to fit.'
        )

    def imu_cb(self, msg):
        self.wz_imu = msg.angular_velocity.z
        self.n_imu += 1

    def joints_cb(self, msg):
        self.n_joints += 1
        if self.wz_imu is None:
            return
        idx = {name: i for i, name in enumerate(msg.name)}
        try:
            q = {leg: msg.position[idx[f'{leg}_HFE']] for leg in LEGS}
            w = {leg: msg.velocity[idx[f'{leg}_WHEEL']] for leg in LEGS}
        except (KeyError, IndexError):
            self.get_logger().warn('HFE/WHEEL joints missing in joints_state', once=True)
            return

        r = self.args.wheel_rad
        # Track ground speeds (forward positive), averaged per side
        v_left = statistics.fmean(SIDE[l] * w[l] * r for l in LEGS if SIDE[l] > 0)
        v_right = statistics.fmean(SIDE[l] * w[l] * r for l in LEGS if SIDE[l] < 0)
        dv = v_right - v_left
        if abs(self.wz_imu) < self.args.min_yaw_rate or abs(dv) < 1e-3:
            self.n_slow += 1
            return

        alphas = [SIGMA[l] * q[l] for l in LEGS]
        x_mean = statistics.fmean(
            self.args.shoulder_x + self.args.leg_length * math.cos(a) for a in alphas
        )
        y_true = dv / (2.0 * self.wz_imu)
        self.samples.append((statistics.fmean(alphas), x_mean, y_true, self.wz_imu, dv))

    def report(self):
        n = len(self.samples)
        print(f'\nReceived: {self.n_imu} IMU msgs on {self.args.imu_topic}, '
              f'{self.n_joints} joints_state msgs on {self.args.joints_topic}')
        if self.n_imu == 0:
            print('No IMU data: is the IMU broadcaster running? '
                  '(ros2 run controller_manager spawner imu_sensor)')
        if self.n_joints == 0:
            print('No joints_state: is omni_controller active?')
        print(f'{n} turning samples (|ωz| > {self.args.min_yaw_rate} rad/s), '
              f'{self.n_slow} discarded as too slow')
        if n < 50:
            print('Too few samples: turn longer (a few seconds per direction and speed).')
            return

        # IMU z pointing down flips every y_true: fix the sign globally
        if statistics.median(s[2] for s in self.samples) < 0:
            print('Note: y_true is negative → IMU z axis points down, sign flipped.')
            self.samples = [(a, x, -y, -wz, dv) for a, x, y, wz, dv in self.samples]

        # Drop outliers (transients, wheel spikes): keep the central 90 %
        ys = sorted(s[2] for s in self.samples)
        lo, hi = ys[int(0.05 * n)], ys[int(0.95 * n) - 1]
        samples = [s for s in self.samples if lo <= s[2] <= hi]

        print('\n alpha [rad]  X̄ [m]   n    y_true [m]  y_true/h')
        bins = {}
        for s in samples:
            bins.setdefault(round(s[0] / 0.05) * 0.05, []).append(s)
        for a in sorted(bins):
            b = bins[a]
            y = statistics.fmean(s[2] for s in b)
            x = statistics.fmean(s[1] for s in b)
            print(f'   {a:5.2f}     {x:6.3f}  {len(b):4d}    {y:7.3f}     {y / self.h:5.2f}')

        # Least squares through the origin: z = γ·u
        u = [(s[1] / self.h) ** 2 for s in samples]
        z = [s[2] / self.h - 1.0 for s in samples]
        gamma = sum(ui * zi for ui, zi in zip(u, z)) / sum(ui * ui for ui in u)
        res = [zi - gamma * ui for ui, zi in zip(u, z)]
        rms = math.sqrt(statistics.fmean(e * e for e in res)) * self.h

        print(f'\nh = track_width/2 = {self.h:.4f} m')
        print(f'γ (traquad.yaw_slip_gain) = {gamma:.4f}')
        print(f'RMS error on y_eff = {rms * 1000:.1f} mm')
        if gamma < 0:
            print('γ < 0: the robot turns MORE than the no-slip model. Check wheel_rad '
                  'and track_width before using it (the controller rejects γ < 0).')
        for a in sorted(bins):
            x = statistics.fmean(s[1] for s in bins[a])
            print(f'  alpha {a:.2f}: y_eff = {self.h * (1 + gamma * (x / self.h) ** 2):.3f} m')
        if len(bins) < 2:
            print('Only one shoulder angle: γ fits, but the X̄ dependence is not validated. '
                  'Repeat with another q_stand to check it.')

        if self.args.csv:
            with open(self.args.csv, 'w', newline='') as f:
                wr = csv.writer(f)
                wr.writerow(['alpha_mean', 'x_mean', 'y_true', 'wz_imu', 'dv'])
                wr.writerows(self.samples)
            print(f'Samples saved to {self.args.csv}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--joints-topic', default='/omni_controller/joints_state')
    p.add_argument('--imu-topic', default='/imu_sensor/imu')
    p.add_argument('--wheel-rad', type=float, default=0.028)
    p.add_argument('--track-width', type=float, default=0.4)
    p.add_argument('--shoulder-x', type=float, default=0.235)
    p.add_argument('--leg-length', type=float, default=0.19)
    p.add_argument('--min-yaw-rate', type=float, default=0.15, help='[rad/s]')
    p.add_argument('--csv', default='', help='optional file for the raw samples')
    args = p.parse_args()

    # Own Ctrl+C handling: on Humble a SIGINT that lands inside take_message() surfaces as
    # a RuntimeError instead of KeyboardInterrupt, which would skip the report
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    node = YawSlipIdentifier(args)
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, RuntimeError) as e:
        if not stop.is_set():
            print(f'Stopped: {e}')
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
