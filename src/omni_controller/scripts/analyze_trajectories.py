#!/usr/bin/env python3
"""Identification missions t1-t3 (NUC joystick, START): measured motion of the robot from the Qualisys, per segment.

Inputs: the robot bag (moteus_pi3hat_interface.launch.py record_bag:=true) and the Qualisys bag (/rigid_bodies). The Qualisys bag can come
from another computer: its clock is aligned to the robot by cross-correlating the yaw rate of the IMU with the yaw
rate of the rigid body, separately on the first and the last part of the mission (the sync turns), so a clock drift
is corrected too. If /rigid_bodies is in the robot bag, give the same bag twice.

Rigid body: define it in QTM with the origin at the centre of the base and x forward; --rb-offset corrects it
otherwise (position of the base centre in the rigid body frame, and yaw of the robot x axis in it). The heading
offset is also estimated from the straight runs of T2 and printed as a check.

Per steady segment (hips still; the first SKIP seconds after a change are left out) it computes, with V the belt
speeds (joint speed x r_s, forward positive; L/R = mean of front and hind) and h = track_width / 2:
  vx, vy, wz       body twist from the Qualisys (base frame), wz also from the IMU
  V_cmd / V        commanded and measured belt speeds per side: their ratio is the speed loop
  y_true           (V_R - V_L) / (2 wz), measured V: lateral skid only;  h / y_true = 1 for no skid
  wz / wz_cmd      against the ideal unicycle with the commanded speeds: speed loop and skid together
  y_L, y_R, x_ICR  ICRs of the tracks and of the body (Martinez): (vx - V_L) / wz, (vx - V_R) / wz, -vy / wz
  vx / V           straight runs: effective sprocket radius on the ground
and fits gamma of the traquad IK, y_true / h - 1 = gamma (X_mean / h)^2, X_mean = mean(shoulder_x + l cos alpha_i).

Outputs: the table on stdout, <out>_segments.csv (one row per segment) and <out>_timeseries.csv (100 Hz on the robot
clock: commands, joints, IMU and the aligned Qualisys pose and twist), the input of the simulation replay.

usage: python3 analyze_trajectories.py --bag ROBOT_BAG --mocap QUALISYS_BAG [--body NAME] [--out PREFIX]
  needs numpy and the python package rosbags (pip install rosbags); no ROS workspace.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

LEGS = ['LF', 'LH', 'RF', 'RH']
SIDE = {'LF': 1, 'LH': 1, 'RF': -1, 'RH': -1}               # +1 left
SIGMA = {'LF': 1.0, 'LH': -1.0, 'RF': -1.0, 'RH': 1.0}      # alpha = sigma * q_HFE
WHEEL_SIGN = {'LF': 1.0, 'LH': 1.0, 'RF': -1.0, 'RH': -1.0}  # joint speed sign for forward belt motion
R_S, SHOULDER_X, LEG_L = 0.028, 0.235, 0.19
SKIP = 1.5      # [s] transient left out at the start of a segment
FS = 100.0      # [Hz] time series
SEGMENT_TOPIC = '/omni_mulinex_joystick/identification/segment'   # markers of the NUC joystick

JOINTS_STATES_MSG = """std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
float64[] current
float64[] temperature
float64[] sec_enc_pos
float64[] sec_enc_vel
float64[] voltage
float64[] power
"""
JOINTS_COMMAND_MSG = """std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
float64[] kp_scale
float64[] kd_scale
"""
MARKER_MSG = """int8 USE_NAME=0
int8 USE_INDEX=1
int8 USE_BOTH=2
int8 id_type
int32 marker_index
string marker_name
geometry_msgs/Point translation
"""


def typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    types.update(get_types_from_msg(JOINTS_STATES_MSG, 'pi3hat_moteus_int_msgs/msg/JointsStates'))
    types.update(get_types_from_msg(JOINTS_COMMAND_MSG, 'pi3hat_moteus_int_msgs/msg/JointsCommand'))
    for pkg in ('mocap_msgs', 'mocap4r2_msgs'):     # the driver in qualisys_docker uses mocap_msgs
        types.update(get_types_from_msg(MARKER_MSG, f'{pkg}/msg/Marker'))
        types.update(get_types_from_msg(f'string rigid_body_name\n{pkg}/Marker[] markers\ngeometry_msgs/Pose pose\n',
                                        f'{pkg}/msg/RigidBody'))
        types.update(get_types_from_msg(f'std_msgs/Header header\nuint32 frame_number\n{pkg}/RigidBody[] rigidbodies\n',
                                        f'{pkg}/msg/RigidBodies'))
    store.register(types)
    return store


def read(path, topics):
    """{topic: [(t_record [s], msg)]} with the record time of the bag."""
    from rosbags.highlevel import AnyReader
    out = {t: [] for t in topics}
    with AnyReader([Path(path)], default_typestore=typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        for conn, stamp, raw in reader.messages(connections=conns):
            out[conn.topic].append((stamp * 1e-9, reader.deserialize(raw, conn.msgtype)))
    return out


def load_robot(path):
    d = read(path, ['/omni_controller/joints_state', '/omni_controller/debug/joints_command', '/imu_sensor/imu',
                    SEGMENT_TOPIC])
    js = d['/omni_controller/joints_state']
    if not js:
        sys.exit(f'{path}: no /omni_controller/joints_state')
    idx = {n: i for i, n in enumerate(js[0][1].name)}
    t = np.array([s for s, _ in js])
    q = {k: np.array([m.position[idx[f'{k}_HFE']] for _, m in js]) for k in LEGS}
    w = {k: np.array([m.velocity[idx[f'{k}_WHEEL']] for _, m in js]) for k in LEGS}
    r = {'t': t, **{f'alpha_{k}': SIGMA[k] * q[k] for k in LEGS}, **{f'V_{k}': WHEEL_SIGN[k] * w[k] * R_S for k in LEGS}}
    jc = d['/omni_controller/debug/joints_command']
    if jc:
        tc = np.array([s for s, _ in jc])
        cidx = {n: i for i, n in enumerate(jc[0][1].name)}
        for k in LEGS:
            v = np.array([m.velocity[cidx[f'{k}_WHEEL']] for _, m in jc]) * WHEEL_SIGN[k] * R_S
            r[f'Vcmd_{k}'] = np.interp(t, tc, v)
    imu = d['/imu_sensor/imu']
    if imu:
        r['imu_t'] = np.array([s for s, _ in imu])
        r['imu_wz'] = np.array([m.angular_velocity.z for _, m in imu])
    r['segments'] = [(s, json.loads(m.data)) for s, m in d[SEGMENT_TOPIC]]
    return r


def quat_yaw(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def load_mocap(path, body):
    d = read(path, ['/rigid_bodies'])['/rigid_bodies']
    if not d:
        sys.exit(f'{path}: no /rigid_bodies')
    names = sorted({b.rigid_body_name for _, m in d for b in m.rigidbodies})
    if body is None:
        if len(names) != 1:
            sys.exit(f'rigid bodies {names}: choose one with --body')
        body = names[0]
    rows = []
    for s, m in d:
        for b in m.rigidbodies:
            p = b.pose.position
            if b.rigid_body_name == body and np.isfinite([p.x, p.y, b.pose.orientation.w]).all():
                rows.append((s, p.x, p.y, p.z, quat_yaw(b.pose.orientation)))
    if len(rows) < 20:
        sys.exit(f'rigid body {body}: {len(rows)} valid poses')
    a = np.array(rows)
    print(f'Qualisys: {body}, {len(a)} poses, {1 / np.median(np.diff(a[:, 0])):.0f} Hz '
          f'(rigid bodies in the bag: {names})')
    return {'t': a[:, 0], 'x': a[:, 1], 'y': a[:, 2], 'z': a[:, 3], 'yaw': np.unwrap(a[:, 4])}


def apply_rb_offset(mc, dx, dy, dyaw):
    """Pose of the base centre (dx, dy in the rigid body frame) with the robot heading (rigid body yaw + dyaw)."""
    c, s = np.cos(mc['yaw']), np.sin(mc['yaw'])
    mc['x'], mc['y'] = mc['x'] + c * dx - s * dy, mc['y'] + s * dx + c * dy
    mc['yaw'] = mc['yaw'] + dyaw
    return mc


SMOOTH = 0.1    # [s] window of the Qualisys derivative (set by --smooth)


def deriv(t, x):
    """Derivative as the slope of a local least-squares line over SMOOTH seconds (Savitzky-Golay, order 1), on a
    uniform grid at the median rate of t: at 200 Hz central differences would turn sub-millimetre noise into cm/s.
    Below 3 samples per window (e.g. 10 Hz) it falls back to central differences."""
    dt = float(np.median(np.diff(t)))
    half = int(round(SMOOTH / dt / 2))
    if half < 1 or len(t) < 2 * half + 2:
        return np.gradient(x, t)
    g = np.arange(t[0], t[-1], dt)
    xg = np.interp(g, t, x)
    k = np.arange(-half, half + 1, dtype=float)
    kernel = k[::-1] / (np.sum(k * k) * dt)          # convolution flips it back: slope = sum(k x[i+k]) / sum(k^2) / dt
    d = np.convolve(xg, kernel, mode='same')
    d[:half] = d[half]; d[-half:] = d[-half - 1]     # ends: hold the first / last full-window value
    return np.interp(t, g, d)


def body_twist(mc):
    vxw, vyw = deriv(mc['t'], mc['x']), deriv(mc['t'], mc['y'])
    c, s = np.cos(mc['yaw']), np.sin(mc['yaw'])
    return c * vxw + s * vyw, -s * vxw + c * vyw, deriv(mc['t'], mc['yaw'])


def xcorr_offset(t_a, a, t_b, b, fs=FS, max_lag=None, center=0.0):
    """Clock offset d such that b(t + d) matches a(t): peak of the cross-correlation of the two signals resampled at
    fs on their own clocks. Returns d, the correlation coefficient at d, and the sign of b that fits (+1 / -1)."""
    ga = np.arange(t_a[0], t_a[-1], 1 / fs)
    gb = np.arange(t_b[0], t_b[-1], 1 / fs)
    sa = np.interp(ga, t_a, a); sa = sa - sa.mean()
    sb = np.interp(gb, t_b, b); sb = sb - sb.mean()
    nfft = 1 << (len(sa) + len(sb) - 1).bit_length()
    cc = np.fft.irfft(np.fft.rfft(sb, nfft) * np.conj(np.fft.rfft(sa, nfft)), nfft)   # cc[k] = sum a[n] b[n + k]
    lags = np.concatenate([np.arange(0, len(sb)), np.arange(-len(sa) + 1, 0)])
    cc = np.concatenate([cc[:len(sb)], cc[nfft - len(sa) + 1:]])
    offs = lags / fs + gb[0] - ga[0]
    if max_lag is not None:
        cc = np.where(np.abs(offs - center) <= max_lag, cc, 0.0)
    i = int(np.argmax(np.abs(cc)))
    di = 0.0
    if 0 < i < len(cc) - 1 and lags[i - 1] + 1 == lags[i] == lags[i + 1] - 1:
        y0, y1, y2 = np.abs(cc[i - 1]), np.abs(cc[i]), np.abs(cc[i + 1])
        den = y0 - 2 * y1 + y2
        di = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    d = offs[i] + di / fs
    m = (ga + d >= t_b[0]) & (ga + d <= t_b[-1])
    corr = abs(float(np.corrcoef(sa[m], np.interp(ga[m] + d, t_b, b))[0, 1])) if m.sum() > 10 else 0.0
    return d, corr, 1 if cc[i] > 0 else -1


def align(robot, mc, max_offset):
    """Clock of the Qualisys bag mapped on the robot clock: t_robot = t_mocap - (o0 + k (t_mocap - tm0))."""
    if 'imu_t' not in robot:
        sys.exit('no /imu_sensor/imu in the robot bag: needed to align the Qualisys clock')
    _, _, wz = body_twist(mc)
    ti, wi = robot['imu_t'], robot['imu_wz']
    # IMU smoothed like the Qualisys derivative
    k = max(1, int(round(0.1 / np.median(np.diff(ti)))))
    wi_s = np.convolve(wi, np.ones(k) / k, mode='same')
    off, corr, sgn = xcorr_offset(ti, wi_s, mc['t'], wz, max_lag=max_offset)
    print(f'Clock alignment, whole mission: Qualisys = robot {off:+.3f} s, correlation {corr:.2f}'
          f'{"" if sgn > 0 else "  (IMU z axis opposite to the Qualisys z: sign flipped)"}')
    # first and last third (sync turns at both ends) for the drift; the trajectories repeat at every shoulder angle,
    # so the search stays within 2 s of the global peak
    parts = []
    T = ti[-1] - ti[0]
    for a, b in ((0.0, 0.35), (0.65, 1.0)):
        m = (ti >= ti[0] + a * T) & (ti <= ti[0] + b * T)
        o, c, _ = xcorr_offset(ti[m], wi_s[m], mc['t'], wz, max_lag=2.0, center=off)   # near the global peak
        if c > 0.5:
            parts.append((ti[m].mean(), o, c))
    if len(parts) == 2:
        (ta, oa, ca), (tb, ob, cb) = parts
        drift = (ob - oa) / (tb - ta)
        print(f'  start {oa:+.3f} s (corr {ca:.2f}), end {ob:+.3f} s (corr {cb:.2f}): drift {drift * 1e6:+.0f} ppm')
        to_robot = lambda tm: tm - (oa + drift * (tm - oa - ta))
    else:
        print('  start/end offsets not reliable: constant offset')
        to_robot = lambda tm: tm - off
    mc['t'] = to_robot(mc['t'])
    if corr < 0.6:
        print('  WARNING: low correlation, check the alignment (body name, IMU topic, sync turns in the mission)')
    return sgn


def segments(robot):
    segs = robot['segments']
    if not segs:
        sys.exit(f'no {SEGMENT_TOPIC} in the robot bag (missions are started with START on the NUC joystick)')
    out = []
    for (t0, info), (t1, _) in zip(segs[:-1], segs[1:]):
        if info.get('segment') in ('end',):
            continue
        out.append((t0, t1, info))
    return out


def mean_in(t, x, a, b):
    m = (t >= a) & (t < b)
    return float(np.mean(x[m])) if m.sum() >= 3 else math.nan


def analyse(robot, mc, imu_sign, h):
    vx_b, vy_b, wz_b = body_twist(mc)
    rows = []
    for t0, t1, info in segments(robot):
        name = info['segment']
        if name in ('move', 'stop') or info.get('hips_moving'):
            continue
        a, b = t0 + SKIP, t1 - 0.2
        if b - a < 1.0:
            continue
        rt = robot['t']
        vsum = info['v_left'] + info['v_right']
        r = {'mission': info.get('mission'), 'i': info['i'], 'segment': name, 't0': round(t0 - rt[0], 2),
             'dir': 'fwd' if vsum > 1e-6 else 'back' if vsum < -1e-6 else 'ccw' if info['v_right'] > 0 else 'cw',
             'alpha_F': info['alpha_F'], 'alpha_H': info['alpha_H'],
             'Vcmd_L_set': info['v_left'], 'Vcmd_R_set': info['v_right']}
        for k in LEGS:
            r[f'alpha_{k}'] = mean_in(rt, robot[f'alpha_{k}'], a, b)
            r[f'V_{k}'] = mean_in(rt, robot[f'V_{k}'], a, b)
            if f'Vcmd_{k}' in robot:
                r[f'Vcmd_{k}'] = mean_in(rt, robot[f'Vcmd_{k}'], a, b)
        r['V_L'] = np.nanmean([r['V_LF'], r['V_LH']])
        r['V_R'] = np.nanmean([r['V_RF'], r['V_RH']])
        r['Vcmd_L'] = np.nanmean([r.get('Vcmd_LF', math.nan), r.get('Vcmd_LH', math.nan)])
        r['Vcmd_R'] = np.nanmean([r.get('Vcmd_RF', math.nan), r.get('Vcmd_RH', math.nan)])
        r['vx'] = mean_in(mc['t'], vx_b, a, b)
        r['vy'] = mean_in(mc['t'], vy_b, a, b)
        r['wz'] = mean_in(mc['t'], wz_b, a, b)
        if 'imu_t' in robot:
            r['wz_imu'] = imu_sign * mean_in(robot['imu_t'], robot['imu_wz'], a, b)
        r['x_mean'] = float(np.mean([SHOULDER_X + LEG_L * math.cos(r[f'alpha_{k}']) for k in LEGS]))
        r['ratio_L'] = r['V_L'] / r['Vcmd_L'] if abs(r['Vcmd_L']) > 1e-3 else math.nan
        r['ratio_R'] = r['V_R'] / r['Vcmd_R'] if abs(r['Vcmd_R']) > 1e-3 else math.nan
        turning = abs(r['wz']) > 0.08
        dv, dv_cmd = r['V_R'] - r['V_L'], r['Vcmd_R'] - r['Vcmd_L']
        r['y_true'] = dv / (2 * r['wz']) if turning else math.nan
        r['h_over_y'] = h / r['y_true'] if turning else math.nan
        r['wz_over_cmd'] = r['wz'] / (dv_cmd / (2 * h)) if abs(dv_cmd) > 1e-3 else math.nan
        r['y_L'] = (r['vx'] - r['V_L']) / r['wz'] if turning else math.nan
        r['y_R'] = (r['vx'] - r['V_R']) / r['wz'] if turning else math.nan
        r['x_icr'] = -r['vy'] / r['wz'] if turning else math.nan
        vm = 0.5 * (r['V_L'] + r['V_R'])
        r['vx_over_V'] = r['vx'] / vm if name == 'straight' and abs(vm) > 1e-3 else math.nan
        rows.append(r)
    return rows


def heading_check(rows, mc):
    """Straight runs: the body velocity should point along x; its angle is the heading offset of the rigid body."""
    ang = [math.atan2(r['vy'], r['vx']) if r['vx'] > 0 else math.atan2(-r['vy'], -r['vx'])
           for r in rows if r['segment'] == 'straight' and math.hypot(r['vx'], r['vy']) > 0.03]
    if ang:
        print(f'Heading check on {len(ang)} straight runs: velocity at {math.degrees(np.median(ang)):+.1f} deg from the '
              f'rigid body x axis (spread {math.degrees(np.std(ang)):.1f} deg); if not ~0, pass --rb-offset 0 0 '
              f'{np.median(ang):+.3f}')


def fit_gamma(rows, h):
    pts = [(r['x_mean'], r['y_true']) for r in rows if np.isfinite(r['y_true']) and r['y_true'] > 0]
    if len(pts) < 3:
        return None
    u = np.array([(x / h) ** 2 for x, _ in pts])
    z = np.array([y / h - 1 for _, y in pts])
    g = float(u @ z / (u @ u))
    return g, float(np.sqrt(np.mean((z - g * u) ** 2)) * h)


def f(x, fmt='{:+.3f}'):
    return '   -   ' if x is None or not np.isfinite(x) else fmt.format(x)


def report(rows, h):
    print('\n seg  type      alpha  Vcmd L/R [m/s]   V/Vcmd L/R   vx [m/s] vy [m/s] wz [rad/s] wz_imu  '
          'wz/wz_cmd  h/y_true  y_L [m]  y_R [m]  x_icr [m]  vx/V')
    for r in rows:
        print(f'{r["i"]:4d}  {r["segment"]:8s}  {r["alpha_F"]:.2f}  {f(r["Vcmd_L"])} {f(r["Vcmd_R"])}   '
              f'{f(r["ratio_L"], "{:.2f}")} {f(r["ratio_R"], "{:.2f}")}   {f(r["vx"])}  {f(r["vy"])}  {f(r["wz"])}  '
              f'{f(r.get("wz_imu"))}  {f(r["wz_over_cmd"], "{:.2f}")}     {f(r["h_over_y"], "{:.2f}")}   '
              f'{f(r["y_L"])}  {f(r["y_R"])}  {f(r["x_icr"])}  {f(r["vx_over_V"], "{:.3f}")}')
    # summary per shoulder angle and segment type
    print('\nPer shoulder angle (mean over the segments):')
    print(' type      dir   alpha   n   V/Vcmd   h/y_true  wz/wz_cmd   y_L      y_R      x_icr    vx/V')
    keys = sorted({(r['segment'], r['dir'], round(r['alpha_F'], 2)) for r in rows})
    for name, dr, al in keys:
        g = [r for r in rows if r['segment'] == name and r['dir'] == dr and round(r['alpha_F'], 2) == al]
        m = lambda k: np.nanmean([r[k] for r in g]) if any(np.isfinite(r[k]) for r in g) else math.nan
        ratio = np.nanmean([x for r in g for x in (r['ratio_L'], r['ratio_R'])])
        print(f' {name:8s}  {dr:4s}  {al:.2f}  {len(g):2d}   {f(ratio, "{:.2f}")}     {f(m("h_over_y"), "{:.2f}")}     '
              f'{f(m("wz_over_cmd"), "{:.2f}")}   {f(m("y_L"))}  {f(m("y_R"))}  {f(m("x_icr"))}  {f(m("vx_over_V"), "{:.3f}")}')
    gm = fit_gamma(rows, h)
    if gm:
        print(f'\ngamma of the traquad IK (yaw_slip_gain) on these segments: {gm[0]:.3f}, rms on y_eff {gm[1] * 1000:.0f} mm')


def timeseries(robot, mc, path):
    rt = robot['t']
    g = np.arange(rt[0], rt[-1], 1 / FS)
    vx, vy, wz = body_twist(mc)
    cols = {'t': g - rt[0]}
    for k in LEGS:
        cols[f'alpha_{k}'] = np.interp(g, rt, robot[f'alpha_{k}'])
        cols[f'V_{k}'] = np.interp(g, rt, robot[f'V_{k}'])
        if f'Vcmd_{k}' in robot:
            cols[f'Vcmd_{k}'] = np.interp(g, rt, robot[f'Vcmd_{k}'])
    if 'imu_t' in robot:
        cols['wz_imu'] = np.interp(g, robot['imu_t'], robot['imu_wz'])
    inside = (g >= mc['t'][0]) & (g <= mc['t'][-1])
    for k, v in (('x', mc['x']), ('y', mc['y']), ('z', mc['z']), ('yaw', mc['yaw']), ('vx', vx), ('vy', vy), ('wz', wz)):
        cols[f'mocap_{k}'] = np.where(inside, np.interp(g, mc['t'], v), np.nan)
    seg = np.full(len(g), -1)
    for t0, t1, info in segments(robot):
        seg[(g >= t0) & (g < t1)] = info['i']
    cols['segment'] = seg
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(len(g)):
            w.writerow([f'{cols[k][i]:.5g}' if k != 'segment' else int(cols[k][i]) for k in cols])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bag', required=True, help='robot bag of the mission (SBC launch, record_bag:=true)')
    p.add_argument('--mocap', required=True, help='Qualisys bag with /rigid_bodies (can be the robot bag)')
    p.add_argument('--body', default=None, help='rigid body name (default: the only one in the bag)')
    p.add_argument('--rb-offset', nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=('DX', 'DY', 'DYAW'),
                   help='base centre in the rigid body frame [m] and yaw of the robot x axis in it [rad]')
    p.add_argument('--max-offset', type=float, default=None,
                   help='largest clock offset between the two computers to search [s] (default: any)')
    p.add_argument('--track-width', type=float, default=0.4)
    p.add_argument('--smooth', type=float, default=0.1,
                   help='window of the Qualisys derivative [s] (twist and yaw rate; 0.1 s = 20 samples at 200 Hz)')
    p.add_argument('--out', default=None, help='prefix of the CSV files (default: the robot bag name)')
    args = p.parse_args()
    global SMOOTH
    SMOOTH = args.smooth
    robot = load_robot(args.bag)
    mc = apply_rb_offset(load_mocap(args.mocap, args.body), *args.rb_offset)
    same = Path(args.bag).resolve() == Path(args.mocap).resolve()
    sgn = 1
    if same:
        print('Qualisys in the robot bag: same clock, no alignment')
        if 'imu_t' in robot:
            _, _, wz = body_twist(mc)
            sgn = 1 if np.corrcoef(np.interp(robot['imu_t'], mc['t'], wz), robot['imu_wz'])[0, 1] >= 0 else -1
    else:
        sgn = align(robot, mc, args.max_offset)
    h = args.track_width / 2
    rows = analyse(robot, mc, sgn, h)
    heading_check(rows, mc)
    report(rows, h)
    out = args.out or str(Path(args.bag).resolve())
    with open(out + '_segments.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else ['i'])
        w.writeheader()
        w.writerows(rows)
    timeseries(robot, mc, out + '_timeseries.csv')
    print(f'\nwritten {out}_segments.csv and {out}_timeseries.csv')


if __name__ == '__main__':
    main()
