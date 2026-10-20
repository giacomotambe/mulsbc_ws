# Omni Controller

Unified ros2_control controller for the Mulinex omnidirectional robot.
It:
- receives per-joint reference setpoints for non-wheel joints (legs, arms, etc.)
- receives planar twist commands and converts them to wheel commands
- broadcasts joint states and power distributor states

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `wheel_joints.<POS>` | `string` or `string[]` | `""` / `[]` | Wheel joint name(s) per position. Mecanum: `LF`, `LH`, `RF`, `RH` (single string each). Differential / traquad: `LEFT`, `RIGHT` (string array each — supports multiple wheels per side for skid-steer). |
| `joints` | `string[]` | `[]` | Joint names — legs, arms, etc. (must match URDF) |
| `distributor_names` | `string[]` | `[]` | Power distributor names |
| `second_encoder_joints` | `string[]` | `[]` | Joints with secondary encoders |
| `feet_type` | `string` | `"none"` | Wheel IK type: `"mecanum"`, `"differential"`, `"traquad"`, or `"none"` |
| `wheel_rad` | `double` | `0.05` | Wheel radius [m] |
| `driveshaft_x` | `double` | `0.235` | Longitudinal offset from base center to wheel [m] |
| `driveshaft_y` | `double` | `0.188` | Lateral offset from base center to wheel [m] |
| `mecanum_angle` | `double` | `135.0` | Mecanum roller angle [deg] (mecanum IK only) |
| `track_width` | `double` | `0.4` | Track width [m] (differential / traquad IK only) |
| `traquad.leg_length` | `double` | `0.19` | HFE → ankle length [m] (traquad IK only) |
| `traquad.shoulder_coupling` | `double` | `1.0` | 1 = wheel encoder relative to the leg, 0 = relative to the body (traquad IK only) |
| `input_frequency` | `int` | `100` | Expected twist command rate [Hz] (sets deadline QoS) |
| `BestEffort_QOS` | `bool` | `true` | Use BestEffort (true) or Reliable (false) QoS for twist subscriber |
| `sim` | `bool` | `false` | Simulation mode (velocity-only command interfaces) |
| `pub_odom` | `bool` | `false` | Publish wheel odometry |
| `pub_performance` | `bool` | `true` | Publish performance indexes |
| `transition_wheels.mode` | `string` | `"lock"` | Wheel behaviour during STAND/REST transitions. `"lock"`: wheels braked (velocity 0, kp_scale 0, kd_scale 1). `"assist"`: each wheel is driven opposite to the HFE joint of the same leg (see below). ACTIVATION always uses `"lock"`. |
| `transition_wheels.gain` | `double` | `0.0` | Assist gain `k` (> 0). With `mode: assist` and `gain <= 0` the controller falls back to `"lock"`. |
| `homing_phases` | `double[]` | `[]` | Duration(s) for homing phases [s]. 1 entry = single phase (qi→qf), 2 entries = two phases (qi→qm, qm→qf). Empty disables homing. |
| `homing_config.<joint>.qi` | `double` | `0.0` | Initial position for homing trajectory [rad] (ignored in phase 1 — uses current position) |
| `homing_config.<joint>.qf` | `double` | `0.0` | Final position for homing trajectory [rad] |
| `homing_config.<joint>.qm` | `double` | `0.0` | Mid position for two-phase homing [rad] (only used when `homing_phases` has 2 entries) |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `~/twist_cmd` | `Twist` | sub | Base velocity command (when `wheel_joints` + IK configured) |
| `~/joints_reference` | `JointsCommand` | sub | Per-joint reference setpoints from upstream controllers (e.g. `ik_controller`); applied to non-wheel joints (legs, arms, etc.) when `joints` is configured |
| `~/joints_state` | `JointsStates` | pub | All motor joint states (hardware measurements) |
| `~/debug/joints_command` | `JointsCommand` | pub | Echo of the commands actually written to hardware each cycle (all motors); for debugging / logging |
| `~/performance` | `PacketPass` | pub | CAN packet loss & cycle duration (when `pub_performance: true`) |
| `~/distributors_state` | `DistributorsState` | pub | Power distributor states (when `distributor_names` configured) |
| `~/odom` | `TwistStamped` | pub | Wheel odometry (when `pub_odom: true`) |

## Services

| Service | Type | Description |
|---|---|---|
| `~/activate_srv` | `SetBool` | Transition from INACTIVE to ACTIVE (`data: true`) |
| `~/emergency_srv` | `SetBool` | Transition from ACTIVE to INACTIVE (`data: true`) |
| `~/homing_srv` | `SetBool` | Start leg homing sequence (`data: true`). Only available when `homing_phases` is configured and legs are present. |

## State Machine

```
on_activate()
     │
     ▼
 INACTIVE ──── ~/activate_srv (data: true) ────► ACTIVE
     ▲                                              │
     │                                              │
     ├──── ~/emergency_srv (data: true) ◄───────────┤
     └──── 10+ deadline misses (auto) ◄─────────────┘

 INACTIVE or ACTIVE ── ~/homing_srv (data: true) ──► HOMING ──(complete)──► ACTIVE
```

- **INACTIVE**: All motor commands zeroed (velocity=0, kp_scale=0, kd_scale=1). State publishing continues.
- **ACTIVE**: Wheel IK and leg commands written to hardware.
- **HOMING**: Legs follow a time-parameterized trajectory (qi→qf or qi→qm→qf). Wheels are locked (velocity=0, kp_scale=1). Transitions to ACTIVE when complete.

## Track-assisted stand-up (`transition_wheels`)

During STAND/REST the HFE joints follow a cosine interpolation in position control:

```
q(t)    = q_start + ½·(1 − cos(π·t))·(q_target − q_start),     t = elapsed / duration ∈ [0, 1]
q̇(t)    = (q_target − q_start) · ½·π·sin(π·t) / duration
ω_wheel = −k · q̇_HFE(t)
```

With `mode: assist`, every wheel receives `ω_wheel` computed from the HFE joint of the same leg
(paired by name prefix: `LF_WHEEL → LF_HFE`), with kp_scale 0 and kd_scale 1. The command is 0 at
start and end and peaks mid-transition; on REST `q̇` changes sign, so the wheels reverse too.
Assist requires wheels, legs, transitions and an HFE joint for every wheel, otherwise the controller
falls back to `lock` with a warning. When the transition ends the wheels return to normal ACTIVE control.

Configs without `transition_wheels` (e.g. omniquad, omnicar) keep the default `lock`.

```yaml
transition_wheels:
  mode: assist
  gain: 7.4
```

## Wheel IK Types

- **`mecanum`**: 4-wheel mecanum inverse kinematics (LF, LH, RF, RH). Requires `wheel_joints.{LF,LH,RF,RH}`, `driveshaft_x`, `driveshaft_y`, `mecanum_angle`, `wheel_rad`.
- **`differential`**: Differential drive with 1 or more wheels per side. Requires `wheel_joints.{LEFT,RIGHT}` (string arrays), `track_width`, `wheel_rad`. All wheels on the same side receive the same velocity. Odometry averages wheel velocities per side.
- **`traquad`**: Skid-steer on tracked legs (traquad). Same `wheel_joints.{LEFT,RIGHT}` as `differential`, plus the leg joints: each wheel is paired with the HFE joint of the same leg (`LF_WHEEL` → `LF_HFE`) and gets its own velocity, compensating the shoulder motion: `ω_i = (κ_i·vx − track_width/2·ωz)/r − (traquad.leg_length·sin α_i/r + traquad.shoulder_coupling)·q̇_i`, with κ = +1 left / −1 right and α = σ·q_HFE (σ = κ for front legs, −κ for hind legs). With still shoulders it equals `differential`. Odometry uses the same model.
- **`none`**: No IK. Wheel joints still get zero commands when INACTIVE but receive no IK output when ACTIVE (useful for direct joint control via `joints`).

## Example Config

See [`config/omnicar.yaml`](config/omnicar.yaml) for a complete Omnicar configuration.

### Differential (skid-steer) example

```yaml
omni_controller:
  ros__parameters:
    feet_type: differential
    wheel_joints:
      LEFT:  [left_front_jnt, left_rear_jnt]
      RIGHT: [right_front_jnt, right_rear_jnt]
    track_width: 0.4
    wheel_rad: 0.05
```
