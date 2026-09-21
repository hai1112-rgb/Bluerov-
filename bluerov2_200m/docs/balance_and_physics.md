# BlueROV2 Balance and 200 m Physics

## World depth

The Gazebo/RViz world is configured as a 200 m water column:

- Water surface: `z = 0`
- Seafloor: `z = -200`
- Simulator depth clamp: `-199.5 <= z <= -0.1`
- Seawater density: `1025 kg/m^3`
- Gravity: `9.80665 m/s^2`

The lightweight simulator publishes hydrostatic pressure on:

```bash
ros2 topic echo /bluerov2/sim/water_pressure
```

Pressure is computed as:

```text
P = 101325 + rho * g * depth
```

At 200 m this is about 2.11 MPa absolute pressure.

## Thruster coordination

The thruster manager receives a 6-DOF wrench:

```text
[surge, sway, heave, roll, pitch, yaw]
```

It maps that wrench into motor commands with a weighted damped least-squares
allocator using `config/TAM.yaml`:

```text
thruster_forces = solve((W*TAM)^T(W*TAM) + lambda^2 I, (W*TAM)^T(W*wrench))
achieved_wrench = TAM * thruster_forces
```

The weights are set in `config/thruster_manager.yaml`. The current values give
high priority to depth, yaw, surge, and sway, keep roll authority active, and
de-prioritize pitch because the 6-thruster BlueROV2 layout has weak pitch
authority.

For live inspection:

```bash
ros2 topic echo /bluerov2/thruster_manager/thrust_setpoints
ros2 topic echo /bluerov2/thruster_manager/allocation_report
```

The six-thruster model uses:

- T0-T3: vectored horizontal thrusters for surge, sway, and yaw.
- T4-T5: vertical thrusters for heave and roll trim.
- Pitch control is limited because this 6-thruster layout has very little pitch authority. A BlueROV2 Heavy 8-thruster layout would be stronger for pitch/depth hold.

## Balance algorithms

The autonomous controller uses PID feedback for the balance axes and PD feedback
for horizontal navigation:

```text
Fx = kp_xy * x_error_body - kd_xy * vx_body
Fy = kp_xy * y_error_body - kd_xy * vy_body
Fz = kp_z * z_error + ki_z * integral_z - kd_z * vz_body + depth_feedforward_force
Tx = kp_roll  * (-roll)  + ki_roll  * integral_roll  - kd_roll  * roll_rate
Ty = kp_pitch * (-pitch) + ki_pitch * integral_pitch - kd_pitch * pitch_rate
Tz = kp_yaw * yaw_error + ki_yaw * integral_yaw - kd_yaw * yaw_rate
```

Each integral term has an anti-windup limit and stops integrating when the
requested output is saturated in the same direction as the error. This removes
steady-state depth/heading/trim error from buoyancy mismatch and steady current
without letting the controller wind up while the thrusters are saturated.

The simulator also adds passive hydrostatic self-righting:

```text
Tx_righting = -(roll_stiffness  + buoyancy * cb_z) * sin(roll)
Ty_righting = -(pitch_stiffness + buoyancy * cb_z) * sin(pitch)
```

This means the ROV can settle naturally like a real submerged body, while the controller actively damps roll/pitch/yaw and depth errors through the TAM allocation.

## Physical effects included

- Added mass on X/Y/Z.
- Linear and quadratic hydrodynamic drag.
- Water-current drag based on relative water velocity.
- Depth shear: current is weaker near the seabed.
- Hydrostatic pressure.
- Weight and buoyancy with slightly positive trim.
- Passive roll/pitch righting moment.
- Wave force fades out below 10 m.
