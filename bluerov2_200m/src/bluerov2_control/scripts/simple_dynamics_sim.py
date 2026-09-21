#!/usr/bin/env python3
"""
BlueROV2 Simple Dynamics Simulator - Enhanced
=============================================
Improvements over the baseline lightweight simulator:
- BlueROV2 Heavy physical parameters (mass=11.5 kg)
- Surface waves disabled by default for deep-water work near 200 m
- Disturbance commands from /disturbance/mode
- Full inertia parameters for yaw, pitch, and roll
- T200-based velocity limits

Topics:
  Sub: thruster_manager/commanded_wrench (Wrench)
  Sub: /disturbance/mode (String): "normal" | "storm" | "current_n" | "current_e" | "current_strong"
  Pub: sim/pose_gt (Odometry)
  Pub: joint_states (JointState)
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def body_to_world(x, y, z, roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    )


def world_to_body(x, y, z, roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cy * cp) * x + (sy * cp) * y - sp * z,
        (cy * sp * sr - sy * cr) * x + (sy * sp * sr + cy * cr) * y + cp * sr * z,
        (cy * sp * cr + sy * sr) * x + (sy * sp * cr - cy * sr) * y + cp * cr * z,
    )


class SimpleDynamicsSim(Node):
    def __init__(self):
        super().__init__("simple_dynamics_sim")

        # ---- BlueROV2 Heavy physical parameters ----
        # Mass: 11.5 kg for a lightly loaded BlueROV2 Heavy.
        # Added mass (hydrodynamic): ~20% mass
        # Linear drag values are tuned from practical tank/CFD-style estimates.
        defaults = {
            "update_rate": 50.0,
            "world_frame": "world",
            "base_frame": "base_link",
            "commanded_wrench_topic": "thruster_manager/commanded_wrench",
            "odom_topic": "sim/pose_gt",
            "joint_state_topic": "joint_states",
            "pressure_topic": "sim/water_pressure",
            "publish_tf": False,
            "print_physics_status": False,
            "physics_status_rate": 1.0,

            # === BlueROV2 Heavy physical params ===
            # Mass: 11.5 kg; added mass raises the effective inertia.
            "mass": 11.5,
            "water_depth": 220.0,
            "water_density": 1025.0,
            "gravity": 9.80665,
            "vehicle_volume": 0.01135,  # m^3, slightly positive BlueROV2 trim
            "added_mass_x": 6.357,       # kg - hydrodynamic added mass X
            "added_mass_y": 7.121,       # kg - hydrodynamic added mass Y
            "added_mass_z": 18.68,       # kg - hydrodynamic added mass Z
            "yaw_inertia": 0.252,        # kg·m² - moment of inertia yaw (Z)
            "pitch_inertia": 0.245,      # kg·m²
            "roll_inertia": 0.245,       # kg·m²
            "center_of_buoyancy_z": 0.035,  # m above CG; gives passive self-righting
            "metacentric_roll_stiffness": 4.5,
            "metacentric_pitch_stiffness": 4.5,

            # Linear damping from CFD/test-style estimates.
            "linear_drag_xy": 13.7,      # N/(m/s) - lateral drag
            "linear_drag_z": 33.0,       # N/(m/s) - vertical drag
            "angular_drag_x": 3.8,       # N·m/(rad/s) - roll drag
            "angular_drag_y": 4.0,       # N·m/(rad/s) - pitch drag
            "angular_drag_z": 6.5,       # N·m/(rad/s) - yaw drag

            # Quadratic drag adds high-speed damping.
            "quad_drag_xy": 8.0,
            "quad_drag_z": 10.0,
            "quad_angular_drag_xy": 0.8,

            # Velocity limits based on the four horizontal T200 thrusters.
            "max_linear_speed": 1.5,     # m/s horizontal
            "max_vertical_speed": 0.8,   # m/s vertical
            "max_roll_rate": 1.0,
            "max_pitch_rate": 1.0,
            "max_yaw_rate": 1.2,         # rad/s
            "max_attitude_angle": 1.0471975512,

            # Initial pose
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_z": -200.0,
            "initial_roll": 0.0,
            "initial_pitch": 0.0,
            "initial_yaw": 0.0,

            # Wave parameters
            "wave_amplitude": 0.0,       # N - surface waves ignored near 200 m depth
            "wave_period": 6.0,          # seconds
            "wave_direction": 0.0,       # rad - world frame, 0=+X

            # Background current: world-frame horizontal water velocity.
            "current_speed": 0.24,       # m/s current aloft; shear gives ~0.13 m/s near 200 m
            "current_direction": 0.2617993878,  # rad, 15 deg from +X world
            "current_vertical_speed": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.step_dt = 1.0 / self.update_rate
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = as_bool(self.get_parameter("publish_tf").value)
        self.print_physics_status = as_bool(self.get_parameter("print_physics_status").value)
        self.physics_status_rate = max(
            0.0,
            float(self.get_parameter("physics_status_rate").value),
        )

        # Physical parameters
        self.mass = max(1e-6, float(self.get_parameter("mass").value))
        self.water_depth = max(1.0, float(self.get_parameter("water_depth").value))
        self.water_density = max(0.0, float(self.get_parameter("water_density").value))
        self.gravity = max(0.0, float(self.get_parameter("gravity").value))
        self.vehicle_volume = max(0.0, float(self.get_parameter("vehicle_volume").value))
        self.added_mass_x = max(0.0, float(self.get_parameter("added_mass_x").value))
        self.added_mass_y = max(0.0, float(self.get_parameter("added_mass_y").value))
        self.added_mass_z = max(0.0, float(self.get_parameter("added_mass_z").value))
        self.yaw_inertia = max(1e-6, float(self.get_parameter("yaw_inertia").value))
        self.pitch_inertia = max(1e-6, float(self.get_parameter("pitch_inertia").value))
        self.roll_inertia = max(1e-6, float(self.get_parameter("roll_inertia").value))
        self.center_of_buoyancy_z = float(self.get_parameter("center_of_buoyancy_z").value)
        self.metacentric_roll_stiffness = max(
            0.0,
            float(self.get_parameter("metacentric_roll_stiffness").value),
        )
        self.metacentric_pitch_stiffness = max(
            0.0,
            float(self.get_parameter("metacentric_pitch_stiffness").value),
        )
        self.linear_drag_xy = float(self.get_parameter("linear_drag_xy").value)
        self.linear_drag_z = float(self.get_parameter("linear_drag_z").value)
        self.angular_drag_x = float(self.get_parameter("angular_drag_x").value)
        self.angular_drag_y = float(self.get_parameter("angular_drag_y").value)
        self.angular_drag_z = float(self.get_parameter("angular_drag_z").value)
        self.quad_drag_xy = float(self.get_parameter("quad_drag_xy").value)
        self.quad_drag_z = float(self.get_parameter("quad_drag_z").value)
        self.quad_angular_drag_xy = float(self.get_parameter("quad_angular_drag_xy").value)
        self.max_linear_speed = max(0.0, float(self.get_parameter("max_linear_speed").value))
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)
        self.max_roll_rate = max(0.0, float(self.get_parameter("max_roll_rate").value))
        self.max_pitch_rate = max(0.0, float(self.get_parameter("max_pitch_rate").value))
        self.max_yaw_rate = max(0.0, float(self.get_parameter("max_yaw_rate").value))
        self.max_attitude_angle = max(0.0, float(self.get_parameter("max_attitude_angle").value))

        # Wave params
        self.wave_amplitude = float(self.get_parameter("wave_amplitude").value)
        self.wave_period = max(0.1, float(self.get_parameter("wave_period").value))
        self.wave_direction = float(self.get_parameter("wave_direction").value)
        self.current_speed = float(self.get_parameter("current_speed").value)
        self.current_direction = float(self.get_parameter("current_direction").value)
        self.current_vertical_speed = float(self.get_parameter("current_vertical_speed").value)

        # Effective mass (including added mass)
        self.eff_mass_x = self.mass + self.added_mass_x
        self.eff_mass_y = self.mass + self.added_mass_y
        self.eff_mass_z = self.mass + self.added_mass_z

        # State
        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.roll = float(self.get_parameter("initial_roll").value)
        self.pitch = float(self.get_parameter("initial_pitch").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.vz_body = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.commanded_wrench = Wrench()
        self.latest_physics_status = {}

        # Disturbance mode: "normal", "storm", "current_n", "current_e", "current_strong"
        self.disturbance_mode = "normal"
        self.disturbance_start_time = 0.0

        # Sim time counter
        self.sim_time = 0.0
        self.joint_names = [
            "thruster_0_joint",
            "thruster_1_joint",
            "thruster_2_joint",
            "thruster_3_joint",
            "thruster_4_joint",
            "thruster_5_joint",
        ]
        self.zero_joint_positions = [0.0] * len(self.joint_names)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.pressure_topic = str(self.get_parameter("pressure_topic").value)
        self.commanded_wrench_topic = str(self.get_parameter("commanded_wrench_topic").value)

        # ROS pubs/subs
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.pressure_pub = self.create_publisher(Float64, self.pressure_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(
            Wrench,
            self.commanded_wrench_topic,
            self._wrench_callback, 10)

        # Receive disturbance commands from the keyboard controller.
        self.create_subscription(
            String, "/disturbance/mode", self._disturbance_callback, 10)

        self.create_timer(self.step_dt, self._step)
        if self.print_physics_status and self.physics_status_rate > 0.0:
            self.create_timer(1.0 / self.physics_status_rate, self._publish_physics_status)

        self.get_logger().info(
            "BlueROV2 Enhanced Dynamics Sim started | "
            "mass=%.1f kg | depth=%.0fm | volume=%.5fm^3 | wave_T=%.1fs | "
            "Disturbance modes: storm|current_n|current_e|current_strong|normal"
            % (self.mass, self.water_depth, self.vehicle_volume, self.wave_period))

    # ------------------------------------------------------------------
    def _wrench_callback(self, msg):
        self.commanded_wrench = msg

    def _disturbance_callback(self, msg):
        mode = msg.data.strip().lower()
        if mode != self.disturbance_mode:
            self.disturbance_mode = mode
            self.disturbance_start_time = self.sim_time
            self.get_logger().info(f"[DISTURBANCE] Mode changed to: {mode.upper()}")

    # ------------------------------------------------------------------
    def _compute_wave_force(self):
        """
        Surface-wave force. The default is 0.0 for deep-water operation near
        200 m, but the function remains available for shallow-water tests.
        """
        if self.z < -10.0:
            depth_factor = 0.0  # Surface waves are ignored below 10 m.
        else:
            depth_factor = max(0.0, 1.0 + self.z / 10.0)  # 0..1

        amp = self.wave_amplitude * depth_factor
        phase = 2 * math.pi * self.sim_time / self.wave_period
        wave_f = amp * math.sin(phase)

        fx = wave_f * math.cos(self.wave_direction)
        fy = wave_f * math.sin(self.wave_direction)
        fz = amp * 0.3 * math.sin(phase + math.pi / 3)  # Smaller vertical force.
        return fx, fy, fz

    def _compute_disturbance_force(self):
        """
        Short-lived storm disturbance force. Steady current is represented as
        water velocity in _compute_current_velocity so drag depends on relative
        velocity, matching the real ocean model more closely.
        """
        mode = self.disturbance_mode
        t = self.sim_time - self.disturbance_start_time

        if mode == "normal":
            return 0.0, 0.0, 0.0, 0.0

        elif mode == "storm":
            # Storm mode: large multi-axis gust force.
            gust = 1.0 + 0.5 * math.sin(2 * math.pi * t / 3.0) + \
                   0.3 * math.sin(2 * math.pi * t / 1.2)
            fx = 8.0 * gust * math.cos(0.5 * t)
            fy = 6.0 * gust * math.sin(0.7 * t)
            fz = 2.0 * math.sin(1.5 * t)
            tz = 1.5 * math.sin(0.8 * t)  # Yaw disturbance torque.
            return fx, fy, fz, tz

        return 0.0, 0.0, 0.0, 0.0

    def _compute_current_velocity(self):
        """Return world-frame water velocity in m/s."""
        speed = self.current_speed
        direction = self.current_direction
        vertical_speed = self.current_vertical_speed

        mode = self.disturbance_mode
        if mode == "current_n":
            speed = max(speed, 0.25)
            direction = math.pi / 2.0
        elif mode == "current_e":
            speed = max(speed, 0.25)
            direction = 0.0
        elif mode == "current_nw":
            speed = max(speed, 0.25)
            direction = 3.0 * math.pi / 4.0
        elif mode == "current_strong":
            speed = max(speed, 0.55)
            direction = math.pi / 2.0
        elif mode == "storm":
            speed = max(speed, 0.35 + 0.12 * math.sin(0.4 * self.sim_time))
            direction = 0.6 * math.sin(0.15 * self.sim_time)

        depth = max(0.0, -self.z)
        shear = 1.0
        if self.water_depth > 1e-6:
            # Deep current is usually weaker close to the seabed.
            shear = clamp(0.55 + 0.45 * (1.0 - depth / self.water_depth), 0.55, 1.0)

        return (
            speed * shear * math.cos(direction),
            speed * shear * math.sin(direction),
            vertical_speed,
        )

    def _hydrostatic_pressure(self):
        depth = max(0.0, -self.z)
        atmospheric_pressure = 101325.0
        return atmospheric_pressure + self.water_density * self.gravity * depth

    def _format_vector(self, values, precision=2):
        return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"

    def _publish_physics_status(self):
        status = self.latest_physics_status
        if not status:
            return

        self.get_logger().info(
            "[PHYSICS STATUS] depth=%.2fm P=%.2fbar | W=%.2fN Fb=%.2fN Fb-W=%+.2fN | "
            "tau_body=%s | current_body=%s nu_body=%s nu_r=%s | "
            "drag_resist_body=%s | wave_world=%s disturbance_world=%s | "
            "net_force_body=%s acc_body=%s | righting_moment=%s angular_drag_resist=%s"
            % (
                status["depth"],
                status["pressure"] / 100000.0,
                status["weight"],
                status["buoyancy"],
                status["net_buoyancy"],
                self._format_vector(status["thruster_body"]),
                self._format_vector(status["current_body"]),
                self._format_vector(status["velocity_body"]),
                self._format_vector(status["relative_velocity_body"]),
                self._format_vector(status["drag_resist_body"]),
                self._format_vector(status["wave_world"]),
                self._format_vector(status["disturbance_world"]),
                self._format_vector(status["net_force_body"]),
                self._format_vector(status["acceleration_body"]),
                self._format_vector(status["righting_moment"]),
                self._format_vector(status["angular_drag_resist"]),
            )
        )

    # ------------------------------------------------------------------
    def _step(self):
        self.sim_time += self.step_dt

        # 1. Thruster force in the body frame.
        fx_cmd = self.commanded_wrench.force.x
        fy_cmd = self.commanded_wrench.force.y
        fz_cmd = self.commanded_wrench.force.z
        tx_cmd = self.commanded_wrench.torque.x
        ty_cmd = self.commanded_wrench.torque.y
        tz_cmd = self.commanded_wrench.torque.z

        # 2. Surface-wave force in the world frame.
        wfx, wfy, wfz = self._compute_wave_force()

        # 3. Disturbance force in the world frame.
        dfx, dfy, dfz, dtz = self._compute_disturbance_force()

        # 4. Transform body thrust to world and add SI-consistent weight/buoyancy.
        fx_world, fy_world, fz_world = body_to_world(
            fx_cmd,
            fy_cmd,
            fz_cmd,
            self.roll,
            self.pitch,
            self.yaw,
        )
        weight = self.mass * self.gravity
        buoyancy = self.water_density * self.gravity * self.vehicle_volume

        fx_total_world = fx_world + wfx + dfx
        fy_total_world = fy_world + wfy + dfy
        fz_total_world = fz_world + wfz + dfz + buoyancy - weight

        # 5. Drag uses relative velocity between the vehicle and the water.
        current_world = self._compute_current_velocity()
        water_vx_body, water_vy_body, water_vz_body = world_to_body(
            current_world[0],
            current_world[1],
            current_world[2],
            self.roll,
            self.pitch,
            self.yaw,
        )
        rel_vx = self.vx_body - water_vx_body
        rel_vy = self.vy_body - water_vy_body
        rel_vz = self.vz_body - water_vz_body

        drag_x = self.linear_drag_xy * rel_vx + self.quad_drag_xy * rel_vx * abs(rel_vx)
        drag_y = self.linear_drag_xy * rel_vy + self.quad_drag_xy * rel_vy * abs(rel_vy)
        drag_z = self.linear_drag_z * rel_vz + self.quad_drag_z * rel_vz * abs(rel_vz)

        fx_body_total, fy_body_total, fz_body_total = world_to_body(
            fx_total_world,
            fy_total_world,
            fz_total_world,
            self.roll,
            self.pitch,
            self.yaw,
        )

        # 6. Hydrostatic righting moment. Offset buoyancy gives passive balance;
        # PD commands from the controller add active trim through thruster allocation.
        righting_roll = -(
            self.metacentric_roll_stiffness + buoyancy * self.center_of_buoyancy_z
        ) * math.sin(self.roll)
        righting_pitch = -(
            self.metacentric_pitch_stiffness + buoyancy * self.center_of_buoyancy_z
        ) * math.sin(self.pitch)
        drag_roll = (
            self.angular_drag_x * self.roll_rate
            + self.quad_angular_drag_xy * self.roll_rate * abs(self.roll_rate)
        )
        drag_pitch = (
            self.angular_drag_y * self.pitch_rate
            + self.quad_angular_drag_xy * self.pitch_rate * abs(self.pitch_rate)
        )
        drag_yaw = self.angular_drag_z * self.yaw_rate

        ax_body = (fx_body_total - drag_x) / self.eff_mass_x
        ay_body = (fy_body_total - drag_y) / self.eff_mass_y
        az_body = (fz_body_total - drag_z) / self.eff_mass_z
        roll_acc = (tx_cmd + righting_roll - drag_roll) / self.roll_inertia
        pitch_acc = (ty_cmd + righting_pitch - drag_pitch) / self.pitch_inertia
        yaw_acc = (tz_cmd + dtz - drag_yaw) / self.yaw_inertia
        self.latest_physics_status = {
            "depth": max(0.0, -self.z),
            "pressure": self._hydrostatic_pressure(),
            "weight": weight,
            "buoyancy": buoyancy,
            "net_buoyancy": buoyancy - weight,
            "thruster_body": (fx_cmd, fy_cmd, fz_cmd, tx_cmd, ty_cmd, tz_cmd),
            "current_body": (water_vx_body, water_vy_body, water_vz_body),
            "velocity_body": (self.vx_body, self.vy_body, self.vz_body),
            "relative_velocity_body": (rel_vx, rel_vy, rel_vz),
            "drag_resist_body": (-drag_x, -drag_y, -drag_z),
            "wave_world": (wfx, wfy, wfz),
            "disturbance_world": (dfx, dfy, dfz, dtz),
            "net_force_body": (
                fx_body_total - drag_x,
                fy_body_total - drag_y,
                fz_body_total - drag_z,
            ),
            "acceleration_body": (ax_body, ay_body, az_body),
            "righting_moment": (righting_roll, righting_pitch, 0.0),
            "angular_drag_resist": (-drag_roll, -drag_pitch, -drag_yaw),
        }

        # 7. Integrate body-frame velocity.
        self.vx_body = clamp(self.vx_body + ax_body * self.step_dt,
                              -self.max_linear_speed, self.max_linear_speed)
        self.vy_body = clamp(self.vy_body + ay_body * self.step_dt,
                              -self.max_linear_speed, self.max_linear_speed)
        self.vz_body = clamp(self.vz_body + az_body * self.step_dt,
                             -self.max_vertical_speed, self.max_vertical_speed)
        self.roll_rate = clamp(self.roll_rate + roll_acc * self.step_dt,
                               -self.max_roll_rate, self.max_roll_rate)
        self.pitch_rate = clamp(self.pitch_rate + pitch_acc * self.step_dt,
                                -self.max_pitch_rate, self.max_pitch_rate)
        self.yaw_rate = clamp(self.yaw_rate + yaw_acc * self.step_dt,
                              -self.max_yaw_rate, self.max_yaw_rate)

        # 8. Integrate world-frame position.
        vx_world, vy_world, vz_world = body_to_world(
            self.vx_body,
            self.vy_body,
            self.vz_body,
            self.roll,
            self.pitch,
            self.yaw,
        )

        self.x += vx_world * self.step_dt
        self.y += vy_world * self.step_dt
        self.z += vz_world * self.step_dt
        self.roll = clamp(
            wrap_angle(self.roll + self.roll_rate * self.step_dt),
            -self.max_attitude_angle,
            self.max_attitude_angle,
        )
        self.pitch = clamp(
            wrap_angle(self.pitch + self.pitch_rate * self.step_dt),
            -self.max_attitude_angle,
            self.max_attitude_angle,
        )
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * self.step_dt)

        # Keep the lightweight simulator inside the configured deep-water world.
        previous_z = self.z
        self.z = clamp(self.z, -self.water_depth + 0.5, -0.1)
        if self.z != previous_z:
            self.vz_body = 0.0

        # 9. Publish
        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(stamp)
        if self.publish_tf:
            self._publish_tf(stamp)
        self._publish_joint_states(stamp)
        self._publish_pressure()

    # ------------------------------------------------------------------
    def _publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx_body
        odom.twist.twist.linear.y = self.vy_body
        odom.twist.twist.linear.z = self.vz_body
        odom.twist.twist.angular.x = self.roll_rate
        odom.twist.twist.angular.y = self.pitch_rate
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)

    def _publish_tf(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_pressure(self):
        msg = Float64()
        msg.data = self._hydrostatic_pressure()
        self.pressure_pub.publish(msg)

    def _publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        msg.position = self.zero_joint_positions
        self.joint_state_pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleDynamicsSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
