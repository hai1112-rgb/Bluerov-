#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Wrench
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float
    qz: float
    qw: float


@dataclass
class ForwardSonarState:
    min_range: float
    left_clearance: float
    right_clearance: float
    received_time: float


@dataclass
class AxisIntegrator:
    value: float = 0.0

    def reset(self):
        self.value = 0.0


class WaypointAutopilot(Node):
    def __init__(self):
        super().__init__("waypoint_autopilot")

        defaults = {
            "update_rate": 20.0,
            "odom_topic": "pose_gt",
            "wrench_topic": "thruster_manager/input",
            "goal_tolerance_xy": 0.2,
            "goal_tolerance_z": 0.15,
            "goal_tolerance_yaw": 0.15,
            "loop_waypoints": False,
            "hold_last_waypoint": True,
            "start_at_current_position": False,
            "interpolate_path": True,
            "path_resolution": 0.3,
            "publish_path_topic": "planned_path",
            "enable_click_to_go": True,
            "clicked_point_topic": "/clicked_point",
            "goal_point_topic": "/goal_point",
            "goal_pose_topic": "/goal_pose",
            "click_use_current_depth": False,
            "click_target_depth": -205.0,
            "click_face_goal": True,
            "enable_depth_setpoint": True,
            "depth_setpoint_topic": "/depth_setpoint",
            "surface_depth_margin": 0.1,
            "enable_forward_sonar_avoidance": True,
            "forward_sonar_topic": "sonar/forward",
            "forward_sonar_sector_half_angle": 0.55,
            "forward_sonar_stop_distance": 1.1,
            "forward_sonar_clear_distance": 2.4,
            "forward_sonar_timeout": 1.0,
            "forward_sonar_reverse_force": 12.0,
            "forward_sonar_lateral_force": 26.0,
            "forward_sonar_yaw_gain": 14.0,
            "kp_xy": 70.0,
            "kd_xy": 35.0,
            "kp_z": 110.0,
            "ki_z": 2.0,
            "kd_z": 55.0,
            "kp_yaw": 35.0,
            "ki_yaw": 0.8,
            "kd_yaw": 14.0,
            "kp_roll": 12.0,
            "ki_roll": 0.4,
            "kd_roll": 4.0,
            "kp_pitch": 12.0,
            "ki_pitch": 0.2,
            "kd_pitch": 4.0,
            "depth_feedforward_force": 0.0,
            "integral_limit_z": 4.0,
            "integral_limit_yaw": 0.7,
            "integral_limit_roll": 0.35,
            "integral_limit_pitch": 0.25,
            "max_force_xy": 80.0,
            "max_force_z": 120.0,
            "max_torque_xy": 20.0,
            "max_torque_z": 25.0,
            "waypoints": ["0.0, 0.0, -205.0, 0.0"],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.wrench_topic = str(self.get_parameter("wrench_topic").value)
        self.goal_tolerance_xy = max(0.0, float(self.get_parameter("goal_tolerance_xy").value))
        self.goal_tolerance_z = max(0.0, float(self.get_parameter("goal_tolerance_z").value))
        self.goal_tolerance_yaw = max(0.0, float(self.get_parameter("goal_tolerance_yaw").value))
        self.loop_waypoints = bool(self.get_parameter("loop_waypoints").value)
        self.hold_last_waypoint = bool(self.get_parameter("hold_last_waypoint").value)
        self.start_at_current_position = bool(self.get_parameter("start_at_current_position").value)
        self.interpolate_path = bool(self.get_parameter("interpolate_path").value)
        self.path_resolution = max(0.05, float(self.get_parameter("path_resolution").value))
        self.publish_path_topic = str(self.get_parameter("publish_path_topic").value)
        self.enable_click_to_go = bool(self.get_parameter("enable_click_to_go").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.goal_point_topic = str(self.get_parameter("goal_point_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.click_use_current_depth = bool(self.get_parameter("click_use_current_depth").value)
        self.click_target_depth = float(self.get_parameter("click_target_depth").value)
        self.click_face_goal = bool(self.get_parameter("click_face_goal").value)
        self.enable_depth_setpoint = bool(self.get_parameter("enable_depth_setpoint").value)
        self.depth_setpoint_topic = str(self.get_parameter("depth_setpoint_topic").value)
        self.surface_depth_margin = max(0.05, float(self.get_parameter("surface_depth_margin").value))
        self.enable_forward_sonar_avoidance = bool(self.get_parameter("enable_forward_sonar_avoidance").value)
        self.forward_sonar_topic = str(self.get_parameter("forward_sonar_topic").value)
        self.forward_sonar_sector_half_angle = clamp(
            float(self.get_parameter("forward_sonar_sector_half_angle").value),
            0.1,
            math.pi / 2.0,
        )
        self.forward_sonar_stop_distance = max(
            0.2,
            float(self.get_parameter("forward_sonar_stop_distance").value),
        )
        self.forward_sonar_clear_distance = max(
            self.forward_sonar_stop_distance + 0.1,
            float(self.get_parameter("forward_sonar_clear_distance").value),
        )
        self.forward_sonar_timeout = max(0.1, float(self.get_parameter("forward_sonar_timeout").value))
        self.forward_sonar_reverse_force = max(
            0.0,
            float(self.get_parameter("forward_sonar_reverse_force").value),
        )
        self.forward_sonar_lateral_force = max(
            0.0,
            float(self.get_parameter("forward_sonar_lateral_force").value),
        )
        self.forward_sonar_yaw_gain = max(
            0.0,
            float(self.get_parameter("forward_sonar_yaw_gain").value),
        )
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kd_xy = float(self.get_parameter("kd_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.ki_z = float(self.get_parameter("ki_z").value)
        self.kd_z = float(self.get_parameter("kd_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.ki_yaw = float(self.get_parameter("ki_yaw").value)
        self.kd_yaw = float(self.get_parameter("kd_yaw").value)
        self.kp_roll = float(self.get_parameter("kp_roll").value)
        self.ki_roll = float(self.get_parameter("ki_roll").value)
        self.kd_roll = float(self.get_parameter("kd_roll").value)
        self.kp_pitch = float(self.get_parameter("kp_pitch").value)
        self.ki_pitch = float(self.get_parameter("ki_pitch").value)
        self.kd_pitch = float(self.get_parameter("kd_pitch").value)
        self.depth_feedforward_force = float(self.get_parameter("depth_feedforward_force").value)
        self.integral_limit_z = max(0.0, float(self.get_parameter("integral_limit_z").value))
        self.integral_limit_yaw = max(0.0, float(self.get_parameter("integral_limit_yaw").value))
        self.integral_limit_roll = max(0.0, float(self.get_parameter("integral_limit_roll").value))
        self.integral_limit_pitch = max(0.0, float(self.get_parameter("integral_limit_pitch").value))
        self.max_force_xy = max(0.0, float(self.get_parameter("max_force_xy").value))
        self.max_force_z = max(0.0, float(self.get_parameter("max_force_z").value))
        self.max_torque_xy = max(0.0, float(self.get_parameter("max_torque_xy").value))
        self.max_torque_z = max(0.0, float(self.get_parameter("max_torque_z").value))

        self.odom = None
        self.last_control_time = None
        self.z_integrator = AxisIntegrator()
        self.yaw_integrator = AxisIntegrator()
        self.roll_integrator = AxisIntegrator()
        self.pitch_integrator = AxisIntegrator()
        self.completed = False
        self.current_waypoint_index = 0
        self.active_loop_waypoints = self.loop_waypoints
        self.zero_wrench = Wrench()
        self.forward_sonar_state = None
        self.last_avoidance_turn = 1.0
        self.forward_path_blocked = False
        self.mission_waypoints = self._load_waypoints()
        self.waypoints = []
        self.path_templates = []
        self._set_guidance_path(self.mission_waypoints, self.loop_waypoints)

        self.wrench_pub = self.create_publisher(Wrench, self.wrench_topic, 10)
        self.path_pub = self.create_publisher(Path, self.publish_path_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        if self.enable_click_to_go:
            self.create_subscription(PointStamped, self.clicked_point_topic, self._clicked_point_callback, 10)
            self.create_subscription(Point, self.goal_point_topic, self._goal_point_callback, 10)
            self.create_subscription(PoseStamped, self.goal_pose_topic, self._goal_pose_callback, 10)
        if self.enable_depth_setpoint:
            self.create_subscription(Float64, self.depth_setpoint_topic, self._depth_setpoint_callback, 10)
        if self.enable_forward_sonar_avoidance:
            self.create_subscription(LaserScan, self.forward_sonar_topic, self._forward_sonar_callback, 10)
        self.create_timer(1.0 / self.update_rate, self._control_loop)
        self.create_timer(1.0, self._publish_path)

        if self.mission_waypoints:
            self.get_logger().info(
                f"Loaded {len(self.mission_waypoints)} mission waypoint(s), expanded to {len(self.waypoints)} guidance point(s)"
            )
        elif self.start_at_current_position:
            self.get_logger().warning(
                "No waypoints configured; the controller will hold the current pose once odometry arrives"
            )
        else:
            self.get_logger().warning(
                "No waypoints configured; the controller will remain idle until waypoints are provided"
            )
        if self.enable_click_to_go:
            self.get_logger().info(
                "Click-to-go enabled on topics: %s, %s, %s"
                % (self.clicked_point_topic, self.goal_point_topic, self.goal_pose_topic)
            )
        if self.enable_depth_setpoint:
            self.get_logger().info(f"Depth setpoint enabled on topic: {self.depth_setpoint_topic}")
        if self.enable_forward_sonar_avoidance:
            self.get_logger().info(
                "Forward sonar avoidance enabled on topic: %s (stop=%.2fm clear=%.2fm)"
                % (
                    self.forward_sonar_topic,
                    self.forward_sonar_stop_distance,
                    self.forward_sonar_clear_distance,
                )
            )

    def _make_waypoint(self, x, y, z, yaw):
        yaw = wrap_angle(float(yaw))
        half_yaw = yaw * 0.5
        return Waypoint(
            x=float(x),
            y=float(y),
            z=float(z),
            yaw=yaw,
            qz=math.sin(half_yaw),
            qw=math.cos(half_yaw),
        )

    def _load_waypoints(self):
        parsed = []
        try:
            raw_waypoints = self.get_parameter("waypoints").value
        except ParameterUninitializedException:
            return parsed
        for index, waypoint in enumerate(raw_waypoints):
            try:
                if isinstance(waypoint, str):
                    values = [float(item.strip()) for item in waypoint.split(",")]
                    if len(values) != 4:
                        raise ValueError("string waypoint must have x,y,z,yaw")
                    parsed.append(self._make_waypoint(*values))
                    continue

                if isinstance(waypoint, (list, tuple)):
                    if len(waypoint) != 4:
                        raise ValueError("list waypoint must have four numeric values")
                    parsed.append(self._make_waypoint(*waypoint))
                    continue

                if isinstance(waypoint, dict):
                    yaw = waypoint.get("yaw")
                    if yaw is None and "yaw_deg" in waypoint:
                        yaw = math.radians(float(waypoint["yaw_deg"]))
                    parsed.append(
                        self._make_waypoint(
                            waypoint["x"],
                            waypoint["y"],
                            waypoint["z"],
                            0.0 if yaw is None else yaw,
                        )
                    )
                    continue

                raise TypeError("unsupported waypoint type")
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(f"Ignoring malformed waypoint {index}: {waypoint}")
        return parsed

    def _build_guidance_path(self, waypoints):
        if not waypoints:
            return []
        if not self.interpolate_path or len(waypoints) == 1:
            return list(waypoints)

        expanded = [waypoints[0]]
        for start, end in zip(waypoints[:-1], waypoints[1:]):
            dx = end.x - start.x
            dy = end.y - start.y
            dz = end.z - start.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            steps = max(1, int(math.ceil(distance / self.path_resolution)))
            yaw_delta = wrap_angle(end.yaw - start.yaw)
            for step in range(1, steps + 1):
                ratio = step / steps
                expanded.append(
                    self._make_waypoint(
                        start.x + dx * ratio,
                        start.y + dy * ratio,
                        start.z + dz * ratio,
                        start.yaw + yaw_delta * ratio,
                    )
                )
        return expanded

    def _build_path_templates(self, waypoints):
        poses = []
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = waypoint.z
            pose.pose.orientation.z = waypoint.qz
            pose.pose.orientation.w = waypoint.qw
            poses.append(pose)
        return poses

    def _set_guidance_path(self, mission_waypoints, loop_enabled):
        self.mission_waypoints = list(mission_waypoints)
        self.waypoints = self._build_guidance_path(self.mission_waypoints)
        self.path_templates = self._build_path_templates(self.waypoints)
        self.current_waypoint_index = 0
        self.completed = False
        self.active_loop_waypoints = bool(loop_enabled)
        self._reset_balance_integrators()

    def _reset_balance_integrators(self):
        self.z_integrator.reset()
        self.yaw_integrator.reset()
        self.roll_integrator.reset()
        self.pitch_integrator.reset()
        self.last_control_time = None

    def _publish_path(self):
        if not self.path_templates:
            return

        path = Path()
        path.header.frame_id = "world" if self.odom is None else self.odom.header.frame_id or "world"
        path.header.stamp = self.get_clock().now().to_msg()
        for template in self.path_templates:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = template.pose
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _odom_callback(self, msg):
        self.odom = msg
        if not self.mission_waypoints and self.start_at_current_position:
            pose = msg.pose.pose
            _, _, yaw = euler_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            self.mission_waypoints = [
                self._make_waypoint(
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    yaw,
                )
            ]
            self._set_guidance_path(self.mission_waypoints, self.loop_waypoints)
            self.get_logger().info("Initialized hold-position waypoint from the current vehicle pose")
            self._publish_path()

    def _accept_goal(self, target_x, target_y, target_z, target_yaw=None):
        if self.odom is None:
            self.get_logger().warning("Ignoring goal point because odometry is not available yet")
            return

        pose = self.odom.pose.pose
        _, _, current_yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        target_x = float(target_x)
        target_y = float(target_y)
        if target_z is None or self.click_use_current_depth:
            target_z = float(pose.position.z)
        else:
            target_z = float(target_z) if abs(float(target_z)) > 1e-6 else self.click_target_depth

        yaw_was_provided = target_yaw is not None
        if target_yaw is None:
            target_yaw = current_yaw
        if self.click_face_goal and not yaw_was_provided:
            dx = target_x - float(pose.position.x)
            dy = target_y - float(pose.position.y)
            if math.hypot(dx, dy) > 1e-6:
                target_yaw = math.atan2(dy, dx)

        target_waypoint = self._make_waypoint(target_x, target_y, target_z, target_yaw)
        self._set_guidance_path([target_waypoint], loop_enabled=False)
        self._publish_path()
        self.get_logger().info(
            "New click target accepted: x=%.2f y=%.2f z=%.2f yaw=%.2f"
            % (target_waypoint.x, target_waypoint.y, target_waypoint.z, target_waypoint.yaw)
        )

    def _clicked_point_callback(self, msg):
        self._accept_goal(msg.point.x, msg.point.y, msg.point.z)

    def _goal_point_callback(self, msg):
        self._accept_goal(msg.x, msg.y, msg.z)

    def _goal_pose_callback(self, msg):
        _, _, yaw = euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self._accept_goal(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, yaw)

    def _depth_setpoint_callback(self, msg):
        if self.odom is None:
            self.get_logger().warning("Ignoring depth setpoint because odometry is not available yet")
            return

        pose = self.odom.pose.pose
        _, _, current_yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        requested_depth = max(0.0, float(msg.data))
        target_z = -max(self.surface_depth_margin, requested_depth)
        target_waypoint = self._make_waypoint(
            pose.position.x,
            pose.position.y,
            target_z,
            current_yaw,
        )
        self._set_guidance_path([target_waypoint], loop_enabled=False)
        self._publish_path()
        self.get_logger().info(
            "New depth target accepted: depth=%.2f m (z=%.2f)"
            % (requested_depth, target_waypoint.z)
        )

    def _publish_zero_wrench(self):
        self._reset_balance_integrators()
        self.wrench_pub.publish(self.zero_wrench)

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _control_dt(self):
        now = self._now_seconds()
        if self.last_control_time is None:
            self.last_control_time = now
            return 1.0 / self.update_rate
        dt = clamp(now - self.last_control_time, 1e-4, 0.5)
        self.last_control_time = now
        return dt

    def _pid_effort(self, error, rate, kp, ki, kd, integrator, integral_limit, output_limit, feedforward=0.0, dt=0.0):
        proposed_integral = clamp(
            integrator.value + error * dt,
            -integral_limit,
            integral_limit,
        )
        raw_output = kp * error + ki * proposed_integral - kd * rate + feedforward
        output = clamp(raw_output, -output_limit, output_limit)

        saturated_high = raw_output > output_limit and error > 0.0
        saturated_low = raw_output < -output_limit and error < 0.0
        if not (saturated_high or saturated_low):
            integrator.value = proposed_integral
        else:
            raw_output = kp * error + ki * integrator.value - kd * rate + feedforward
            output = clamp(raw_output, -output_limit, output_limit)

        return output

    def _forward_sonar_callback(self, msg):
        left_clearance = msg.range_max
        right_clearance = msg.range_max
        sector_ranges = []
        left_samples = 0
        right_samples = 0

        for index, raw_value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) > self.forward_sonar_sector_half_angle:
                continue

            if math.isnan(raw_value):
                continue
            if math.isinf(raw_value):
                if raw_value <= 0.0:
                    continue
                value = msg.range_max
            else:
                value = clamp(raw_value, msg.range_min, msg.range_max)

            sector_ranges.append(value)
            if angle >= 0.0:
                left_clearance = min(left_clearance, value)
                left_samples += 1
            else:
                right_clearance = min(right_clearance, value)
                right_samples += 1

        if not sector_ranges:
            self.forward_sonar_state = None
            if self.forward_path_blocked:
                self.forward_path_blocked = False
                self.get_logger().info("Forward sonar data lost; reverting to nominal guidance")
            return

        min_range = min(sector_ranges)
        if left_samples == 0:
            left_clearance = min_range
        if right_samples == 0:
            right_clearance = min_range

        self.forward_sonar_state = ForwardSonarState(
            min_range=min_range,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
            received_time=self._now_seconds(),
        )

        blocked = min_range <= self.forward_sonar_stop_distance
        if blocked != self.forward_path_blocked:
            self.forward_path_blocked = blocked
            if blocked:
                self.get_logger().warning(
                    "Forward sonar sees obstacle at %.2fm; slowing and steering around it" % min_range
                )
            else:
                self.get_logger().info("Forward sonar path cleared; resuming nominal guidance")

    def _get_fresh_forward_sonar(self):
        if not self.enable_forward_sonar_avoidance or self.forward_sonar_state is None:
            return None

        if self._now_seconds() - self.forward_sonar_state.received_time > self.forward_sonar_timeout:
            if self.forward_path_blocked:
                self.forward_path_blocked = False
                self.get_logger().info("Forward sonar timed out; reverting to nominal guidance")
            return None
        return self.forward_sonar_state

    def _apply_forward_sonar_avoidance(self, wrench, dx_body, dy_body, yaw_error):
        sonar_state = self._get_fresh_forward_sonar()
        if sonar_state is None or sonar_state.min_range >= self.forward_sonar_clear_distance:
            return

        clearance_span = max(
            self.forward_sonar_clear_distance - self.forward_sonar_stop_distance,
            1e-6,
        )
        slow_ratio = clamp(
            (sonar_state.min_range - self.forward_sonar_stop_distance) / clearance_span,
            0.0,
            1.0,
        )
        avoidance_strength = 1.0 - slow_ratio

        steering_bias = clamp(
            (sonar_state.left_clearance - sonar_state.right_clearance)
            / max(self.forward_sonar_clear_distance, 1e-6),
            -1.0,
            1.0,
        )
        if abs(steering_bias) < 0.08:
            steering_bias = clamp(
                dy_body / max(self.forward_sonar_clear_distance, 1.0),
                -1.0,
                1.0,
            )
        if abs(steering_bias) < 0.08:
            steering_bias = clamp(yaw_error / math.pi, -1.0, 1.0)
        if abs(steering_bias) < 0.08:
            steering_bias = 1.0 if self.last_avoidance_turn >= 0.0 else -1.0
        self.last_avoidance_turn = steering_bias

        if dx_body > 0.0 and wrench.force.x > 0.0:
            wrench.force.x = min(
                wrench.force.x,
                self.max_force_xy * max(0.0, slow_ratio),
            )
            if sonar_state.min_range <= self.forward_sonar_stop_distance:
                wrench.force.x = min(
                    wrench.force.x,
                    -self.forward_sonar_reverse_force * avoidance_strength,
                )

        wrench.force.y = clamp(
            wrench.force.y + self.forward_sonar_lateral_force * steering_bias * avoidance_strength,
            -self.max_force_xy,
            self.max_force_xy,
        )
        wrench.torque.z = clamp(
            wrench.torque.z + self.forward_sonar_yaw_gain * steering_bias * avoidance_strength,
            -self.max_torque_z,
            self.max_torque_z,
        )

    def _advance_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.waypoints) - 1:
                self.get_logger().info("Approaching final guidance point")
            return

        if self.active_loop_waypoints and self.waypoints:
            self.current_waypoint_index = 0
            self.get_logger().info("Restarting path loop from guidance point 0")
            return

        if not self.hold_last_waypoint:
            self.completed = True
            self.get_logger().info("Waypoint mission completed; stopping controller output")

    def _target_errors(self, waypoint, pose, yaw):
        dx_world = waypoint.x - pose.position.x
        dy_world = waypoint.y - pose.position.y
        dz_world = waypoint.z - pose.position.z
        yaw_error = wrap_angle(waypoint.yaw - yaw)
        return dx_world, dy_world, dz_world, yaw_error

    def _control_loop(self):
        if self.odom is None or not self.waypoints or self.completed:
            self._publish_zero_wrench()
            return

        dt = self._control_dt()
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        roll, pitch, yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        waypoint = self.waypoints[self.current_waypoint_index]
        dx_world, dy_world, dz_world, yaw_error = self._target_errors(waypoint, pose, yaw)

        if (
            math.hypot(dx_world, dy_world) <= self.goal_tolerance_xy
            and abs(dz_world) <= self.goal_tolerance_z
            and abs(yaw_error) <= self.goal_tolerance_yaw
        ):
            self._advance_waypoint()
            if self.completed:
                self._publish_zero_wrench()
                return
            waypoint = self.waypoints[self.current_waypoint_index]
            dx_world, dy_world, dz_world, yaw_error = self._target_errors(waypoint, pose, yaw)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx_body = cos_yaw * dx_world + sin_yaw * dy_world
        dy_body = -sin_yaw * dx_world + cos_yaw * dy_world

        wrench = Wrench()
        wrench.force.x = clamp(
            self.kp_xy * dx_body - self.kd_xy * twist.linear.x,
            -self.max_force_xy,
            self.max_force_xy,
        )
        wrench.force.y = clamp(
            self.kp_xy * dy_body - self.kd_xy * twist.linear.y,
            -self.max_force_xy,
            self.max_force_xy,
        )
        wrench.force.z = clamp(
            self._pid_effort(
                dz_world,
                twist.linear.z,
                self.kp_z,
                self.ki_z,
                self.kd_z,
                self.z_integrator,
                self.integral_limit_z,
                self.max_force_z,
                self.depth_feedforward_force,
                dt,
            ),
            -self.max_force_z,
            self.max_force_z,
        )
        wrench.torque.x = clamp(
            self._pid_effort(
                -roll,
                twist.angular.x,
                self.kp_roll,
                self.ki_roll,
                self.kd_roll,
                self.roll_integrator,
                self.integral_limit_roll,
                self.max_torque_xy,
                0.0,
                dt,
            ),
            -self.max_torque_xy,
            self.max_torque_xy,
        )
        wrench.torque.y = clamp(
            self._pid_effort(
                -pitch,
                twist.angular.y,
                self.kp_pitch,
                self.ki_pitch,
                self.kd_pitch,
                self.pitch_integrator,
                self.integral_limit_pitch,
                self.max_torque_xy,
                0.0,
                dt,
            ),
            -self.max_torque_xy,
            self.max_torque_xy,
        )
        wrench.torque.z = clamp(
            self._pid_effort(
                yaw_error,
                twist.angular.z,
                self.kp_yaw,
                self.ki_yaw,
                self.kd_yaw,
                self.yaw_integrator,
                self.integral_limit_yaw,
                self.max_torque_z,
                0.0,
                dt,
            ),
            -self.max_torque_z,
            self.max_torque_z,
        )
        self._apply_forward_sonar_avoidance(wrench, dx_body, dy_body, yaw_error)
        self.wrench_pub.publish(wrench)


def main():
    rclpy.init()
    node = WaypointAutopilot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
