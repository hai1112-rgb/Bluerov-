#!/usr/bin/env python3
"""Drive the visual fish actors in the Gazebo deep-water world."""

import math
import random

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String


class FishAgent:
    """Trajectory generator for one visual fish actor."""

    PATTERNS = ['circle', 'figure8', 'drift', 'patrol']
    MIN_Z = -218.0
    MAX_Z = -185.0

    def __init__(self, fish_id: int, center_x: float, center_y: float,
                 center_z: float, pattern: str = None):
        self.id = fish_id
        self.name = f'fish_{fish_id}'

        self.cx = float(center_x)
        self.cy = float(center_y)
        self.cz = float(clamp(float(center_z), self.MIN_Z, self.MAX_Z))

        self.pattern = pattern or random.choice(self.PATTERNS)

        self.radius   = random.uniform(1.8, 5.0)
        self.speed    = random.uniform(0.025, 0.075)
        self.phase    = random.uniform(0, 2 * math.pi)
        self.v_amp    = random.uniform(0.15, 0.65)
        self.v_freq   = random.uniform(0.025, 0.08)
        self.drift_vx = random.uniform(-0.035, 0.035)
        self.drift_vy = random.uniform(-0.035, 0.035)

        self.x = float(self.cx)
        self.y = float(self.cy)
        self.z = float(self.cz)
        self.yaw = 0.0

    def update(self, t: float, storm_mode: bool, dt: float) -> tuple:
        """Update and return (x, y, z, yaw)."""
        phase = self.phase + t * self.speed

        storm_jitter = 0.0
        if storm_mode:
            storm_jitter = 0.10 * math.sin(t * 1.2 + self.phase * 2)

        if self.pattern == 'circle':
            self.x = self.cx + self.radius * math.cos(phase)
            self.y = self.cy + self.radius * math.sin(phase)
            self.yaw = phase + math.pi / 2 + storm_jitter
            self.z = self.cz + self.v_amp * math.sin(t * self.v_freq)

        elif self.pattern == 'figure8':
            self.x = self.cx + self.radius * math.sin(phase)
            self.y = self.cy + self.radius * 0.5 * math.sin(2 * phase)
            self.yaw = math.atan2(
                self.radius * math.cos(2 * phase),
                self.radius * math.cos(phase)
            ) + storm_jitter
            self.z = self.cz + self.v_amp * math.sin(t * self.v_freq * 1.5)

        elif self.pattern == 'drift':
            self.x += (self.drift_vx + storm_jitter * 0.1) * dt
            self.y += self.drift_vy * dt
            if abs(self.x - self.cx) > 15:
                self.x = self.cx
            if abs(self.y - self.cy) > 15:
                self.y = self.cy
            self.yaw = math.atan2(self.drift_vy, self.drift_vx) + storm_jitter
            self.z = self.cz + self.v_amp * math.sin(t * self.v_freq)

        elif self.pattern == 'patrol':
            segment = (t * self.speed) % (2 * math.pi)
            if segment < math.pi:
                self.x = self.cx + self.radius * (segment / math.pi * 2 - 1)
                self.y = self.cy
                self.yaw = 0.0 + storm_jitter
            else:
                self.x = self.cx + self.radius * (1 - (segment - math.pi) / math.pi * 2)
                self.y = self.cy
                self.yaw = math.pi + storm_jitter
            self.z = self.cz + self.v_amp * math.sin(t * self.v_freq)

        self.z = clamp(self.z, self.MIN_Z, self.MAX_Z)

        return self.x, self.y, self.z, self.yaw


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_pose_msg(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = float(sy)
    pose.orientation.w = float(cy)
    return pose


class FishControllerNode(Node):
    def __init__(self):
        super().__init__('fish_controller')

        fish_configs = [
            (5.0,    3.0, -195.0, 'circle'),
            (-4.0,   6.0, -197.0, 'figure8'),
            (4.0,   -5.0, -198.0, 'circle'),
            (-8.0,  -3.0, -196.0, 'drift'),
            (8.0,   -8.0, -202.0, 'patrol'),
            (-6.0,  12.0, -200.0, 'figure8'),
            (13.0,   5.0, -204.0, 'circle'),
            (-12.0, -10.0, -199.0, 'drift'),
        ]

        self.fish_agents = []
        for i, (cx, cy, cz, pat) in enumerate(fish_configs):
            agent = FishAgent(i, cx, cy, cz, pat)
            self.fish_agents.append(agent)

        self.pose_pubs = {}
        for agent in self.fish_agents:
            topic = f'/model/{agent.name}/pose_cmd'
            self.pose_pubs[agent.name] = self.create_publisher(Pose, topic, 10)

        self.storm_mode = False
        self.create_subscription(String, '/disturbance/mode',
                                 self._disturbance_cb, 10)

        self.update_hz = 6.0
        self.dt = 1.0 / self.update_hz
        self.sim_time = 0.0
        self.create_timer(self.dt, self._update_fish)

        self.get_logger().info(
            f'Fish Controller started: {len(self.fish_agents)} fish')
        self.get_logger().info(
            '  Patterns: ' + ', '.join(
                f'{a.name}={a.pattern}' for a in self.fish_agents))
        self.get_logger().info(
            '  Chill deep-water motion: slow speed, low vertical amplitude, soft heading changes')

    def _disturbance_cb(self, msg: String):
        self.storm_mode = (msg.data.strip().lower() == 'storm')

    def _update_fish(self):
        self.sim_time += self.dt
        for agent in self.fish_agents:
            x, y, z, yaw = agent.update(self.sim_time, self.storm_mode, self.dt)
            pose_msg = make_pose_msg(x, y, z, yaw)
            self.pose_pubs[agent.name].publish(pose_msg)


def main():
    rclpy.init()
    node = FishControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
