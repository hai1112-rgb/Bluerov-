#!/usr/bin/env python3

import math
import struct
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


POINT_STRUCT = struct.Struct("<ffff")
POINT_STEP_BYTES = POINT_STRUCT.size


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def yaw_from_quaternion(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


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


def rotate_xy(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def ray_circle_distance(origin_x, origin_y, dir_x, dir_y, center_x, center_y, radius):
    if radius <= 0.0:
        return None

    dx = origin_x - center_x
    dy = origin_y - center_y
    b = 2.0 * (dx * dir_x + dy * dir_y)
    c = dx * dx + dy * dy - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None

    sqrt_discriminant = math.sqrt(discriminant)
    first = (-b - sqrt_discriminant) * 0.5
    second = (-b + sqrt_discriminant) * 0.5
    candidates = [distance for distance in (first, second) if distance >= 0.0]
    if not candidates:
        return None
    return min(candidates)


def ray_box_distance(origin_x, origin_y, dir_x, dir_y, center_x, center_y, size_x, size_y):
    half_x = 0.5 * max(0.0, size_x)
    half_y = 0.5 * max(0.0, size_y)
    min_x = center_x - half_x
    max_x = center_x + half_x
    min_y = center_y - half_y
    max_y = center_y + half_y

    t_min = -math.inf
    t_max = math.inf

    if abs(dir_x) < 1e-9:
        if origin_x < min_x or origin_x > max_x:
            return None
    else:
        tx1 = (min_x - origin_x) / dir_x
        tx2 = (max_x - origin_x) / dir_x
        t_min = max(t_min, min(tx1, tx2))
        t_max = min(t_max, max(tx1, tx2))

    if abs(dir_y) < 1e-9:
        if origin_y < min_y or origin_y > max_y:
            return None
    else:
        ty1 = (min_y - origin_y) / dir_y
        ty2 = (max_y - origin_y) / dir_y
        t_min = max(t_min, min(ty1, ty2))
        t_max = min(t_max, max(ty1, ty2))

    if t_max < max(t_min, 0.0):
        return None
    if t_min >= 0.0:
        return t_min
    if t_max >= 0.0:
        return t_max
    return None


def ray_box_distance_3d(origin, direction, center, size):
    t_min = -math.inf
    t_max = math.inf

    for origin_value, direction_value, center_value, size_value in zip(origin, direction, center, size):
        half = 0.5 * max(0.0, size_value)
        min_value = center_value - half
        max_value = center_value + half

        if abs(direction_value) < 1e-9:
            if origin_value < min_value or origin_value > max_value:
                return None
            continue

        t1 = (min_value - origin_value) / direction_value
        t2 = (max_value - origin_value) / direction_value
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))

    if t_max < max(t_min, 0.0):
        return None
    if t_min >= 0.0:
        return t_min
    if t_max >= 0.0:
        return t_max
    return None


def ray_sphere_distance_3d(origin, direction, center, radius):
    if radius <= 0.0:
        return None

    ox, oy, oz = origin
    dx, dy, dz = direction
    cx, cy, cz = center
    px = ox - cx
    py = oy - cy
    pz = oz - cz
    b = 2.0 * (px * dx + py * dy + pz * dz)
    c = px * px + py * py + pz * pz - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None

    sqrt_discriminant = math.sqrt(discriminant)
    first = (-b - sqrt_discriminant) * 0.5
    second = (-b + sqrt_discriminant) * 0.5
    candidates = [distance for distance in (first, second) if distance >= 0.0]
    if not candidates:
        return None
    return min(candidates)


def ray_cylinder_z_distance_3d(origin, direction, center_x, center_y, min_z, max_z, radius):
    if radius <= 0.0:
        return None

    ox, oy, oz = origin
    dx, dy, dz = direction
    distances = []

    a = dx * dx + dy * dy
    if a > 1e-9:
        px = ox - center_x
        py = oy - center_y
        b = 2.0 * (px * dx + py * dy)
        c = px * px + py * py - radius * radius
        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0.0:
            sqrt_discriminant = math.sqrt(discriminant)
            for distance in (
                (-b - sqrt_discriminant) / (2.0 * a),
                (-b + sqrt_discriminant) / (2.0 * a),
            ):
                z = oz + distance * dz
                if distance >= 0.0 and min_z <= z <= max_z:
                    distances.append(distance)

    if abs(dz) > 1e-9:
        for cap_z in (min_z, max_z):
            distance = (cap_z - oz) / dz
            if distance >= 0.0:
                x = ox + distance * dx
                y = oy + distance * dy
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius * radius:
                    distances.append(distance)

    if not distances:
        return None
    return min(distances)


def make_point_cloud(frame_id, stamp, points):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = POINT_STEP_BYTES
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    data = bytearray(msg.row_step)
    for index, (x, y, z, intensity) in enumerate(points):
        POINT_STRUCT.pack_into(
            data,
            index * POINT_STEP_BYTES,
            float(x),
            float(y),
            float(z),
            float(intensity),
        )
    msg.data = bytes(data)
    return msg


class SimpleSonarSim(Node):
    def __init__(self):
        super().__init__("simple_sonar_sim")

        defaults = {
            "input_odom_topic": "sim/pose_gt",
            "scene_file": "",
            "update_rate": 12.0,
            "base_frame": "base_link",
            "forward_frame": "base_link_forward_sonar",
            "downward_frame": "base_link_downward_sonar",
            "forward_topic": "sonar/forward",
            "downward_topic": "sonar/downward",
            "spherical_cloud_topic": "sonar/spherical_cloud",
            "terrain_cloud_topic": "sonar/terrain_cloud",
            "forward_min_angle": -0.7853981634,
            "forward_max_angle": 0.7853981634,
            "forward_samples": 61,
            "forward_min_range": 0.25,
            "forward_max_range": 25.0,
            "downward_min_range": 0.10,
            "downward_max_range": 220.0,
            "enable_spherical_sonar": True,
            "spherical_azimuth_samples": 96,
            "spherical_elevation_samples": 25,
            "spherical_min_elevation": -1.3962634016,
            "spherical_max_elevation": 0.5235987756,
            "spherical_min_range": 0.25,
            "spherical_max_range": 220.0,
            "terrain_voxel_size": 0.15,
            "terrain_max_points": 60000,
            "forward_sensor_x": 0.24,
            "forward_sensor_y": 0.0,
            "forward_sensor_z": 0.0,
            "downward_sensor_x": 0.0,
            "downward_sensor_y": 0.0,
            "downward_sensor_z": -0.08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.forward_frame = str(self.get_parameter("forward_frame").value)
        self.downward_frame = str(self.get_parameter("downward_frame").value)
        self.forward_topic = str(self.get_parameter("forward_topic").value)
        self.downward_topic = str(self.get_parameter("downward_topic").value)
        self.spherical_cloud_topic = str(self.get_parameter("spherical_cloud_topic").value)
        self.terrain_cloud_topic = str(self.get_parameter("terrain_cloud_topic").value)
        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.forward_min_angle = float(self.get_parameter("forward_min_angle").value)
        self.forward_max_angle = float(self.get_parameter("forward_max_angle").value)
        self.forward_samples = max(1, int(self.get_parameter("forward_samples").value))
        self.forward_min_range = max(0.01, float(self.get_parameter("forward_min_range").value))
        self.forward_max_range = max(self.forward_min_range, float(self.get_parameter("forward_max_range").value))
        self.downward_min_range = max(0.01, float(self.get_parameter("downward_min_range").value))
        self.downward_max_range = max(
            self.downward_min_range,
            float(self.get_parameter("downward_max_range").value),
        )
        self.enable_spherical_sonar = bool(self.get_parameter("enable_spherical_sonar").value)
        self.spherical_azimuth_samples = max(8, int(self.get_parameter("spherical_azimuth_samples").value))
        self.spherical_elevation_samples = max(3, int(self.get_parameter("spherical_elevation_samples").value))
        self.spherical_min_elevation = clamp(
            float(self.get_parameter("spherical_min_elevation").value),
            -math.pi / 2.0,
            math.pi / 2.0,
        )
        self.spherical_max_elevation = clamp(
            float(self.get_parameter("spherical_max_elevation").value),
            self.spherical_min_elevation,
            math.pi / 2.0,
        )
        self.spherical_min_range = max(0.01, float(self.get_parameter("spherical_min_range").value))
        self.spherical_max_range = max(
            self.spherical_min_range,
            float(self.get_parameter("spherical_max_range").value),
        )
        self.terrain_voxel_size = max(0.01, float(self.get_parameter("terrain_voxel_size").value))
        self.terrain_max_points = max(1000, int(self.get_parameter("terrain_max_points").value))
        self.forward_sensor_offset = (
            float(self.get_parameter("forward_sensor_x").value),
            float(self.get_parameter("forward_sensor_y").value),
            float(self.get_parameter("forward_sensor_z").value),
        )
        self.downward_sensor_offset = (
            float(self.get_parameter("downward_sensor_x").value),
            float(self.get_parameter("downward_sensor_y").value),
            float(self.get_parameter("downward_sensor_z").value),
        )
        # Static scan geometry is reused every timer tick; cache it once.
        self.forward_scan_angles = self._build_forward_scan_angles()
        self.spherical_elevations = self._build_spherical_elevations()
        azimuth_step = 2.0 * math.pi / self.spherical_azimuth_samples
        self.spherical_azimuth_offsets = [
            index * azimuth_step for index in range(self.spherical_azimuth_samples)
        ]

        scene_file_value = str(self.get_parameter("scene_file").value).strip()
        scene_file = Path(scene_file_value) if scene_file_value else None
        self.scene = {}
        if scene_file is not None:
            with scene_file.open("r", encoding="utf-8") as stream:
                self.scene = yaml.safe_load(stream) or {}

        self.latest_odom = None
        self.terrain_points = {}
        self.forward_pub = self.create_publisher(LaserScan, self.forward_topic, 10)
        self.downward_pub = self.create_publisher(LaserScan, self.downward_topic, 10)
        self.spherical_cloud_pub = self.create_publisher(PointCloud2, self.spherical_cloud_topic, 10)
        self.terrain_cloud_pub = self.create_publisher(PointCloud2, self.terrain_cloud_topic, 1)
        self.create_subscription(Odometry, self.input_odom_topic, self._odom_callback, 10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_sensor_frames()
        self.create_timer(1.0 / self.update_rate, self._publish_scans)

        obstacle_count = len(self.scene.get("obstacles", []))
        self.get_logger().info(
            "Simple sonar sim started using %s with %d obstacle(s); spherical cloud=%s"
            % (
                scene_file if scene_file is not None else "<empty scene>",
                obstacle_count,
                "on" if self.enable_spherical_sonar else "off",
            )
        )

    def _build_forward_scan_angles(self):
        if self.forward_samples == 1:
            return [0.0]

        increment = (
            self.forward_max_angle - self.forward_min_angle
        ) / (self.forward_samples - 1)
        return [
            self.forward_min_angle + index * increment
            for index in range(self.forward_samples)
        ]

    def _build_spherical_elevations(self):
        if self.spherical_elevation_samples == 1:
            return [0.0]

        elevation_step = (
            self.spherical_max_elevation - self.spherical_min_elevation
        ) / (self.spherical_elevation_samples - 1)
        return [
            self.spherical_min_elevation + index * elevation_step
            for index in range(self.spherical_elevation_samples)
        ]

    def _publish_sensor_frames(self):
        forward = TransformStamped()
        forward.header.stamp = self.get_clock().now().to_msg()
        forward.header.frame_id = self.base_frame
        forward.child_frame_id = self.forward_frame
        forward.transform.translation.x = self.forward_sensor_offset[0]
        forward.transform.translation.y = self.forward_sensor_offset[1]
        forward.transform.translation.z = self.forward_sensor_offset[2]
        forward.transform.rotation.w = 1.0

        downward = TransformStamped()
        downward.header.stamp = forward.header.stamp
        downward.header.frame_id = self.base_frame
        downward.child_frame_id = self.downward_frame
        downward.transform.translation.x = self.downward_sensor_offset[0]
        downward.transform.translation.y = self.downward_sensor_offset[1]
        downward.transform.translation.z = self.downward_sensor_offset[2]
        qx, qy, qz, qw = quaternion_from_euler(0.0, -math.pi / 2.0, 0.0)
        downward.transform.rotation.x = qx
        downward.transform.rotation.y = qy
        downward.transform.rotation.z = qz
        downward.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform([forward, downward])

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _forward_sensor_origin(self, pose_x, pose_y, pose_z, yaw):
        offset_x, offset_y = rotate_xy(
            self.forward_sensor_offset[0],
            self.forward_sensor_offset[1],
            yaw,
        )
        return (
            pose_x + offset_x,
            pose_y + offset_y,
            pose_z + self.forward_sensor_offset[2],
        )

    def _downward_sensor_origin(self, pose_x, pose_y, pose_z, yaw):
        offset_x, offset_y = rotate_xy(
            self.downward_sensor_offset[0],
            self.downward_sensor_offset[1],
            yaw,
        )
        return (
            pose_x + offset_x,
            pose_y + offset_y,
            pose_z + self.downward_sensor_offset[2],
        )

    def _box_forward_distance(self, obstacle, origin_x, origin_y, origin_z, dir_x, dir_y):
        center_z = float(obstacle.get("z", 0.0))
        size_z = float(obstacle.get("size_z", 1.0))
        min_z = center_z - 0.5 * size_z
        max_z = center_z + 0.5 * size_z
        if origin_z < min_z or origin_z > max_z:
            return None
        return ray_box_distance(
            origin_x,
            origin_y,
            dir_x,
            dir_y,
            float(obstacle.get("x", 0.0)),
            float(obstacle.get("y", 0.0)),
            float(obstacle.get("size_x", 1.0)),
            float(obstacle.get("size_y", 1.0)),
        )

    def _cylinder_forward_distance(self, obstacle, origin_x, origin_y, origin_z, dir_x, dir_y):
        center_z = float(obstacle.get("z", 0.0))
        height = float(obstacle.get("height", obstacle.get("size_z", 1.0)))
        min_z = center_z - 0.5 * height
        max_z = center_z + 0.5 * height
        if origin_z < min_z or origin_z > max_z:
            return None
        radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size_x", 1.0)))
        return ray_circle_distance(
            origin_x,
            origin_y,
            dir_x,
            dir_y,
            float(obstacle.get("x", 0.0)),
            float(obstacle.get("y", 0.0)),
            radius,
        )

    def _sphere_forward_distance(self, obstacle, origin_x, origin_y, origin_z, dir_x, dir_y):
        center_z = float(obstacle.get("z", 0.0))
        radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size", 1.0)))
        vertical_delta = abs(origin_z - center_z)
        if vertical_delta > radius:
            return None
        slice_radius = math.sqrt(max(0.0, radius * radius - vertical_delta * vertical_delta))
        return ray_circle_distance(
            origin_x,
            origin_y,
            dir_x,
            dir_y,
            float(obstacle.get("x", 0.0)),
            float(obstacle.get("y", 0.0)),
            slice_radius,
        )

    def _forward_distance_for_obstacle(self, obstacle, origin_x, origin_y, origin_z, dir_x, dir_y):
        obstacle_type = str(obstacle.get("type", "box")).lower()
        if obstacle_type == "cylinder":
            return self._cylinder_forward_distance(
                obstacle,
                origin_x,
                origin_y,
                origin_z,
                dir_x,
                dir_y,
            )
        if obstacle_type == "sphere":
            return self._sphere_forward_distance(
                obstacle,
                origin_x,
                origin_y,
                origin_z,
                dir_x,
                dir_y,
            )
        return self._box_forward_distance(
            obstacle,
            origin_x,
            origin_y,
            origin_z,
            dir_x,
            dir_y,
        )

    def _point_inside_box(self, point_x, point_y, obstacle):
        half_x = 0.5 * float(obstacle.get("size_x", 1.0))
        half_y = 0.5 * float(obstacle.get("size_y", 1.0))
        center_x = float(obstacle.get("x", 0.0))
        center_y = float(obstacle.get("y", 0.0))
        return (
            center_x - half_x <= point_x <= center_x + half_x
            and center_y - half_y <= point_y <= center_y + half_y
        )

    def _downward_distance_for_obstacle(self, obstacle, origin_x, origin_y, origin_z):
        obstacle_type = str(obstacle.get("type", "box")).lower()

        if obstacle_type == "cylinder":
            center_x = float(obstacle.get("x", 0.0))
            center_y = float(obstacle.get("y", 0.0))
            radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size_x", 1.0)))
            dx = origin_x - center_x
            dy = origin_y - center_y
            if dx * dx + dy * dy > radius * radius:
                return None
            top_z = float(obstacle.get("z", 0.0)) + 0.5 * float(
                obstacle.get("height", obstacle.get("size_z", 1.0))
            )
        elif obstacle_type == "sphere":
            center_x = float(obstacle.get("x", 0.0))
            center_y = float(obstacle.get("y", 0.0))
            center_z = float(obstacle.get("z", 0.0))
            radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size", 1.0)))
            dx = origin_x - center_x
            dy = origin_y - center_y
            radial_sq = dx * dx + dy * dy
            if radial_sq > radius * radius:
                return None
            top_z = center_z + math.sqrt(max(0.0, radius * radius - radial_sq))
        else:
            if not self._point_inside_box(origin_x, origin_y, obstacle):
                return None
            top_z = float(obstacle.get("z", 0.0)) + 0.5 * float(obstacle.get("size_z", 1.0))

        if top_z > origin_z:
            return None
        return origin_z - top_z

    def _forward_range(self, pose_x, pose_y, pose_z, yaw, relative_angle):
        origin_x, origin_y, origin_z = self._forward_sensor_origin(pose_x, pose_y, pose_z, yaw)
        world_angle = yaw + relative_angle
        dir_x = math.cos(world_angle)
        dir_y = math.sin(world_angle)

        best_distance = self.forward_max_range
        for obstacle in self.scene.get("obstacles", []):
            distance = self._forward_distance_for_obstacle(
                obstacle,
                origin_x,
                origin_y,
                origin_z,
                dir_x,
                dir_y,
            )
            if distance is not None:
                best_distance = min(best_distance, distance)

        return clamp(best_distance, self.forward_min_range, self.forward_max_range)

    def _downward_range(self, pose_x, pose_y, pose_z, yaw):
        origin_x, origin_y, origin_z = self._downward_sensor_origin(pose_x, pose_y, pose_z, yaw)
        best_distance = self.downward_max_range

        seabed = self.scene.get("seabed", {})
        seabed_top = float(seabed.get("z", -1.3)) + 0.5 * float(seabed.get("thickness", 0.35))
        if seabed_top <= origin_z:
            best_distance = min(best_distance, origin_z - seabed_top)

        for obstacle in self.scene.get("obstacles", []):
            distance = self._downward_distance_for_obstacle(
                obstacle,
                origin_x,
                origin_y,
                origin_z,
            )
            if distance is not None:
                best_distance = min(best_distance, distance)

        return clamp(best_distance, self.downward_min_range, self.downward_max_range)

    def _spherical_sensor_origin(self, pose_x, pose_y, pose_z, yaw):
        offset_x, offset_y = rotate_xy(
            self.downward_sensor_offset[0],
            self.downward_sensor_offset[1],
            yaw,
        )
        return (
            pose_x + offset_x,
            pose_y + offset_y,
            pose_z + self.downward_sensor_offset[2],
        )

    def _terrain_plane_distance(self, origin, direction):
        seabed = self.scene.get("seabed", {})
        seabed_top = float(seabed.get("z", -1.3)) + 0.5 * float(seabed.get("thickness", 0.35))
        dz = direction[2]
        if abs(dz) < 1e-9:
            return None
        distance = (seabed_top - origin[2]) / dz
        if distance < 0.0:
            return None

        hit_x = origin[0] + distance * direction[0]
        hit_y = origin[1] + distance * direction[1]
        half_x = 0.5 * float(seabed.get("size_x", 18.0))
        half_y = 0.5 * float(seabed.get("size_y", 18.0))
        center_x = float(seabed.get("x", 0.0))
        center_y = float(seabed.get("y", 0.0))
        if center_x - half_x <= hit_x <= center_x + half_x and center_y - half_y <= hit_y <= center_y + half_y:
            return distance
        return None

    def _spherical_distance_for_obstacle(self, obstacle, origin, direction):
        obstacle_type = str(obstacle.get("type", "box")).lower()
        center = (
            float(obstacle.get("x", 0.0)),
            float(obstacle.get("y", 0.0)),
            float(obstacle.get("z", 0.0)),
        )

        if obstacle_type == "cylinder":
            height = float(obstacle.get("height", obstacle.get("size_z", 1.0)))
            min_z = center[2] - 0.5 * height
            max_z = center[2] + 0.5 * height
            radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size_x", 1.0)))
            return ray_cylinder_z_distance_3d(origin, direction, center[0], center[1], min_z, max_z, radius)

        if obstacle_type == "sphere":
            radius = 0.5 * float(obstacle.get("diameter", obstacle.get("size", 1.0)))
            return ray_sphere_distance_3d(origin, direction, center, radius)

        size = (
            float(obstacle.get("size_x", 1.0)),
            float(obstacle.get("size_y", 1.0)),
            float(obstacle.get("size_z", 1.0)),
        )
        return ray_box_distance_3d(origin, direction, center, size)

    def _spherical_hit(self, origin, direction):
        best_distance = None
        best_type = None

        terrain_distance = self._terrain_plane_distance(origin, direction)
        if (
            terrain_distance is not None
            and self.spherical_min_range <= terrain_distance <= self.spherical_max_range
        ):
            best_distance = terrain_distance
            best_type = "terrain"

        for obstacle in self.scene.get("obstacles", []):
            distance = self._spherical_distance_for_obstacle(obstacle, origin, direction)
            if distance is None:
                continue
            if not (self.spherical_min_range <= distance <= self.spherical_max_range):
                continue
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_type = str(obstacle.get("type", "obstacle")).lower()

        if best_distance is None:
            return None

        x = origin[0] + best_distance * direction[0]
        y = origin[1] + best_distance * direction[1]
        z = origin[2] + best_distance * direction[2]
        intensity_by_type = {
            "terrain": 90.0,
            "box": 120.0,
            "cylinder": 170.0,
            "sphere": 210.0,
        }
        intensity = intensity_by_type.get(best_type, 140.0)
        intensity *= 1.0 - 0.35 * (best_distance / self.spherical_max_range)
        return (x, y, z, max(1.0, intensity))

    def _build_spherical_cloud(self, pose_x, pose_y, pose_z, yaw):
        origin = self._spherical_sensor_origin(pose_x, pose_y, pose_z, yaw)
        points = []

        for elevation in self.spherical_elevations:
            cos_elevation = math.cos(elevation)
            sin_elevation = math.sin(elevation)
            for azimuth_offset in self.spherical_azimuth_offsets:
                azimuth = yaw + azimuth_offset
                direction = (
                    cos_elevation * math.cos(azimuth),
                    cos_elevation * math.sin(azimuth),
                    sin_elevation,
                )
                hit = self._spherical_hit(origin, direction)
                if hit is not None:
                    points.append(hit)

        return points

    def _update_terrain_map(self, points):
        for point in points:
            key = (
                round(point[0] / self.terrain_voxel_size),
                round(point[1] / self.terrain_voxel_size),
                round(point[2] / self.terrain_voxel_size),
            )
            self.terrain_points[key] = point

        if len(self.terrain_points) <= self.terrain_max_points:
            return

        overflow = len(self.terrain_points) - self.terrain_max_points
        for key in list(self.terrain_points.keys())[:overflow]:
            del self.terrain_points[key]

    def _publish_spherical_clouds(self, stamp, pose_x, pose_y, pose_z, yaw):
        if not self.enable_spherical_sonar:
            return

        points = self._build_spherical_cloud(pose_x, pose_y, pose_z, yaw)
        self._update_terrain_map(points)
        self.spherical_cloud_pub.publish(make_point_cloud("world", stamp, points))
        self.terrain_cloud_pub.publish(
            make_point_cloud("world", stamp, list(self.terrain_points.values()))
        )

    def _publish_forward_scan(self, stamp, pose_x, pose_y, pose_z, yaw):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.forward_frame
        msg.angle_min = self.forward_min_angle
        msg.angle_max = self.forward_max_angle
        msg.range_min = self.forward_min_range
        msg.range_max = self.forward_max_range
        if self.forward_samples == 1:
            msg.angle_increment = 0.0
        else:
            msg.angle_increment = (
                self.forward_max_angle - self.forward_min_angle
            ) / (self.forward_samples - 1)
        msg.scan_time = 1.0 / self.update_rate
        msg.time_increment = 0.0
        msg.ranges = [
            self._forward_range(pose_x, pose_y, pose_z, yaw, angle)
            for angle in self.forward_scan_angles
        ]
        self.forward_pub.publish(msg)

    def _publish_downward_scan(self, stamp, pose_x, pose_y, pose_z, yaw):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.downward_frame
        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 0.0
        msg.range_min = self.downward_min_range
        msg.range_max = self.downward_max_range
        msg.scan_time = 1.0 / self.update_rate
        msg.time_increment = 0.0
        msg.ranges = [self._downward_range(pose_x, pose_y, pose_z, yaw)]
        self.downward_pub.publish(msg)

    def _publish_scans(self):
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation.z, pose.orientation.w)
        stamp = self.get_clock().now().to_msg()
        self._publish_forward_scan(
            stamp,
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            yaw,
        )
        self._publish_downward_scan(
            stamp,
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            yaw,
        )
        self._publish_spherical_clouds(
            stamp,
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            yaw,
        )


def main():
    rclpy.init()
    node = SimpleSonarSim()
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
