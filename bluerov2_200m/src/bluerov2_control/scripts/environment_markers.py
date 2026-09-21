#!/usr/bin/env python3

from pathlib import Path

from geometry_msgs.msg import Point
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def rgba(values, default):
    color = list(default if values is None else values)
    if len(color) == 3:
        color.append(1.0)
    while len(color) < 4:
        color.append(1.0)
    return [float(component) for component in color[:4]]


class EnvironmentMarkers(Node):
    def __init__(self):
        super().__init__("environment_markers")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("marker_topic", "environment_markers")
        self.declare_parameter("scene_file", "")
        self.declare_parameter("publish_rate", 0.5)
        self.declare_parameter("show_axes", True)
        self.declare_parameter("axis_length", 6.0)
        self.declare_parameter("axis_radius", 0.08)
        self.declare_parameter("axis_label_scale", 0.7)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.show_axes = bool(self.get_parameter("show_axes").value)
        self.axis_length = max(0.1, float(self.get_parameter("axis_length").value))
        self.axis_radius = max(0.01, float(self.get_parameter("axis_radius").value))
        self.axis_label_scale = max(0.1, float(self.get_parameter("axis_label_scale").value))
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            qos,
        )

        scene_file = Path(str(self.get_parameter("scene_file").value))
        with scene_file.open("r", encoding="utf-8") as stream:
            self.scene = yaml.safe_load(stream) or {}

        self.markers = self._build_markers()
        self.create_timer(1.0 / max(0.1, float(self.get_parameter("publish_rate").value)), self._publish)
        self._publish()
        self.get_logger().info(
            f"Published {len(self.markers.markers)} environment markers from {scene_file}"
        )

    def _make_marker(self, marker_id, marker_def):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = str(marker_def.get("ns", "environment"))
        marker.id = int(marker_id)
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(marker_def.get("x", 0.0))
        marker.pose.position.y = float(marker_def.get("y", 0.0))
        marker.pose.position.z = float(marker_def.get("z", 0.0))

        marker.pose.orientation.x = float(marker_def.get("qx", 0.0))
        marker.pose.orientation.y = float(marker_def.get("qy", 0.0))
        marker.pose.orientation.z = float(marker_def.get("qz", 0.0))
        marker.pose.orientation.w = float(marker_def.get("qw", 1.0))

        marker_type = str(marker_def.get("type", "box")).lower()
        if marker_type == "cylinder":
            marker.type = Marker.CYLINDER
            marker.scale.x = float(marker_def.get("diameter", marker_def.get("size_x", 1.0)))
            marker.scale.y = float(marker_def.get("diameter", marker_def.get("size_y", 1.0)))
            marker.scale.z = float(marker_def.get("height", marker_def.get("size_z", 1.0)))
        elif marker_type == "sphere":
            marker.type = Marker.SPHERE
            diameter = float(marker_def.get("diameter", marker_def.get("size", 1.0)))
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter
        else:
            marker.type = Marker.CUBE
            marker.scale.x = float(marker_def.get("size_x", 1.0))
            marker.scale.y = float(marker_def.get("size_y", 1.0))
            marker.scale.z = float(marker_def.get("size_z", 1.0))

        r, g, b, a = rgba(marker_def.get("color"), [0.6, 0.6, 0.6, 1.0])
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    def _point(self, x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _set_color(self, marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    def _make_axis_arrow(self, marker_id, end_point, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "world_axes"
        marker.id = int(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [self._point(0.0, 0.0, 0.0), self._point(*end_point)]
        marker.scale.x = self.axis_radius
        marker.scale.y = self.axis_radius * 3.0
        marker.scale.z = self.axis_radius * 4.5
        self._set_color(marker, color)
        return marker

    def _make_axis_label(self, marker_id, label, position, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "world_axes"
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.scale.z = self.axis_label_scale
        marker.text = str(label)
        self._set_color(marker, color)
        return marker

    def _append_world_axes(self, markers, next_id):
        length = self.axis_length
        label_offset = length + (self.axis_label_scale * 0.9)
        axes = [
            ("X", (length, 0.0, 0.0), (label_offset, 0.0, 0.0), (1.0, 0.1, 0.1, 1.0)),
            ("Y", (0.0, length, 0.0), (0.0, label_offset, 0.0), (0.1, 1.0, 0.1, 1.0)),
            ("Z", (0.0, 0.0, length), (0.0, 0.0, label_offset), (0.2, 0.45, 1.0, 1.0)),
        ]
        for label, end_point, label_position, color in axes:
            markers.markers.append(self._make_axis_arrow(next_id, end_point, color))
            next_id += 1
            markers.markers.append(self._make_axis_label(next_id, label, label_position, color))
            next_id += 1
        return next_id

    def _build_markers(self):
        markers = MarkerArray()
        next_id = 0

        seabed = self.scene.get("seabed", {})
        seabed_marker = {
            "ns": "seabed",
            "type": "box",
            "x": float(seabed.get("x", 0.0)),
            "y": float(seabed.get("y", 0.0)),
            "z": float(seabed.get("z", -1.3)),
            "size_x": float(seabed.get("size_x", 18.0)),
            "size_y": float(seabed.get("size_y", 18.0)),
            "size_z": float(seabed.get("thickness", 0.35)),
            "color": seabed.get("color", [0.42, 0.34, 0.22, 1.0]),
        }
        markers.markers.append(self._make_marker(next_id, seabed_marker))
        next_id += 1

        for obstacle in self.scene.get("obstacles", []):
            markers.markers.append(self._make_marker(next_id, obstacle))
            next_id += 1

        if self.show_axes:
            next_id = self._append_world_axes(markers, next_id)

        return markers

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.marker_pub.publish(self.markers)


def main():
    rclpy.init()
    node = EnvironmentMarkers()
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
