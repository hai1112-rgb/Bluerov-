#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def clamp01(value):
    return max(0.0, min(1.0, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def quaternion_from_yaw(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class SimpleLocalizer(Node):
    def __init__(self):
        super().__init__("simple_localizer")

        defaults = {
            "imu_topic": "imu/data",
            "dvl_topic": "dvl/twist",
            "depth_topic": "depth",
            "output_odom_topic": "pose_gt",
            "pose_topic": "localized_pose",
            "path_topic": "localized_path",
            "world_frame": "world",
            "base_frame": "base_link",
            "publish_tf": True,
            "update_rate": 30.0,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_z": -200.0,
            "initial_yaw": 0.0,
            "depth_alpha": 0.5,
            "yaw_alpha": 0.3,
            "velocity_alpha": 0.45,
            "max_dt": 0.2,
            "sensor_timeout": 0.5,
            "max_path_length": 500,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.dvl_topic = str(self.get_parameter("dvl_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.initial_x = float(self.get_parameter("initial_x").value)
        self.initial_y = float(self.get_parameter("initial_y").value)
        self.initial_z = float(self.get_parameter("initial_z").value)
        self.initial_yaw = float(self.get_parameter("initial_yaw").value)
        self.depth_alpha = clamp01(float(self.get_parameter("depth_alpha").value))
        self.yaw_alpha = clamp01(float(self.get_parameter("yaw_alpha").value))
        self.velocity_alpha = clamp01(float(self.get_parameter("velocity_alpha").value))
        self.max_dt = max(1e-3, float(self.get_parameter("max_dt").value))
        self.sensor_timeout = max(0.0, float(self.get_parameter("sensor_timeout").value))
        self.max_path_length = max(1, int(self.get_parameter("max_path_length").value))

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, 10)
        self.create_subscription(TwistStamped, self.dvl_topic, self._dvl_callback, 10)
        self.create_subscription(Float64, self.depth_topic, self._depth_callback, 10)
        self.create_timer(1.0 / self.update_rate, self._update)

        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.yaw = self.initial_yaw
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.depth = max(0.0, -self.initial_z)
        self.last_imu_yaw = self.initial_yaw
        self.last_stamp = self.get_clock().now()
        self.last_sensor_stamp = self.last_stamp.to_msg()
        self.have_imu = False
        self.have_dvl = False
        self.have_depth = False
        self.path = Path()
        self.path.header.frame_id = self.world_frame

        self.get_logger().info(
            f"Simple localizer fusing {self.imu_topic}, {self.dvl_topic}, {self.depth_topic} into {self.output_odom_topic}"
        )
        self._publish(self.get_clock().now().to_msg())

    def _blend(self, current, measurement, alpha):
        return (1.0 - alpha) * current + alpha * measurement

    def _stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _imu_callback(self, msg):
        self.last_imu_yaw = yaw_from_quaternion(msg.orientation.z, msg.orientation.w)
        self.wz = self._blend(self.wz, msg.angular_velocity.z, self.velocity_alpha)
        self.last_sensor_stamp = msg.header.stamp
        self.have_imu = True

    def _dvl_callback(self, msg):
        self.vx = self._blend(self.vx, msg.twist.linear.x, self.velocity_alpha)
        self.vy = self._blend(self.vy, msg.twist.linear.y, self.velocity_alpha)
        self.vz = self._blend(self.vz, msg.twist.linear.z, self.velocity_alpha)
        self.last_sensor_stamp = msg.header.stamp
        self.have_dvl = True

    def _depth_callback(self, msg):
        self.depth = max(0.0, float(msg.data))
        self.have_depth = True

    def _update(self):
        now = self.get_clock().now()
        if not (self.have_imu and self.have_dvl and self.have_depth):
            self._publish(now.to_msg())
            return

        if self.sensor_timeout > 0.0:
            sensor_age = now.nanoseconds * 1e-9 - self._stamp_to_seconds(self.last_sensor_stamp)
            if sensor_age > self.sensor_timeout:
                self._publish(now.to_msg())
                return

        dt = (now.nanoseconds - self.last_stamp.nanoseconds) * 1e-9
        self.last_stamp = now
        if dt <= 0.0:
            return
        dt = min(dt, self.max_dt)

        yaw_error = wrap_angle(self.last_imu_yaw - self.yaw)
        self.yaw = wrap_angle(self.yaw + self.yaw_alpha * yaw_error)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        vx_world = cos_yaw * self.vx - sin_yaw * self.vy
        vy_world = sin_yaw * self.vx + cos_yaw * self.vy

        self.x += vx_world * dt
        self.y += vy_world * dt
        self.z = self._blend(self.z, -self.depth, self.depth_alpha)

        self._publish(now.to_msg())

    def _publish(self, stamp):
        qz, qw = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.04
        odom.pose.covariance[7] = 0.04
        odom.pose.covariance[14] = 0.02
        odom.pose.covariance[35] = 0.03
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom.header
        pose_msg.pose = odom.pose
        self.pose_pub.publish(pose_msg)

        if self.publish_tf and self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = self.z
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        self.path.header = odom.header
        path_pose = PoseStamped()
        path_pose.header = odom.header
        path_pose.pose = odom.pose.pose
        self.path.poses.append(path_pose)
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = SimpleLocalizer()
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
