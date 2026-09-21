#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


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


class SimpleSensorSuite(Node):
    def __init__(self):
        super().__init__("simple_sensor_suite")

        defaults = {
            "input_odom_topic": "sim/pose_gt",
            "imu_topic": "imu/data",
            "dvl_topic": "dvl/twist",
            "depth_topic": "depth",
            "imu_yaw_bias": 0.01,
            "imu_yaw_rate_bias": 0.005,
            "dvl_scale_xy": 0.985,
            "dvl_scale_z": 0.99,
            "depth_bias": 0.015,
            "max_accel_dt": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.dvl_topic = str(self.get_parameter("dvl_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.imu_yaw_bias = float(self.get_parameter("imu_yaw_bias").value)
        self.imu_yaw_rate_bias = float(self.get_parameter("imu_yaw_rate_bias").value)
        self.dvl_scale_xy = float(self.get_parameter("dvl_scale_xy").value)
        self.dvl_scale_z = float(self.get_parameter("dvl_scale_z").value)
        self.depth_bias = float(self.get_parameter("depth_bias").value)
        self.max_accel_dt = max(1e-6, float(self.get_parameter("max_accel_dt").value))

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.dvl_pub = self.create_publisher(TwistStamped, self.dvl_topic, 10)
        self.depth_pub = self.create_publisher(Float64, self.depth_topic, 10)
        self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self._odom_callback,
            10,
        )

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0
        self.prev_stamp = None

        self.get_logger().info("Simple sensor suite started")

    def _stamp_dt(self, stamp):
        if self.prev_stamp is None:
            return 0.0
        return (stamp.sec - self.prev_stamp.sec) + (stamp.nanosec - self.prev_stamp.nanosec) * 1e-9

    def _odom_callback(self, msg):
        stamp = msg.header.stamp
        dt = self._stamp_dt(stamp)

        roll, pitch, yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        yaw += self.imu_yaw_bias
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z + self.imu_yaw_rate_bias
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = msg.child_frame_id or "base_link"
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.02
        imu.angular_velocity.x = wx
        imu.angular_velocity.y = wy
        imu.angular_velocity.z = wz
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        if 1e-6 < dt <= self.max_accel_dt:
            imu.linear_acceleration.x = (vx - self.prev_vx) / dt
            imu.linear_acceleration.y = (vy - self.prev_vy) / dt
            imu.linear_acceleration.z = (vz - self.prev_vz) / dt
        imu.linear_acceleration_covariance[0] = 0.03
        imu.linear_acceleration_covariance[4] = 0.03
        imu.linear_acceleration_covariance[8] = 0.03
        self.imu_pub.publish(imu)

        dvl = TwistStamped()
        dvl.header = msg.header
        dvl.header.frame_id = msg.child_frame_id or "base_link"
        dvl.twist.linear.x = vx * self.dvl_scale_xy
        dvl.twist.linear.y = vy * self.dvl_scale_xy
        dvl.twist.linear.z = vz * self.dvl_scale_z
        dvl.twist.angular.z = wz
        self.dvl_pub.publish(dvl)

        depth = Float64()
        depth.data = max(0.0, -msg.pose.pose.position.z + self.depth_bias)
        self.depth_pub.publish(depth)

        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vz = vz
        self.prev_stamp = stamp


def main():
    rclpy.init()
    node = SimpleSensorSuite()
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
