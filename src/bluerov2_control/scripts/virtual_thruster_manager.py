#!/usr/bin/env python3

from pathlib import Path

import numpy
import rclpy
import yaml
from geometry_msgs.msg import Wrench
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class VirtualThrusterManager(Node):
    def __init__(self):
        super().__init__("virtual_thruster_manager")

        self.declare_parameter("tam_file", "")
        self.declare_parameter("thruster_count", 6)
        self.declare_parameter("max_thrust", 1540.0)
        self.declare_parameter("input_topic", "thruster_manager/input")
        self.declare_parameter("commanded_wrench_topic", "thruster_manager/commanded_wrench")
        self.declare_parameter("thruster_topic_prefix", "thrusters")
        self.declare_parameter("thruster_topic_suffix", "input")
        self.declare_parameter("allocation_report_topic", "thruster_manager/allocation_report")
        self.declare_parameter("diagnostics_rate", 2.0)
        self.declare_parameter("log_allocation", False)
        self.declare_parameter("print_thruster_status", False)
        self.declare_parameter("thruster_status_deadband", 0.05)
        self.declare_parameter("allocation_method", "weighted_damped_least_squares")
        self.declare_parameter("allocation_damping", 0.05)
        self.declare_parameter("dof_weights", [1.0, 1.0, 1.2, 0.9, 0.2, 1.0])
        self.declare_parameter("command_smoothing_alpha", 1.0)

        tam_file = Path(str(self.get_parameter("tam_file").value))
        self.tam = self._load_tam(tam_file)
        self.inverse_tam = numpy.linalg.pinv(self.tam)
        tam_thruster_count = int(self.tam.shape[1])
        configured_thruster_count = int(self.get_parameter("thruster_count").value)
        if configured_thruster_count != tam_thruster_count:
            self.get_logger().warning(
                "Configured thruster_count=%d does not match TAM columns=%d; using TAM value"
                % (configured_thruster_count, tam_thruster_count)
            )
        self.thruster_count = tam_thruster_count
        self.max_thrust = max(0.0, float(self.get_parameter("max_thrust").value))
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.commanded_wrench_topic = str(self.get_parameter("commanded_wrench_topic").value)
        self.allocation_report_topic = str(self.get_parameter("allocation_report_topic").value)
        self.diagnostics_rate = max(0.0, float(self.get_parameter("diagnostics_rate").value))
        self.log_allocation = as_bool(self.get_parameter("log_allocation").value)
        self.print_thruster_status = as_bool(self.get_parameter("print_thruster_status").value)
        self.thruster_status_deadband = max(
            0.0,
            float(self.get_parameter("thruster_status_deadband").value),
        )
        self.allocation_method = str(self.get_parameter("allocation_method").value).strip().lower()
        self.allocation_damping = max(0.0, float(self.get_parameter("allocation_damping").value))
        self.dof_weights = self._load_dof_weights(self.get_parameter("dof_weights").value)
        self.weighted_lhs, self.weighted_rhs_matrix = self._build_weighted_allocator()
        self.command_smoothing_alpha = clamp(
            float(self.get_parameter("command_smoothing_alpha").value),
            0.0,
            1.0,
        )

        prefix = str(self.get_parameter("thruster_topic_prefix").value).strip("/")
        suffix = str(self.get_parameter("thruster_topic_suffix").value).strip("/")

        self.commanded_wrench_pub = self.create_publisher(Wrench, self.commanded_wrench_topic, 10)
        self.allocation_report_pub = self.create_publisher(String, self.allocation_report_topic, 10)
        self.thruster_vector_pub = self.create_publisher(
            Float64MultiArray,
            "thruster_manager/thrust_setpoints",
            10,
        )
        self.thruster_publishers = []
        for index in range(self.thruster_count):
            topic = f"{prefix}/thruster_{index}/{suffix}"
            self.thruster_publishers.append(self.create_publisher(Float64, topic, 10))

        self.desired_wrench = numpy.zeros(6, dtype=float)
        self.latest_thruster_forces = numpy.zeros(self.thruster_count, dtype=float)
        self.previous_thruster_forces = numpy.zeros(self.thruster_count, dtype=float)
        self.latest_achieved_wrench = numpy.zeros(6, dtype=float)
        self.latest_error = numpy.zeros(6, dtype=float)
        self.latest_report = ""
        self.thruster_roles = self._describe_thruster_roles()
        self.create_subscription(Wrench, self.input_topic, self._input_callback, 10)
        if self.diagnostics_rate > 0.0:
            self.create_timer(1.0 / self.diagnostics_rate, self._publish_allocation_report)
        self.get_logger().info(
            "Virtual thruster manager ready with TAM from %s | method=%s | roles: %s"
            % (tam_file, self.allocation_method, "; ".join(self.thruster_roles))
        )

    def _load_tam(self, tam_file):
        if not tam_file.is_file():
            raise FileNotFoundError(f"TAM file not found: {tam_file}")

        with tam_file.open("r", encoding="utf-8") as stream:
            tam_yaml = yaml.safe_load(stream) or {}

        if "tam" not in tam_yaml:
            raise ValueError(f"Missing 'tam' key in TAM file: {tam_file}")

        tam = numpy.array(tam_yaml["tam"], dtype=float)
        if tam.ndim != 2:
            raise ValueError(f"TAM must be a 2D matrix, got shape {tam.shape}")
        if tam.shape[0] != 6:
            raise ValueError(f"TAM must have 6 rows for wrench DOFs, got {tam.shape[0]}")
        if tam.shape[1] <= 0:
            raise ValueError("TAM must contain at least one thruster column")
        return tam

    def _load_dof_weights(self, values):
        weights = numpy.array(list(values), dtype=float)
        if weights.size != 6:
            raise ValueError(f"dof_weights must contain 6 values, got {weights.size}")
        return numpy.maximum(weights, 0.0)

    def _build_weighted_allocator(self):
        """Precompute static matrices used by weighted damped least-squares."""
        weight_matrix = numpy.diag(self.dof_weights)
        weighted_tam = weight_matrix.dot(self.tam)
        lhs = weighted_tam.T.dot(weighted_tam)
        if self.allocation_damping > 0.0:
            lhs += (self.allocation_damping ** 2) * numpy.eye(self.thruster_count)
        rhs_matrix = weighted_tam.T.dot(weight_matrix)
        return lhs, rhs_matrix

    def _describe_thruster_roles(self):
        dof_names = ("surge", "sway", "heave", "roll", "pitch", "yaw")
        roles = []
        for index in range(self.thruster_count):
            column = self.tam[:, index]
            dominant = numpy.argsort(numpy.abs(column))[::-1][:2]
            role = "+".join(dof_names[dof] for dof in dominant if abs(column[dof]) > 1e-9)
            if not role:
                role = "unused"
            roles.append(f"T{index}:{role}")
        return roles

    def _format_vector(self, values, precision=2):
        return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"

    def _format_thruster_status(self):
        if self.max_thrust > 1e-9:
            percentages = 100.0 * self.latest_thruster_forces / self.max_thrust
        else:
            percentages = numpy.zeros(self.thruster_count, dtype=float)

        active_count = int(
            numpy.count_nonzero(numpy.abs(self.latest_thruster_forces) > self.thruster_status_deadband)
        )
        peak_force = float(numpy.max(numpy.abs(self.latest_thruster_forces)))
        thrusters = " ".join(
            "T%d=%+.2fN(%+.1f%%)" % (index, force, percent)
            for index, (force, percent) in enumerate(zip(self.latest_thruster_forces, percentages))
        )
        return (
            "[THRUSTER STATUS] active=%d/%d peak=%.2fN | %s | achieved_wrench=%s"
            % (
                active_count,
                self.thruster_count,
                peak_force,
                thrusters,
                self._format_vector(self.latest_achieved_wrench),
            )
        )

    def _allocate_thrusters(self, desired_wrench):
        if self.allocation_method in ("pinv", "pseudo_inverse", "pseudoinverse"):
            return self.inverse_tam.dot(desired_wrench)

        rhs = self.weighted_rhs_matrix.dot(desired_wrench)
        try:
            return numpy.linalg.solve(self.weighted_lhs, rhs)
        except numpy.linalg.LinAlgError:
            return numpy.linalg.lstsq(self.weighted_lhs, rhs, rcond=None)[0]

    def _publish_allocation_report(self):
        if not self.latest_report:
            return
        msg = String()
        msg.data = self.latest_report
        self.allocation_report_pub.publish(msg)
        if self.log_allocation:
            self.get_logger().info(self.latest_report)
        if self.print_thruster_status:
            self.get_logger().info(self._format_thruster_status())

    def _input_callback(self, msg):
        self.desired_wrench[0] = msg.force.x
        self.desired_wrench[1] = msg.force.y
        self.desired_wrench[2] = msg.force.z
        self.desired_wrench[3] = msg.torque.x
        self.desired_wrench[4] = msg.torque.y
        self.desired_wrench[5] = msg.torque.z

        thruster_forces = self._allocate_thrusters(self.desired_wrench)
        thruster_forces = numpy.clip(thruster_forces, -self.max_thrust, self.max_thrust)
        if self.command_smoothing_alpha < 1.0:
            thruster_forces = (
                self.command_smoothing_alpha * thruster_forces
                + (1.0 - self.command_smoothing_alpha) * self.previous_thruster_forces
            )
        achieved = self.tam.dot(thruster_forces)
        error = self.desired_wrench - achieved
        self.latest_thruster_forces = thruster_forces
        self.previous_thruster_forces = thruster_forces
        self.latest_achieved_wrench = achieved
        self.latest_error = error
        self.latest_report = (
            "method=%s weights=%s desired_wrench=%s achieved_wrench=%s error=%s thrusters=%s roles=%s"
            % (
                self.allocation_method,
                self._format_vector(self.dof_weights),
                self._format_vector(self.desired_wrench),
                self._format_vector(achieved),
                self._format_vector(error),
                self._format_vector(thruster_forces),
                "; ".join(self.thruster_roles),
            )
        )

        wrench = Wrench()
        wrench.force.x = float(achieved[0])
        wrench.force.y = float(achieved[1])
        wrench.force.z = float(achieved[2])
        wrench.torque.x = float(achieved[3])
        wrench.torque.y = float(achieved[4])
        wrench.torque.z = float(achieved[5])
        self.commanded_wrench_pub.publish(wrench)

        vector_msg = Float64MultiArray()
        vector_msg.data = [float(value) for value in thruster_forces]
        self.thruster_vector_pub.publish(vector_msg)

        for publisher, value in zip(self.thruster_publishers, thruster_forces):
            thrust = Float64()
            thrust.data = float(value)
            publisher.publish(thrust)


def main():
    rclpy.init()
    node = VirtualThrusterManager()
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
