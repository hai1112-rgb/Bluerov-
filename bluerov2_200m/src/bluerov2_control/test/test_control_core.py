#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import sys
import unittest

import numpy
import yaml


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_DIR / "scripts"
CONFIG_DIR = PACKAGE_DIR / "config"


def load_script(name):
    path = SCRIPTS_DIR / name
    module_name = path.stem
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


dynamics = load_script("simple_dynamics_sim.py")
sonar = load_script("simple_sonar_sim.py")
thrusters = load_script("virtual_thruster_manager.py")
autopilot = load_script("waypoint_autopilot.py")


class ControlCoreTests(unittest.TestCase):
    def test_body_world_transforms_are_inverse(self):
        body_vector = (1.2, -0.4, 0.7)
        attitude = (0.12, -0.22, 0.51)

        world_vector = dynamics.body_to_world(*body_vector, *attitude)
        recovered = dynamics.world_to_body(*world_vector, *attitude)

        for actual, expected in zip(recovered, body_vector):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_angle_wrapping_keeps_values_inside_pi_range(self):
        self.assertAlmostEqual(autopilot.wrap_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(autopilot.wrap_angle(-3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(autopilot.wrap_angle(0.25), 0.25)

    def test_tam_has_expected_shape_rank_and_allocation_behavior(self):
        with (CONFIG_DIR / "TAM.yaml").open("r", encoding="utf-8") as stream:
            tam = numpy.array(yaml.safe_load(stream)["tam"], dtype=float)

        self.assertEqual(tam.shape, (6, 6))
        self.assertEqual(numpy.linalg.matrix_rank(tam), 5)

        known_thruster_forces = numpy.array([10.0, -8.0, 6.0, -4.0, 3.0, -2.0])
        desired = tam.dot(known_thruster_forces)
        allocated = numpy.linalg.pinv(tam).dot(desired)
        achieved = tam.dot(allocated)

        numpy.testing.assert_allclose(achieved, desired, atol=1e-8)

    def test_damped_allocator_matrix_is_well_conditioned(self):
        with (CONFIG_DIR / "TAM.yaml").open("r", encoding="utf-8") as stream:
            tam = numpy.array(yaml.safe_load(stream)["tam"], dtype=float)

        weights = numpy.diag([1.0, 1.0, 1.2, 0.9, 0.2, 1.0])
        weighted_tam = weights.dot(tam)
        lhs = weighted_tam.T.dot(weighted_tam) + (0.05 ** 2) * numpy.eye(tam.shape[1])

        self.assertGreater(float(numpy.linalg.det(lhs)), 0.0)
        self.assertTrue(numpy.all(numpy.linalg.eigvalsh(lhs) > 0.0))

    def test_sonar_ray_intersections(self):
        circle_hit = sonar.ray_circle_distance(0.0, 0.0, 1.0, 0.0, 5.0, 0.0, 1.0)
        box_hit = sonar.ray_box_distance(0.0, 0.0, 1.0, 0.0, 5.0, 0.0, 2.0, 2.0)
        sphere_hit = sonar.ray_sphere_distance_3d((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 0.0, 0.0), 1.0)

        self.assertAlmostEqual(circle_hit, 4.0)
        self.assertAlmostEqual(box_hit, 4.0)
        self.assertAlmostEqual(sphere_hit, 4.0)

    def test_point_cloud_layout_matches_declared_fields(self):
        cloud = sonar.make_point_cloud("world", None, [(1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 8.0)])

        self.assertEqual(cloud.header.frame_id, "world")
        self.assertEqual(cloud.width, 2)
        self.assertEqual(cloud.point_step, 16)
        self.assertEqual(cloud.row_step, 32)
        self.assertEqual(len(cloud.data), 32)

    def test_yaml_configs_parse_and_include_required_parameters(self):
        required = {
            "simulator.yaml": ["mass", "water_depth", "current_speed"],
            "thruster_manager.yaml": ["allocation_method", "allocation_damping", "dof_weights"],
            "autonomous_controller.yaml": ["waypoints", "enable_forward_sonar_avoidance", "kp_xy"],
            "localization.yaml": ["world_frame", "base_frame", "update_rate"],
            "sensors.yaml": ["imu_topic", "dvl_topic", "depth_topic"],
        }

        for filename, keys in required.items():
            with (CONFIG_DIR / filename).open("r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream)
            parameters = config["/**"]["ros__parameters"] if "/**" in config else next(iter(config.values()))["ros__parameters"]
            for key in keys:
                self.assertIn(key, parameters, filename)

    def test_disturbance_modes_are_consistent_between_keyboard_and_dynamics(self):
        keyboard = load_script("keyboard_disturbance.py")
        modes = set(keyboard.MODE_MAP.values())
        self.assertEqual(modes, {"storm", "current_n", "current_e", "current_strong", "normal"})

        source = (SCRIPTS_DIR / "simple_dynamics_sim.py").read_text(encoding="utf-8")
        for mode in modes:
            self.assertIn(mode, source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
