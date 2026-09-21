#!/usr/bin/env python3

import pathlib
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]


class GazeboAssetTests(unittest.TestCase):
    def run_command(self, command):
        return subprocess.run(
            command,
            cwd=PACKAGE_DIR,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_world_is_valid_sdf_and_contains_expected_systems(self):
        world_path = PACKAGE_DIR / "worlds" / "bluerov2_demo.sdf"
        self.run_command(["gz", "sdf", "-k", str(world_path)])

        root = ET.parse(world_path).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib["name"], "underwater_world")
        model_names = {model.attrib["name"] for model in world.findall("model")}
        self.assertIn("inspection_panel_1", model_names)
        self.assertIn("water_surface", model_names)

    def test_rviz_environment_config_has_markers_and_disturbances(self):
        with (PACKAGE_DIR / "config" / "rviz_environment.yaml").open("r", encoding="utf-8") as stream:
            environment = yaml.safe_load(stream)
        with (PACKAGE_DIR / "config" / "disturbances.yaml").open("r", encoding="utf-8") as stream:
            disturbances = yaml.safe_load(stream)

        self.assertGreaterEqual(len(environment.get("obstacles", [])), 1)
        self.assertEqual(disturbances["metadata"]["environment"], "deep_water_200m")
        self.assertGreaterEqual(len(disturbances.get("disturbances", [])), 4)

    def test_launch_files_compile(self):
        for path in (PACKAGE_DIR / "launch").glob("*.launch.py"):
            self.run_command(["python3", "-m", "py_compile", str(path)])

    def test_rviz_config_is_present(self):
        rviz_path = PACKAGE_DIR / "rviz" / "autonomous.rviz"
        self.assertTrue(rviz_path.is_file())
        text = rviz_path.read_text(encoding="utf-8", errors="replace")
        self.assertIn("RobotModel", text)
        self.assertIn("/bluerov2/camera/front/image", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
