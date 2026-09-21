#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]


class DescriptionAssetTests(unittest.TestCase):
    def run_command(self, command):
        return subprocess.run(
            command,
            cwd=PACKAGE_DIR,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_robot_xacro_generates_valid_urdf(self):
        with tempfile.NamedTemporaryFile(suffix=".urdf") as output:
            result = self.run_command([
                "xacro",
                "robots/bluerov2_default.xacro",
                "namespace:=bluerov2",
            ])
            output.write(result.stdout.encode("utf-8"))
            output.flush()

            root = ET.fromstring(result.stdout)
            self.assertEqual(root.tag, "robot")
            self.assertEqual(root.attrib["name"], "bluerov2")
            links = {link.attrib["name"] for link in root.findall("link")}
            self.assertIn("base_link", links)
            self.assertIn("front_camera_link", links)

            self.run_command(["check_urdf", output.name])

    def test_gazebo_xacro_generates_valid_sdf(self):
        result = self.run_command([
            "xacro",
            "models/bluerov2_gz.sdf.xacro",
            "namespace:=bluerov2",
        ])
        root = ET.fromstring(result.stdout)
        self.assertEqual(root.tag, "sdf")
        model = root.find("model")
        self.assertIsNotNone(model)
        self.assertEqual(model.attrib["name"], "bluerov2")

        with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
            output.write(result.stdout.encode("utf-8"))
            output.flush()
            self.run_command(["gz", "sdf", "-k", output.name])

    def test_launch_file_imports(self):
        for path in (PACKAGE_DIR / "launch").glob("*.launch.py"):
            self.run_command(["python3", "-m", "py_compile", str(path)])


if __name__ == "__main__":
    unittest.main(verbosity=2)
