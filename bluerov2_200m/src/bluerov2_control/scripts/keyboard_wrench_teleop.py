#!/usr/bin/env python3

import select
import sys
import termios
import time
import tkinter as tk
import tty

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node


HELP_LINES = [
    "W / S: forward / reverse",
    "A / D: yaw left / yaw right",
    "Q / E: strafe left / strafe right",
    "R / F: ascend / descend",
    "Space: stop immediately",
    "Esc: exit",
]


class KeyboardWrenchTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_wrench_teleop")

        defaults = {
            "wrench_topic": "thruster_manager/input",
            "publish_rate": 20.0,
            "surge_force": 65.0,
            "sway_force": 45.0,
            "heave_force": 55.0,
            "yaw_torque": 16.0,
            "window_title": "BlueROV2 WASD Teleop",
            "ui_mode": "terminal",
            "key_timeout": 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.wrench_topic = str(self.get_parameter("wrench_topic").value)
        self.publish_rate = max(5.0, float(self.get_parameter("publish_rate").value))
        self.surge_force = max(0.0, float(self.get_parameter("surge_force").value))
        self.sway_force = max(0.0, float(self.get_parameter("sway_force").value))
        self.heave_force = max(0.0, float(self.get_parameter("heave_force").value))
        self.yaw_torque = max(0.0, float(self.get_parameter("yaw_torque").value))
        self.window_title = str(self.get_parameter("window_title").value)
        self.ui_mode = str(self.get_parameter("ui_mode").value).lower()
        self.key_timeout = max(0.05, float(self.get_parameter("key_timeout").value))

        self.wrench_pub = self.create_publisher(Wrench, self.wrench_topic, 10)
        self.pressed_keys = set()
        self.root = None

        if self.ui_mode not in {"terminal", "tk"}:
            self.get_logger().warning("Unknown ui_mode=%s; using terminal" % self.ui_mode)
            self.ui_mode = "terminal"

        if self.ui_mode == "terminal" and not sys.stdin.isatty():
            self.get_logger().warning("stdin is not a TTY; falling back to Tk window mode")
            self.ui_mode = "tk"

        if self.ui_mode == "tk":
            self._init_tk()
        else:
            self.get_logger().info(
                "Terminal teleop publishing to %s | press/hold WASD, Q/E, R/F; Space stops; Esc exits"
                % self.wrench_topic
            )

    def _init_tk(self):
        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("540x360")
        self.root.minsize(480, 320)
        self.root.configure(bg="#0a1720")
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.status_var = tk.StringVar(value="Focus this window, then hold W/A/S/D to drive.")
        self.axes_var = tk.StringVar(value="")
        self.keys_var = tk.StringVar(value="Held keys: none")

        self._build_ui()
        self._set_axes_text(0.0, 0.0, 0.0, 0.0)
        self.root.after(120, self._tick)
        self.root.after(300, self._focus_window)

        self.get_logger().info(
            "Keyboard teleop publishing to %s | hold W+A or W+D to combine axes"
            % self.wrench_topic
        )

    def _build_ui(self):
        title = tk.Label(
            self.root,
            text="BlueROV2 Manual Teleop",
            bg="#0a1720",
            fg="#e9fff8",
            font=("Segoe UI", 20, "bold"),
        )
        title.pack(anchor="w", padx=20, pady=(18, 6))

        subtitle = tk.Label(
            self.root,
            text="This window captures real key presses. Hold W+A, W+D, Q+W, R+W, and similar combinations to blend motion axes.",
            justify="left",
            wraplength=500,
            bg="#0a1720",
            fg="#9ac9ca",
            font=("Segoe UI", 10),
        )
        subtitle.pack(anchor="w", padx=20)

        help_frame = tk.Frame(self.root, bg="#11232d", highlightbackground="#234552", highlightthickness=1)
        help_frame.pack(fill="x", padx=20, pady=(16, 14))
        for line in HELP_LINES:
            label = tk.Label(
                help_frame,
                text=line,
                anchor="w",
                bg="#11232d",
                fg="#effaf8",
                font=("Consolas", 11),
                padx=14,
                pady=4,
            )
            label.pack(fill="x")

        status = tk.Label(
            self.root,
            textvariable=self.status_var,
            justify="left",
            wraplength=500,
            bg="#0a1720",
            fg="#ffd277",
            font=("Segoe UI", 10, "bold"),
        )
        status.pack(anchor="w", padx=20, pady=(0, 10))

        axes = tk.Label(
            self.root,
            textvariable=self.axes_var,
            justify="left",
            bg="#081118",
            fg="#7ce7d0",
            font=("Consolas", 12),
            padx=14,
            pady=12,
            relief="flat",
        )
        axes.pack(fill="x", padx=20)

        keys = tk.Label(
            self.root,
            textvariable=self.keys_var,
            justify="left",
            bg="#0a1720",
            fg="#c7ece5",
            font=("Segoe UI", 10),
        )
        keys.pack(anchor="w", padx=20, pady=(12, 8))

        footer = tk.Label(
            self.root,
            text="Run teleop in a separate terminal together with 'enhanced' or 'nav_manual'.",
            justify="left",
            bg="#0a1720",
            fg="#9ac9ca",
            font=("Segoe UI", 9),
        )
        footer.pack(anchor="w", padx=20)

    def _focus_window(self):
        try:
            self.root.focus_force()
        except tk.TclError:
            pass

    def _normalize_key(self, event):
        key = event.keysym.lower()
        if key in {"up", "down", "left", "right", "prior", "next", "space", "escape"}:
            return key
        if len(key) == 1:
            return key
        return None

    def _on_key_press(self, event):
        key = self._normalize_key(event)
        if key is None:
            return
        if key == "space":
            self._clear_keys("Emergency stop")
            return
        if key == "escape":
            self._close()
            return

        self.pressed_keys.add(key)
        self.status_var.set("Manual control active. Release a key to stop that axis.")
        self._update_keys_text()

    def _on_key_release(self, event):
        key = self._normalize_key(event)
        if key is None:
            return
        self.pressed_keys.discard(key)
        if not self.pressed_keys:
            self.status_var.set("All keys released. Commanded wrench is returning to zero.")
        self._update_keys_text()

    def _clear_keys(self, reason):
        self.pressed_keys.clear()
        self.status_var.set(f"{reason}. All axes are zero.")
        self._update_keys_text()
        self._publish_current_wrench()

    def _update_keys_text(self):
        if self.pressed_keys:
            shown = " + ".join(sorted(self.pressed_keys))
        else:
            shown = "none"
        self.keys_var.set(f"Held keys: {shown}")

    def _axis_value(self, positive_keys, negative_keys):
        positive = any(key in self.pressed_keys for key in positive_keys)
        negative = any(key in self.pressed_keys for key in negative_keys)
        return float(positive) - float(negative)

    def _compute_axes(self):
        surge = self._axis_value({"w", "up"}, {"s", "down"})
        yaw = self._axis_value({"a", "left"}, {"d", "right"})
        sway = self._axis_value({"q"}, {"e"})
        heave = self._axis_value({"r", "prior"}, {"f", "next"})
        return surge, sway, heave, yaw

    def _set_axes_text(self, surge, sway, heave, yaw):
        if not hasattr(self, "axes_var"):
            return
        self.axes_var.set(
            "surge x : {:+5.1f} N\n"
            "sway  y : {:+5.1f} N\n"
            "heave z : {:+5.1f} N\n"
            "yaw   z : {:+5.1f} Nm".format(
                surge * self.surge_force,
                sway * self.sway_force,
                heave * self.heave_force,
                yaw * self.yaw_torque,
            )
        )

    def _publish_current_wrench(self):
        surge, sway, heave, yaw = self._compute_axes()
        msg = Wrench()
        msg.force.x = surge * self.surge_force
        msg.force.y = sway * self.sway_force
        msg.force.z = heave * self.heave_force
        msg.torque.z = yaw * self.yaw_torque
        self.wrench_pub.publish(msg)
        self._set_axes_text(surge, sway, heave, yaw)

    def _tick(self):
        if self.root is None or not rclpy.ok():
            return
        self._publish_current_wrench()
        period_ms = max(20, int(1000.0 / self.publish_rate))
        self.root.after(period_ms, self._tick)

    def _close(self):
        if self.root is None:
            return

        self.pressed_keys.clear()
        self._publish_current_wrench()
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        self.root = None

    def run(self):
        if self.ui_mode == "tk":
            try:
                self.root.mainloop()
            finally:
                self._publish_zero_wrench()
            return

        self._run_terminal()

    def _publish_zero_wrench(self):
        self.pressed_keys.clear()
        self._publish_current_wrench()

    def _run_terminal(self):
        old_settings = termios.tcgetattr(sys.stdin)
        last_key_time = 0.0
        period = max(0.02, 1.0 / self.publish_rate)
        print(
            "\nBlueROV2 WASD teleop\n"
            "  W/S: forward/back, A/D: yaw left/right, Q/E: strafe, R/F: up/down\n"
            "  Hold a key or press repeatedly. Space stops. Esc or Ctrl-C exits.\n"
        )
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], period)
                if readable:
                    char = sys.stdin.read(1)
                    key = self._terminal_key(char)
                    if key == "escape":
                        break
                    if key == "space":
                        self.pressed_keys.clear()
                    elif key is not None:
                        self.pressed_keys = {key}
                        last_key_time = time.monotonic()

                if self.pressed_keys and time.monotonic() - last_key_time > self.key_timeout:
                    self.pressed_keys.clear()

                self._publish_current_wrench()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self._publish_zero_wrench()
            print("\nTeleop stopped; published zero wrench.")

    def _terminal_key(self, char):
        if char == "\x1b":
            return "escape"
        if char == " ":
            return "space"
        key = char.lower()
        if key in {"w", "a", "s", "d", "q", "e", "r", "f"}:
            return key
        return None


def main():
    rclpy.init()
    node = KeyboardWrenchTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
