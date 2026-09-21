#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class NavigationGoalPanel(Node):
    def __init__(self):
        super().__init__("navigation_goal_panel")

        defaults = {
            "window_title": "BlueROV2 Navigation Goal Panel",
            "frame_id": "world",
            "odom_topic": "pose_gt",
            "planned_path_topic": "planned_path",
            "goal_pose_topic": "/goal_pose",
            "depth_setpoint_topic": "/depth_setpoint",
            "default_target_x": 8.0,
            "default_target_y": 0.0,
            "default_target_depth": 200.0,
            "default_target_yaw_deg": 0.0,
            "map_extent_m": 45.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.window_title = str(self.get_parameter("window_title").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.planned_path_topic = str(self.get_parameter("planned_path_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.depth_setpoint_topic = str(self.get_parameter("depth_setpoint_topic").value)
        self.map_extent_m = max(5.0, float(self.get_parameter("map_extent_m").value))

        self.odom = None
        self.last_goal = None
        self.path_count = 0
        self.last_error_text = "Waiting for odometry..."

        self.goal_pose_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.depth_pub = self.create_publisher(Float64, self.depth_setpoint_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.create_subscription(Path, self.planned_path_topic, self._path_callback, 10)

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("880x560")
        self.root.minsize(780, 500)
        self.root.configure(bg="#081016")
        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

        self.target_x_var = tk.StringVar(value="%.2f" % float(self.get_parameter("default_target_x").value))
        self.target_y_var = tk.StringVar(value="%.2f" % float(self.get_parameter("default_target_y").value))
        self.target_depth_var = tk.StringVar(
            value="%.2f" % float(self.get_parameter("default_target_depth").value)
        )
        self.target_yaw_var = tk.StringVar(
            value="%.1f" % float(self.get_parameter("default_target_yaw_deg").value)
        )
        self.face_goal_var = tk.BooleanVar(value=True)
        self.current_depth_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Enter a target coordinate and press Send target.")
        self.current_pose_var = tk.StringVar(value="Position: --")
        self.goal_summary_var = tk.StringVar(value="Goal: --")
        self.error_var = tk.StringVar(value=self.last_error_text)

        self._configure_style()
        self._build_ui()
        self.root.after(25, self._spin_once)
        self.root.after(200, self._refresh_ui)

        self.get_logger().info(
            "Navigation goal panel ready | goal=%s depth=%s odom=%s"
            % (self.goal_pose_topic, self.depth_setpoint_topic, self.odom_topic)
        )

    def _configure_style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background="#081016", foreground="#eaf7f3", font=("Segoe UI", 10))
        style.configure("TFrame", background="#081016")
        style.configure("Panel.TFrame", background="#0f1d25", relief="flat")
        style.configure("TLabel", background="#081016", foreground="#eaf7f3")
        style.configure("Muted.TLabel", background="#081016", foreground="#9bbbc0")
        style.configure("Panel.TLabel", background="#0f1d25", foreground="#eaf7f3")
        style.configure("Status.TLabel", background="#0f1d25", foreground="#ffd56a")
        style.configure("TCheckbutton", background="#0f1d25", foreground="#d9f0ec")
        style.configure("TButton", background="#18323d", foreground="#f2fffc", padding=(12, 7))
        style.map("TButton", background=[("active", "#245064")])
        style.configure("Accent.TButton", background="#0b6f69", foreground="#ffffff", padding=(12, 8))
        style.map("Accent.TButton", background=[("active", "#109486")])
        style.configure("Danger.TButton", background="#6b2b30", foreground="#ffffff", padding=(12, 8))
        style.map("Danger.TButton", background=[("active", "#8c3840")])
        style.configure("TEntry", fieldbackground="#071017", foreground="#f4fffb", insertcolor="#f4fffb")

    def _build_ui(self):
        root = ttk.Frame(self.root, padding=18)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1, minsize=430)
        root.columnconfigure(1, weight=1, minsize=300)
        root.rowconfigure(1, weight=1)

        header = ttk.Frame(root)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        title = ttk.Label(header, text="BlueROV2 Navigation", font=("Segoe UI", 18, "bold"))
        title.pack(side="left")
        ttk.Label(
            header,
            text="goal_pose / depth_setpoint",
            style="Muted.TLabel",
            font=("Segoe UI", 10),
        ).pack(side="right")

        left = ttk.Frame(root, style="Panel.TFrame", padding=16)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 12))
        left.columnconfigure(1, weight=1)

        ttk.Label(left, text="Target", style="Panel.TLabel", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )
        self._add_labeled_entry(left, 1, "X world (m)", self.target_x_var)
        self._add_labeled_entry(left, 2, "Y world (m)", self.target_y_var)
        self._add_labeled_entry(left, 3, "Positive depth (m)", self.target_depth_var)
        self._add_labeled_entry(left, 4, "Yaw (deg)", self.target_yaw_var)

        ttk.Checkbutton(
            left,
            text="Automatically face the target",
            variable=self.face_goal_var,
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Checkbutton(
            left,
            text="Keep current depth when sending the target",
            variable=self.current_depth_var,
        ).grid(row=6, column=0, columnspan=2, sticky="w", pady=(6, 0))

        button_row = ttk.Frame(left, style="Panel.TFrame")
        button_row.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(18, 10))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        ttk.Button(button_row, text="Send target", style="Accent.TButton", command=self._send_target).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(button_row, text="Hold current", style="Danger.TButton", command=self._hold_current).grid(
            row=0, column=1, sticky="ew", padx=(6, 0)
        )

        secondary = ttk.Frame(left, style="Panel.TFrame")
        secondary.grid(row=8, column=0, columnspan=2, sticky="ew")
        secondary.columnconfigure(0, weight=1)
        secondary.columnconfigure(1, weight=1)
        ttk.Button(secondary, text="Load current", command=self._load_current).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Button(secondary, text="Depth only", command=self._send_depth_only).grid(
            row=0, column=1, sticky="ew", padx=(6, 0)
        )

        ttk.Label(left, textvariable=self.status_var, style="Status.TLabel", wraplength=360).grid(
            row=9, column=0, columnspan=2, sticky="ew", pady=(18, 0)
        )
        ttk.Label(left, textvariable=self.current_pose_var, style="Panel.TLabel", font=("Consolas", 10)).grid(
            row=10, column=0, columnspan=2, sticky="ew", pady=(18, 0)
        )
        ttk.Label(left, textvariable=self.goal_summary_var, style="Panel.TLabel", font=("Consolas", 10)).grid(
            row=11, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )
        ttk.Label(left, textvariable=self.error_var, style="Panel.TLabel", font=("Consolas", 10)).grid(
            row=12, column=0, columnspan=2, sticky="ew", pady=(8, 0)
        )

        right = ttk.Frame(root, style="Panel.TFrame", padding=16)
        right.grid(row=1, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)
        ttk.Label(right, text="Top-down map", style="Panel.TLabel", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 12)
        )
        self.canvas = tk.Canvas(
            right,
            bg="#061017",
            highlightthickness=1,
            highlightbackground="#24424d",
            width=360,
            height=360,
        )
        self.canvas.grid(row=1, column=0, sticky="nsew")
        ttk.Label(
            right,
            text="Blue = ROV, amber = target, green = planned path.",
            style="Panel.TLabel",
        ).grid(row=2, column=0, sticky="w", pady=(10, 0))

    def _add_labeled_entry(self, parent, row, label, variable):
        ttk.Label(parent, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=6)
        ttk.Entry(parent, textvariable=variable, width=14).grid(row=row, column=1, sticky="ew", pady=6)

    def _odom_callback(self, msg):
        self.odom = msg

    def _path_callback(self, msg):
        self.path_count = len(msg.poses)

    def _spin_once(self):
        if not rclpy.ok():
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(25, self._spin_once)

    def _refresh_ui(self):
        self._update_pose_text()
        self._draw_map()
        if rclpy.ok():
            self.root.after(200, self._refresh_ui)

    def _update_pose_text(self):
        if self.odom is None:
            self.current_pose_var.set("Position: --")
            self.error_var.set("Error: waiting for /%s" % self.odom_topic.strip("/"))
            return

        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        depth = -pose.position.z
        self.current_pose_var.set(
            "Position: x=%6.2f  y=%6.2f  depth=%5.2f  yaw=%6.1f deg"
            % (pose.position.x, pose.position.y, depth, math.degrees(yaw))
        )

        if self.last_goal is None:
            self.error_var.set("Error: -- | path points=%d" % self.path_count)
            return

        dx = self.last_goal[0] - pose.position.x
        dy = self.last_goal[1] - pose.position.y
        dz = self.last_goal[2] - pose.position.z
        self.error_var.set(
            "Error: xy=%5.2f m  z=%5.2f m  total=%5.2f m | path points=%d"
            % (math.hypot(dx, dy), dz, math.sqrt(dx * dx + dy * dy + dz * dz), self.path_count)
        )

    def _parse_float(self, variable, name):
        raw = variable.get().strip()
        try:
            return float(raw)
        except ValueError as exc:
            raise ValueError("%s is not a valid number: %r" % (name, raw)) from exc

    def _target_from_fields(self):
        x = self._parse_float(self.target_x_var, "X")
        y = self._parse_float(self.target_y_var, "Y")
        depth = max(0.0, self._parse_float(self.target_depth_var, "Depth"))
        yaw = math.radians(self._parse_float(self.target_yaw_var, "Yaw"))

        if self.current_depth_var.get() and self.odom is not None:
            z = self.odom.pose.pose.position.z
        else:
            z = -depth

        if self.face_goal_var.get() and self.odom is not None:
            pose = self.odom.pose.pose
            dx = x - pose.position.x
            dy = y - pose.position.y
            if math.hypot(dx, dy) > 1e-6:
                yaw = math.atan2(dy, dx)

        return x, y, z, wrap_angle(yaw)

    def _send_target(self):
        try:
            x, y, z, yaw = self._target_from_fields()
        except ValueError as exc:
            self.status_var.set(str(exc))
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pose_pub.publish(msg)

        self.last_goal = (x, y, z, yaw)
        self.goal_summary_var.set(
            "Goal:     x=%6.2f  y=%6.2f  depth=%5.2f  yaw=%6.1f deg"
            % (x, y, -z, math.degrees(yaw))
        )
        self.status_var.set("Sent target to %s." % self.goal_pose_topic)
        self.get_logger().info(
            "Sent target x=%.2f y=%.2f z=%.2f yaw=%.2f rad" % (x, y, z, yaw)
        )

    def _send_depth_only(self):
        try:
            depth = max(0.0, self._parse_float(self.target_depth_var, "Depth"))
        except ValueError as exc:
            self.status_var.set(str(exc))
            return
        msg = Float64()
        msg.data = depth
        self.depth_pub.publish(msg)
        self.status_var.set("Sent depth %.2f m to %s." % (depth, self.depth_setpoint_topic))

    def _load_current(self):
        if self.odom is None:
            self.status_var.set("No odometry is available to load the current pose.")
            return
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.target_x_var.set("%.2f" % pose.position.x)
        self.target_y_var.set("%.2f" % pose.position.y)
        self.target_depth_var.set("%.2f" % max(0.0, -pose.position.z))
        self.target_yaw_var.set("%.1f" % math.degrees(yaw))
        self.status_var.set("Loaded the current pose into the form.")

    def _hold_current(self):
        self._load_current()
        if self.odom is not None:
            was_face_goal = self.face_goal_var.get()
            self.face_goal_var.set(False)
            self._send_target()
            self.face_goal_var.set(was_face_goal)
            self.status_var.set("Sent hold-current command to the autopilot.")

    def _world_to_canvas(self, x, y):
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        scale = min(width, height) / (2.0 * self.map_extent_m)
        cx = width * 0.5 + x * scale
        cy = height * 0.5 - y * scale
        return cx, cy

    def _draw_map(self):
        canvas = self.canvas
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        canvas.delete("all")

        for value in range(int(-self.map_extent_m), int(self.map_extent_m) + 1, 5):
            x0, y0 = self._world_to_canvas(value, -self.map_extent_m)
            x1, y1 = self._world_to_canvas(value, self.map_extent_m)
            canvas.create_line(x0, y0, x1, y1, fill="#122933")
            x0, y0 = self._world_to_canvas(-self.map_extent_m, value)
            x1, y1 = self._world_to_canvas(self.map_extent_m, value)
            canvas.create_line(x0, y0, x1, y1, fill="#122933")

        ox0, oy0 = self._world_to_canvas(-self.map_extent_m, 0.0)
        ox1, oy1 = self._world_to_canvas(self.map_extent_m, 0.0)
        canvas.create_line(ox0, oy0, ox1, oy1, fill="#31525d")
        ox0, oy0 = self._world_to_canvas(0.0, -self.map_extent_m)
        ox1, oy1 = self._world_to_canvas(0.0, self.map_extent_m)
        canvas.create_line(ox0, oy0, ox1, oy1, fill="#31525d")
        canvas.create_text(12, height - 14, text="world frame", fill="#8bb1b8", anchor="w")

        if self.last_goal is not None:
            gx, gy = self._world_to_canvas(self.last_goal[0], self.last_goal[1])
            canvas.create_oval(gx - 7, gy - 7, gx + 7, gy + 7, fill="#f0b84a", outline="")
            canvas.create_line(gx - 13, gy, gx + 13, gy, fill="#f0b84a", width=2)
            canvas.create_line(gx, gy - 13, gx, gy + 13, fill="#f0b84a", width=2)

        if self.odom is not None:
            pose = self.odom.pose.pose
            x, y = self._world_to_canvas(pose.position.x, pose.position.y)
            yaw = yaw_from_quaternion(pose.orientation)
            heading_len = 22
            hx = x + math.cos(yaw) * heading_len
            hy = y - math.sin(yaw) * heading_len
            if self.last_goal is not None:
                gx, gy = self._world_to_canvas(self.last_goal[0], self.last_goal[1])
                canvas.create_line(x, y, gx, gy, fill="#4ccf9f", dash=(4, 5), width=2)
            canvas.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#54a6ff", outline="")
            canvas.create_line(x, y, hx, hy, fill="#dff7ff", width=3)
            canvas.create_text(x + 12, y - 12, text="ROV", fill="#dff7ff", anchor="w")
        else:
            canvas.create_text(
                width * 0.5,
                height * 0.5,
                text="Dang doi odometry",
                fill="#9bbbc0",
                font=("Segoe UI", 12, "bold"),
            )

    def run(self):
        try:
            self.root.mainloop()
        finally:
            try:
                self.root.destroy()
            except tk.TclError:
                pass


def main():
    rclpy.init()
    node = NavigationGoalPanel()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
