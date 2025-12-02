#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Geomagic -> WX250 teleop for ROS 2 (Interbotix XS Arms), simulation-friendly.
#
# Run MoveIt (sim) first:
#   ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx250 use_sim:=true
#
# Then run this script:
#   python3 ~/interbotix_ws/src/wx250_geomagic_teleop/wx250_geomagic_teleop/geomagic_to_wx250.py
#
# Topics (can be remapped via params):
#   /geomagic/pose    (geometry_msgs/PoseStamped)
#   /geomagic/buttons (std_msgs/UInt8)  bit0=clutch, bit1=freeze toggle
#
# Controls:
#   Hold Button1 -> "clutch" (enable motion)
#   Tap  Button2 -> toggle "freeze"
#
import math
import os
import sys
import glob
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8

# --- BEGIN PATH SHIM (lets this script find the Interbotix interface when run directly) ---
WS_INSTALL = os.path.expanduser('~/interbotix_ws/install')
# Try site-packages installs
for sp in glob.glob(os.path.join(WS_INSTALL, '*', 'lib', 'python*', 'site-packages')):
    if os.path.isdir(os.path.join(sp, 'interbotix_xsarm_moveit_interface')):
        if sp not in sys.path:
            sys.path.insert(0, sp)
# Try "lib/<pkg>" style (some distros install python modules directly under lib)
alt = os.path.join(WS_INSTALL, 'interbotix_xsarm_moveit_interface', 'lib', 'interbotix_xsarm_moveit_interface')
if os.path.isdir(alt) and alt not in sys.path:
    sys.path.insert(0, alt)
# --- END PATH SHIM ---

try:
    from interbotix_xsarm_moveit_interface.xsarm_moveit_interface import InterbotixMoveItXSInterface
except Exception as e:
    print("[geomagic_to_wx250] ERROR: cannot import InterbotixMoveItXSInterface. "
          "Build/source your workspace or keep this window sourced with install/setup.bash.\n"
          f"Import error: {e}", file=sys.stderr)
    raise


def quat_multiply(q1: Tuple[float, float, float, float],
                  q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    )


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0 or math.isnan(n) or math.isinf(n):
        # return identity if invalid
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


class GeomagicWxTeleop(Node):
    def __init__(self):
        super().__init__("geomagic_to_wx250")

        # Robot/frames
        self.declare_parameter("robot_model", "wx250")
        self.declare_parameter("group_name", "interbotix_arm")
        self.declare_parameter("ee_frame", "ee_gripper_link")
        self.declare_parameter("base_frame", "wx250/base_link")

        # Topic names (allow remap via params if your driver uses different names)
        self.declare_parameter("geomagic_pose_topic", "/geomagic/pose")
        self.declare_parameter("geomagic_buttons_topic", "/geomagic/buttons")

        # Geomagic -> robot mapping
        self.declare_parameter("pos_scale", 0.5)                # scale stylus workspace
        self.declare_parameter("calib_xyz", [0.3, 0.0, 0.2])    # offset (in base frame)
        self.declare_parameter("calib_quat_xyzw", [0.0, 0.0, 0.0, 1.0])  # orientation offset

        robot_model = self.get_parameter("robot_model").value
        self.mv = InterbotixMoveItXSInterface(robot_model)

        # Subscriptions
        pose_topic = self.get_parameter("geomagic_pose_topic").value
        buttons_topic = self.get_parameter("geomagic_buttons_topic").value
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)
        self.btn_sub = self.create_subscription(UInt8, buttons_topic, self.buttons_cb, 10)

        # State
        self.enable_motion = False
        self.freeze = False
        self.target_pose_robot = None  # PoseStamped
        self.last_pose_time = self.get_clock().now()

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Geomagic->WX teleop ready for '{robot_model}'. "
            f"Hold Button1 to clutch; Button2 toggles freeze.\n"
            f"Listening on: pose='{pose_topic}', buttons='{buttons_topic}'"
        )

    def buttons_cb(self, msg: UInt8):
        b = int(msg.data)
        clutch = bool(b & 0x01)   # Button1
        toggle = bool(b & 0x02)   # Button2
        self.enable_motion = clutch
        if toggle:
            self.freeze = not self.freeze
            self.get_logger().info(f"[buttons] Freeze toggled -> {self.freeze}")

    def pose_cb(self, msg: PoseStamped):
        # Params
        pos_scale = float(self.get_parameter("pos_scale").value)
        calib_xyz = list(self.get_parameter("calib_xyz").value)
        calib_quat = tuple(self.get_parameter("calib_quat_xyzw").value)
        base_frame = self.get_parameter("base_frame").value

        # Position (scaled + offset)
        x = float(msg.pose.position.x) * pos_scale + float(calib_xyz[0])
        y = float(msg.pose.position.y) * pos_scale + float(calib_xyz[1])
        z = float(msg.pose.position.z) * pos_scale + float(calib_xyz[2])

        # Orientation (calib * geomagic), normalized
        gq = (float(msg.pose.orientation.x),
              float(msg.pose.orientation.y),
              float(msg.pose.orientation.z),
              float(msg.pose.orientation.w))
        cq = quat_normalize(tuple(calib_quat))
        rq = quat_normalize(quat_multiply(cq, gq))

        if any(map(lambda v: math.isnan(v) or math.isinf(v), (x, y, z, *rq))):
            # Skip bogus data
            return

        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = base_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.x = rq[0]
        p.pose.orientation.y = rq[1]
        p.pose.orientation.z = rq[2]
        p.pose.orientation.w = rq[3]

        self.target_pose_robot = p
        self.last_pose_time = self.get_clock().now()

    def _tick(self):
        # Only move when clutch is held and not frozen, and we have a fresh target
        if not self.enable_motion or self.freeze or self.target_pose_robot is None:
            return

        if (self.get_clock().now() - self.last_pose_time) > Duration(seconds=0.5):
            # stale input; don't chase it
            return

        base_frame = self.get_parameter("base_frame").value
        ee_frame = self.get_parameter("ee_frame").value

        try:
            success = self.mv.move_to_ee_pose(
                pose_goal=self.target_pose_robot.pose,
                base_link=base_frame,
                ee_link=ee_frame,
                execute=True,
                plan_time=0.25,
                num_planning_attempts=1,
                cartesian=False
            )
            if not success:
                # Common cause: target outside reachable workspace
                self.get_logger().warn_throttle(2.0, "IK/plan failed (likely out of reach) — adjust pos_scale/calib_xyz")
        except Exception as e:
            self.get_logger().error(f"Move error: {e}")


def main():
    rclpy.init()
    node = GeomagicWxTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

