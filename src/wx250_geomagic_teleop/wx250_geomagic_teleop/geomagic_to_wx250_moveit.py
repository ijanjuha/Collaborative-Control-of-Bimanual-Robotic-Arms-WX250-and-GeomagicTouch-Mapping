#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Geomagic -> WX250 teleop using MoveIt Python API (no Interbotix interface import).

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize
from moveit_commander.conversions import pose_to_list

def quat_multiply(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_normalize(q):
    x,y,z,w = q
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n == 0.0 or math.isnan(n) or math.isinf(n):
        return (0.0,0.0,0.0,1.0)
    return (x/n,y/n,z/n,w/n)

class GeomagicToMoveIt(Node):
    def __init__(self):
        super().__init__("geomagic_to_wx250")

        # Params
        self.declare_parameter("group_name", "interbotix_arm")
        self.declare_parameter("base_frame", "wx250/base_link")
        self.declare_parameter("ee_link", "ee_gripper_link")
        self.declare_parameter("geomagic_pose_topic", "/geomagic/pose")
        self.declare_parameter("geomagic_buttons_topic", "/geomagic/buttons")
        self.declare_parameter("pos_scale", 0.5)
        self.declare_parameter("calib_xyz", [0.3, 0.0, 0.2])
        self.declare_parameter("calib_quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("plan_time", 0.25)

        # MoveIt init
        roscpp_initialize([])
        group_name = self.get_parameter("group_name").value
        self.group = MoveGroupCommander(group_name)
        self.group.set_end_effector_link(self.get_parameter("ee_link").value)
        self.group.set_pose_reference_frame(self.get_parameter("base_frame").value)
        self.group.set_num_planning_attempts(1)
        self.group.set_planning_time(float(self.get_parameter("plan_time").value))
        self.group.set_max_velocity_scaling_factor(0.6)
        self.group.set_max_acceleration_scaling_factor(0.6)

        # Subs
        self.pose_sub = self.create_subscription(PoseStamped, self.get_parameter("geomagic_pose_topic").value, self.pose_cb, 10)
        self.btn_sub  = self.create_subscription(UInt8,        self.get_parameter("geomagic_buttons_topic").value, self.buttons_cb, 10)

        # State
        self.enable_motion = False
        self.freeze = False
        self.target_pose_robot = None
        self.last_pose_time = self.get_clock().now()

        # Loop
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info("Teleop ready (MoveIt API). Hold Button1 to clutch, Button2 toggles freeze.")

    def buttons_cb(self, msg: UInt8):
        b = int(msg.data)
        clutch = bool(b & 0x01)
        toggle = bool(b & 0x02)
        self.enable_motion = clutch
        if toggle:
            self.freeze = not self.freeze
            self.get_logger().info(f"Freeze -> {self.freeze}")

    def pose_cb(self, msg: PoseStamped):
        pos_scale = float(self.get_parameter("pos_scale").value)
        cx, cy, cz = list(self.get_parameter("calib_xyz").value)
        cq = quat_normalize(tuple(self.get_parameter("calib_quat_xyzw").value))
        base_frame = self.get_parameter("base_frame").value

        x = float(msg.pose.position.x)*pos_scale + cx
        y = float(msg.pose.position.y)*pos_scale + cy
        z = float(msg.pose.position.z)*pos_scale + cz

        gq = (float(msg.pose.orientation.x), float(msg.pose.orientation.y),
              float(msg.pose.orientation.z), float(msg.pose.orientation.w))
        rq = quat_normalize(quat_multiply(cq, gq))

        if any(map(lambda v: math.isnan(v) or math.isinf(v), (x,y,z,*rq))):
            return

        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = base_frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x,y,z
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = rq
        self.target_pose_robot = p
        self.last_pose_time = self.get_clock().now()

    def tick(self):
        if not self.enable_motion or self.freeze or self.target_pose_robot is None:
            return
        if (self.get_clock().now() - self.last_pose_time) > Duration(seconds=0.5):
            return

        try:
            self.group.set_pose_target(self.target_pose_robot.pose)
            plan = self.group.plan()
            success = plan and len(plan.joint_trajectory.points) > 0
            if success:
                self.group.execute(plan, wait=False)  # non-blocking for responsiveness
            else:
                self.get_logger().warn_throttle(2.0, "IK/plan failed (adjust pos_scale/calib_xyz).")
        except Exception as e:
            self.get_logger().error(f"MoveIt error: {e}")

def main():
    rclpy.init()
    node = GeomagicToMoveIt()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

