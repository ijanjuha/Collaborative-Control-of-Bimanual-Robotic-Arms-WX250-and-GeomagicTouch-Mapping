#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

class MoveitTest(Node):
    def __init__(self):
        super().__init__("moveit_test")
        roscpp_initialize([])
        self.arm = MoveGroupCommander("interbotix_arm")
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.get_logger().info("Moved to home position.")
        roscpp_shutdown()

def main():
    rclpy.init()
    MoveitTest()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

