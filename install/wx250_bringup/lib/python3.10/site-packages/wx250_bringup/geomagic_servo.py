#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

class GeomagicToServo(Node):
    def __init__(self):
        super().__init__('geomagic_to_servo')

        self.declare_parameter('pose_topic', '/phantom/pose')
        self.declare_parameter('frame_id', 'wx250/base_link')
        self.declare_parameter('scale', 2.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.scale      = float(self.get_parameter('scale').value)

        # Transformation matrix with INVERTED Z
        self.T = np.array([
            [1,  0,  0,  0.0],
            [0,  1,  0, -0.2],
            [0,  0, -1,  0.10],   # ← NEGATIVE Z!
            [0,  0,  0,  1.0]
        ])

        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.cb_pose, 50)
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 50)

        self.get_logger().info("Geomagic with FIXED Z direction!")

    def transform_point(self, p_geomagic):
        """Apply transformation matrix"""
        p_hom = np.array([p_geomagic[0], p_geomagic[1], p_geomagic[2] + 0.06, 1.0])
        p_transformed = self.T @ p_hom
        return p_transformed[:3]

    def cb_pose(self, msg: PoseStamped):
        p = msg.pose.position
        p_geomagic = [p.x, p.y, p.z]
        
        p_inter = self.transform_point(p_geomagic)
        
        target_x = p_inter[0] * self.scale
        target_y = p_inter[1] * self.scale
        target_z = p_inter[2]
        
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = self.frame_id
        tw.twist.linear.x = target_x
        tw.twist.linear.y = target_y
        tw.twist.linear.z = target_z
        tw.twist.angular.x = 0.0
        tw.twist.angular.y = 0.0
        tw.twist.angular.z = 0.0
        
        self.pub.publish(tw)

def main():
    rclpy.init()
    node = GeomagicToServo()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
