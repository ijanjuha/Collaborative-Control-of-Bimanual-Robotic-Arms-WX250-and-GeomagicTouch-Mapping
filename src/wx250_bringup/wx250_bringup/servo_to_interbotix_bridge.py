#!/usr/bin/env python3
"""
Bridge between MoveIt Servo JointTrajectory messages and Interbotix JointTrajectoryCommand
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from interbotix_xs_msgs.msg import JointTrajectoryCommand


class ServoToInterbotixBridge(Node):
    """Converts standard JointTrajectory to Interbotix JointTrajectoryCommand"""
    
    def __init__(self):
        super().__init__('servo_to_interbotix_bridge')
        
        # Parameters
        self.declare_parameter('group_name', 'arm')
        self.group_name = self.get_parameter('group_name').value
        
        # Subscriber to servo output
        self.sub = self.create_subscription(
            JointTrajectory,
            '/servo_node/joint_trajectory_cmd',
            self.trajectory_callback,
            10
        )
        
        # Publisher to Interbotix command
        self.pub = self.create_publisher(
            JointTrajectoryCommand,
            '/wx250/commands/joint_trajectory',
            10
        )
        
        self.get_logger().info(
            f"Bridging /servo_node/joint_trajectory_cmd (JointTrajectory)  ->  "
            f"/wx250/commands/joint_trajectory (Interbotix JointTrajectoryCommand)  "
            f"with group=\"{self.group_name}\""
        )
    
    def trajectory_callback(self, msg):
        """Convert JointTrajectory to JointTrajectoryCommand"""
        # Create Interbotix command
        cmd = JointTrajectoryCommand()
        cmd.cmd_type = 'group'  # CRITICAL: Must be set to 'group' for xs_sdk_sim to process!
        cmd.name = self.group_name
        cmd.traj = msg
        
        # Publish
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ServoToInterbotixBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
