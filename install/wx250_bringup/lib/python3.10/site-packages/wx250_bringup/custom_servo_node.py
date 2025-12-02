#!/usr/bin/env python3
"""
Custom servo that mimics RX200 teleop - position control instead of velocity
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
import numpy as np
from threading import Lock

try:
    from moveit_commander import RobotCommander, MoveGroupCommander
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("[WARN] moveit_commander not available")


class CustomServoNode(Node):
    def __init__(self):
        super().__init__('custom_servo_node')
        
        self.declare_parameter('move_group_name', 'interbotix_arm')
        self.declare_parameter('publish_period', 0.05)
        
        self.move_group_name = self.get_parameter('move_group_name').value
        self.publish_period = self.get_parameter('publish_period').value
        
        self.get_logger().info("Custom Servo using RX200-style IK!")
        
        self.current_joint_state = None
        self.joint_names = None
        self.lock = Lock()
        self.target_pose = None
        self.moveit_initialized = False
        
        # Try to initialize MoveIt
        if MOVEIT_AVAILABLE:
            try:
                self.robot = RobotCommander()
                self.move_group = MoveGroupCommander(self.move_group_name)
                self.joint_names = self.move_group.get_active_joints()
                self.moveit_initialized = True
                self.get_logger().info(f"MoveIt initialized: {self.joint_names}")
            except Exception as e:
                self.get_logger().error(f"MoveIt init failed: {e}")
                self.moveit_initialized = False
        
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            self.twist_callback,
            qos_profile
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/wx250/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/servo_node/joint_trajectory_cmd',
            10
        )
        
        self.status_pub = self.create_publisher(Int8, '/servo_node/status', 10)
        
        self.get_logger().info("Servo initialized!")
        
    def joint_state_callback(self, msg):
        with self.lock:
            self.current_joint_state = msg
            if self.joint_names is None:
                self.joint_names = [name for name in msg.name 
                                   if 'finger' not in name and 'gripper' not in name]
                self.get_logger().info(f"Joints: {self.joint_names}")
    
    def twist_callback(self, msg):
        """Twist now contains target POSITION (X,Y,Z) not velocity!"""
        with self.lock:
            # Extract target Cartesian position from twist message
            self.target_pose = {
                'x': msg.twist.linear.x,
                'y': msg.twist.linear.y,
                'z': msg.twist.linear.z
            }
        
        self.publish_trajectory()
    
    def publish_trajectory(self):
        try:
            with self.lock:
                if self.target_pose is None or self.current_joint_state is None:
                    return
                if self.joint_names is None:
                    return
                
                target = self.target_pose
                
                current_positions = []
                for joint_name in self.joint_names:
                    try:
                        idx = self.current_joint_state.name.index(joint_name)
                        current_positions.append(self.current_joint_state.position[idx])
                    except (ValueError, IndexError):
                        return
            
            # Compute IK for target Cartesian position
            target_joints = self.compute_ik(target, current_positions)
            
            if target_joints is None:
                return
            
            # Create trajectory
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = self.joint_names
            
            point0 = JointTrajectoryPoint()
            point0.positions = current_positions
            point0.velocities = [0.0] * len(self.joint_names)
            point0.time_from_start.sec = 0
            point0.time_from_start.nanosec = 0
            
            point1 = JointTrajectoryPoint()
            point1.positions = target_joints
            point1.velocities = [0.0] * len(self.joint_names)
            point1.time_from_start.sec = 0
            point1.time_from_start.nanosec = int(self.publish_period * 1e9)
            
            traj.points.append(point0)
            traj.points.append(point1)
            
            self.trajectory_pub.publish(traj)
            self.get_logger().info(f"IK target: [{target['x']:.2f}, {target['y']:.2f}, {target['z']:.2f}]", 
                                  throttle_duration_sec=0.5)
            
            status = Int8()
            status.data = 0
            self.status_pub.publish(status)
        except Exception as e:
            self.get_logger().error(f"Error: {e}", throttle_duration_sec=1.0)
    
    def compute_ik(self, target_pose, current_joints):
        """Compute inverse kinematics for target Cartesian pose"""
        
        if self.moveit_initialized:
            try:
                # Use MoveIt's IK
                self.move_group.set_joint_value_target(current_joints)
                pose_goal = self.move_group.get_current_pose().pose
                pose_goal.position.x = target_pose['x']
                pose_goal.position.y = target_pose['y']
                pose_goal.position.z = target_pose['z']
                
                joint_values = self.move_group.compute_ik(pose_goal, timeout=0.01)
                if joint_values:
                    return list(joint_values)
            except Exception as e:
                self.get_logger().warn(f"MoveIt IK failed: {e}")
        
        # Fallback: Simple analytical IK for 5-DOF arm
        x, y, z = target_pose['x'], target_pose['y'], target_pose['z']
        
        # Simple geometric IK (approximate)
        waist = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        
        L1 = 0.25  # shoulder-to-elbow length (estimate)
        L2 = 0.25  # elbow-to-wrist length (estimate)
        
        # 2-link planar arm IK in r-z plane
        d = np.sqrt(r**2 + z**2)
        d = np.clip(d, 0, L1 + L2 - 0.01)
        
        cos_elbow = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_elbow = np.clip(cos_elbow, -1, 1)
        elbow = np.arccos(cos_elbow)
        
        alpha = np.arctan2(z, r)
        beta = np.arctan2(L2 * np.sin(elbow), L1 + L2 * np.cos(elbow))
        shoulder = alpha - beta
        
        wrist_angle = -(shoulder + elbow)  # Keep wrist level
        wrist_rotate = 0.0
        
        return [waist, shoulder, elbow, wrist_angle, wrist_rotate]


def main(args=None):
    rclpy.init(args=args)
    node = CustomServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
