#!/usr/bin/env python3
"""
VEGETA MODE: Intense force feedback from robot to Geomagic
Feel the gravity, push through the resistance!
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64MultiArray
import numpy as np

class RobotToGeomagicFeedback(Node):
    def __init__(self):
        super().__init__('robot_to_geomagic_feedback')
        
        # VEGETA PARAMETERS - Tune these for intensity!
        self.declare_parameter('gravity_force_scale', 0.5)  # How much gravity you feel (0.3-0.5 = intense)
        self.declare_parameter('damping', 0.8)              # Movement resistance (higher = harder to move)
        self.declare_parameter('max_force', 3.3)            # Geomagic Touch hardware limit
        
        self.gravity_scale = self.get_parameter('gravity_force_scale').value
        self.damping = self.get_parameter('damping').value
        self.max_force = self.get_parameter('max_force').value
        
        # Robot parameters for WX250 (realistic masses)
        self.link_masses = [0.5, 0.8, 0.6, 0.3, 0.2]  # kg
        self.link_lengths = [0.0, 0.25, 0.25, 0.15, 0.1]  # meters
        self.g = 9.81  # m/s^2
        
        # State
        self.current_joint_state = None
        self.gravity_torques = np.zeros(5)
        self.last_velocities = np.zeros(5)
        self.callback_count = 0
        
        # Subscribe to robot joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/wx250/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.force_pub = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 10)
        self.gravity_pub = self.create_publisher(Float64MultiArray, '/robot/gravity_torques', 10)
        
        self.get_logger().info("Started!")
        self.get_logger().info(f"Gravity scale: {self.gravity_scale}, Damping: {self.damping}")
        self.get_logger().info("📡 Force feedback will publish with joint states (~100Hz)")
    
    def compute_gravity_torques(self, joint_positions):
        """Compute gravity torques - the weight you'll feel!"""
        torques = np.zeros(5)
        
        if len(joint_positions) < 5:
            return torques
        
        q = np.array(joint_positions[:5])
        
        # Waist (joint 0) - rotates, no vertical gravity effect
        torques[0] = 0.0
        
        # Shoulder (joint 1) - feels weight of entire arm
        shoulder_angle = q[1]
        shoulder_mass = sum(self.link_masses[1:5])  # Total arm mass
        shoulder_com_dist = (self.link_lengths[1] * 0.5 + 
                            self.link_lengths[2] + 
                            self.link_lengths[3] * 0.5)
        torques[1] = shoulder_mass * self.g * shoulder_com_dist * np.cos(shoulder_angle)
        
        # Elbow (joint 2) - feels weight of forearm + wrist
        elbow_angle = q[1] + q[2]
        elbow_mass = sum(self.link_masses[2:5])
        elbow_com_dist = self.link_lengths[2] * 0.5 + self.link_lengths[3] * 0.5
        torques[2] = elbow_mass * self.g * elbow_com_dist * np.cos(elbow_angle)
        
        # Wrist angle (joint 3) - feels weight of hand/gripper
        wrist_angle = q[1] + q[2] + q[3]
        wrist_mass = sum(self.link_masses[3:5])
        wrist_com_dist = self.link_lengths[3] * 0.5
        torques[3] = wrist_mass * self.g * wrist_com_dist * np.cos(wrist_angle)
        
        # Wrist rotate (joint 4) - no gravity
        torques[4] = 0.0
        
        # Debug output every 100 callbacks (~0.1 seconds)
        if self.callback_count % 100 == 0:
            self.get_logger().info(
                f"Shoulder={torques[1]:.2f} Nm, "
                f"Elbow={torques[2]:.2f} Nm, Wrist={torques[3]:.2f} Nm"
            )
        
        return torques
    
    def joint_state_callback(self, msg):
        """Process joint state from robot"""
        self.callback_count += 1
        self.current_joint_state = msg
        
        if len(msg.position) >= 5:
            # Compute gravity torques
            self.gravity_torques = self.compute_gravity_torques(msg.position)
            
            # Publish for debugging
            gravity_msg = Float64MultiArray()
            gravity_msg.data = self.gravity_torques.tolist()
            self.gravity_pub.publish(gravity_msg)
            
            # Store velocities for damping
            if len(msg.velocity) >= 5:
                self.last_velocities = np.array(msg.velocity[:5])
            
            # CRITICAL: Publish force feedback HERE!
            self.publish_feedback()
    
    def publish_feedback(self):
        """
        VEGETA MODE FORCE FEEDBACK
        
        Force mapping strategy:
        - Z-axis (up/down): Shoulder + Elbow gravity (main resistance)
        - X-axis (forward/back): Wrist gravity + elbow contribution
        - Y-axis (left/right): Waist rotation (minimal)
        """
        feedback = OmniFeedback()
        
        if self.current_joint_state is None:
            if self.callback_count <= 3:
                self.get_logger().warn("⚠️ No joint state yet, sending zero forces")
            feedback.force.x = 0.0
            feedback.force.y = 0.0
            feedback.force.z = 0.0
            self.force_pub.publish(feedback)
            return
        
        # === GRAVITY FORCES ===
        # Z-axis: Primary gravity resistance (shoulder + elbow)
        fz = -(self.gravity_torques[1] + self.gravity_torques[2]) * self.gravity_scale
        
        # X-axis: Secondary resistance (elbow + wrist)
        fx = (self.gravity_torques[2] * 0.3 + self.gravity_torques[3]) * self.gravity_scale
        
        # Y-axis: Minimal resistance (waist is mostly unaffected by gravity)
        fy = self.gravity_torques[0] * self.gravity_scale * 0.1
        
        # === VELOCITY DAMPING (makes it harder to move fast) ===
        # This creates viscous resistance - the faster you move, the more you feel it!
        fz -= (self.last_velocities[1] + self.last_velocities[2]) * self.damping
        fx -= self.last_velocities[2] * self.damping * 0.5
        fy -= self.last_velocities[0] * self.damping * 0.3
        
        # === CLAMP TO HARDWARE LIMITS ===
        fx = np.clip(fx, -self.max_force, self.max_force)
        fy = np.clip(fy, -self.max_force, self.max_force)
        fz = np.clip(fz, -self.max_force, self.max_force)
        
        # Debug output every 100 callbacks
        if self.callback_count % 100 == 0:
            total_force = np.sqrt(fx**2 + fy**2 + fz**2)
            self.get_logger().info(
                f"⚡ Force Output: X={fx:.2f}N, Y={fy:.2f}N, Z={fz:.2f}N "
                f"(Total: {total_force:.2f}N)"
            )
        
        feedback.force.x = fx
        feedback.force.y = fy
        feedback.force.z = fz
        
        self.force_pub.publish(feedback)

def main(args=None):
    rclpy.init(args=args)
    node = RobotToGeomagicFeedback()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Training complete! ")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
