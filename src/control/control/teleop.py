#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, asin, degrees
from omni_msgs.msg import OmniButtonEvent
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# Global variables to store orientation values (from stylus joints)
roll_mapped = 0.0
pitch_remapped = 0.0

# Transformation matrix to map Geomagic coordinates to Interbotix RX200 frame
T = np.array([
    [1,  0,  0,  0.0],     # No change to X
    [0,  1,  0, -0.2],     # Y is shifted left by 0.2 meters
    [0,  0,  1,  0.10],    # Z is lifted up by 0.1 meters
    [0,  0,  0,  1.0]
])

# Function to apply transformation matrix to Geomagic position
def transform_point(p_geomagic):
    # Convert to homogeneous coordinates and apply a +0.06m Z offset
    p_hom = np.array([p_geomagic[0], p_geomagic[1], p_geomagic[2]+0.06, 1.0])
    # Multiply by transformation matrix
    p_transformed = T @ p_hom
    return p_transformed[:3]  # Return only x, y, z

# Main class that handles input from Geomagic and controls Interbotix robot
class PhantomPoseToInterbotix(Node):
    def __init__(self, bot):
        super().__init__('phantom_to_interbotix_node')

        # Interbotix robot object
        self.bot = bot
        self.bot.gripper.release()  # Start with gripper open

        # Time trackers to throttle callback frequency to 50 Hz
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()

        # Button states
        self.white_button_pressed = False
        self.grey_button_pressed = False

        # Subscribe to pose of stylus
        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/pose")

        # Subscribe to button events
        self.subscription = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        # Subscribe to joint states from stylus
        self.joint_sub = self.create_subscription(
            JointState,
            'phantom/joint_states',
            self.joint_callback,
            10
        )

    # (Optional) Callback for checking mimic robot’s validity
    def mimic_valid_cb(self, msg: Bool):
        self.mimic_valid = msg.data

    # Button press logic for controlling gripper and mode switching
    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        # Gripper close only when grey is pressed and white is NOT pressed
        if grey == 1 and white == 0:
            self.bot.gripper.grasp(delay=0.0)
            self.get_logger().info("Gripper closed.")
        else:
            self.bot.gripper.release(delay=0.0)

        # Save button states
        self.white_button_pressed = white == 1
        self.grey_button_pressed = grey == 1

    # Pose callback handles XYZ positioning of end-effector
    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()

        # Throttle callback to 50 Hz
        if (now - self.last_pose_time).nanoseconds < 20_000_000:
            return
        self.last_pose_time = now

        # Only allow position control if both buttons are released
        if self.white_button_pressed == False and self.grey_button_pressed == False:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]
            p_inter = transform_point(p_geomagic)

            # Move end-effector to target XYZ with current orientation
            self.bot.arm.set_ee_pose_components(
                x=p_inter[0]*2,     # X and Y are scaled up for sensitivity
                y=p_inter[1]*2,
                z=p_inter[2],
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )

        # Allow position update while gripping (grey button pressed alone)
        if self.white_button_pressed == False and self.grey_button_pressed == True:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]
            p_inter = transform_point(p_geomagic)

            self.bot.arm.set_ee_pose_components(
                x=p_inter[0]*2,
                y=p_inter[1]*2,
                z=p_inter[2],
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )

        # Commented out experimental code for restricted motion while holding
        '''
        if self.white_button_pressed == True and self.grey_button_pressed == True:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]
            p_inter = transform_point(p_geomagic)
            if self.x1 > 0.25:
                self.x1 = 0.25

            self.bot.arm.set_ee_pose_components(
                x=self.x1,
                y=p_inter[1]*2,
                z=p_inter[2],
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )
        
        if self.white_button_pressed == True and self.grey_button_pressed == False:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]
            p_inter = transform_point(p_geomagic)

            self.bot.arm.set_ee_pose_components(
                x=p_inter[0]*2,
                y=p_inter[1]*2,
                z=p_inter[2],
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )
        '''

    # Joint callback handles roll and pitch orientation updates
    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()

        # Throttle callback to 50 Hz
        if (now - self.last_pose_time).nanoseconds < 20_000_000:
            return
        self.last_pose_time = now

        # Only update orientation if not in grip-only mode (grey-only)
        if not (self.white_button_pressed == False and self.grey_button_pressed == True):
            # Create name-to-position mapping from JointState
            name_to_pos = dict(zip(msg.name, msg.position))

            # Read roll and pitch from stylus
            roll = name_to_pos.get('roll', 0.0)
            pitch = name_to_pos.get('pitch', 0.0)

            # Offset to map stylus joint values to robot's frame
            roll += 2.61
            pitch += 2.60

            # Save to global variables
            global roll_mapped
            roll_mapped = roll
            global pitch_remapped
            pitch_remapped = pitch  # You can clamp here if needed

# Main function to start ROS 2 node and run the event loop
def main(args=None):
    rclpy.init(args=args)

    # Initialize Interbotix RX200 robot
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        group_name='arm',
        gripper_name='gripper'
    )
    robot_startup()  # Enable motors and home the arm

    node = PhantomPoseToInterbotix(bot)

    try:
        rclpy.spin(node)  # Spin the node (event loop)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_shutdown()  # Turn off motors
        rclpy.shutdown()

# Entry point
if __name__ == '__main__':
    main()
