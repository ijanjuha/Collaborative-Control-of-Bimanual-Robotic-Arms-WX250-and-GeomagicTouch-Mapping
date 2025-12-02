#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # Resolve launch args at runtime (strings)
    robot_model_val = LaunchConfiguration("robot_model").perform(context)      # e.g. "wx250"
    hardware_type_val = LaunchConfiguration("hardware_type").perform(context)  # e.g. "fake"

    robot_model = LaunchConfiguration("robot_model")
    hardware_type = LaunchConfiguration("hardware_type")

    # Packages
    xsarm_desc_pkg = FindPackageShare("interbotix_xsarm_descriptions")
    xsarm_moveit_pkg = FindPackageShare("interbotix_xsarm_moveit")

    # ----- robot_description (URDF) -----
    urdf_dir = PathJoinSubstitution([
        xsarm_desc_pkg,
        "urdf"
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                urdf_dir, "/",
                robot_model, ".urdf.xacro ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "use_gripper:=true ",
                "show_ar_tag:=false ",
                "show_gripper_bar:=true ",
                "show_gripper_fingers:=true ",
                "use_world_frame:=true ",
                "hardware_type:=", hardware_type, " "
            ]),
            value_type=str,
        )
    }

    # ----- robot_description_semantic (SRDF) -----
    srdf_dir = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "srdf"
    ])

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "xacro ",
                srdf_dir, "/",
                robot_model, ".srdf.xacro ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "show_ar_tag:=false ",
                "external_srdf_loc:=",
            ]),
            value_type=str,
        )
    }

    # --- joint_state_publisher_gui ---
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # --- robot_state_publisher ---
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # --- static world -> base_link TF ---
    # For now we assume "<robot_model>/base_link".
    # We'll correct this once we know the actual root frame from RViz.
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=[
            "0", "0", "0",      # x y z
            "0", "0", "0",      # roll pitch yaw
            "world",
            f"{robot_model_val}/base_link",
        ],
        output="screen",
    )

    # --- MoveIt move_group ---
    kinematics_yaml = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "kinematics.yaml"
    ])
    ompl_yaml = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "ompl_planning.yaml"
    ])

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": False},
            {
                "planning_scene_monitor_options": {
                    "robot_description": "robot_description",
                    "joint_state_topic": "/joint_states",
                }
            },
            kinematics_yaml,
            ompl_yaml,
        ],
    )

    # --- MoveIt Servo ---
    # We'll update planning_frame and ee_frame_name after TF inspection.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_server",
        name="servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
                # PoseStamped input from teleop bridge
                "cartesian_command_in_topic": "/servo_node/pose_cmds",
                "command_in_type": "pose",

                # IMPORTANT: placeholders for now
                "planning_frame": f"{robot_model_val}/base_link",
                "ee_frame_name": f"{robot_model_val}/ee_gripper_link",

                # timing/smoothing
                "publish_period": 0.01,
                "use_gazebo": False,
                "smoothing_filter_plugin_name": "online_signal_smoothing/ButterworthFilter",
            },
        ],
    )

    # --- RViz ---
    rviz_config = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "rviz",
        "xsarm_moveit.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config,
            "-f", "world",
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return [
        jsp_gui_node,
        rsp_node,
        static_tf_node,
        move_group_node,
        servo_node,
        rviz_node,
    ]


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value="wx250",
        description="Interbotix arm model",
    )

    hardware_type_arg = DeclareLaunchArgument(
        name="hardware_type",
        default_value="fake",
        description="actual, fake, or gz_classic",
    )

    return LaunchDescription([
        robot_model_arg,
        hardware_type_arg,
        OpaqueFunction(function=launch_setup),
    ])

