#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ----- Launch args -----
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

    robot_model = LaunchConfiguration("robot_model")
    hardware_type = LaunchConfiguration("hardware_type")

    # ----- Packages -----
    xsarm_desc_pkg = FindPackageShare("interbotix_xsarm_descriptions")
    xsarm_moveit_pkg = FindPackageShare("interbotix_xsarm_moveit")

    # dirs for xacro inputs
    urdf_dir = PathJoinSubstitution([
        xsarm_desc_pkg,
        "urdf"
    ])

    srdf_dir = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "srdf"
    ])

    # ----- robot_description (URDF) -----
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

    # --- robot_state_publisher ---
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # --- joint_state_publisher_gui ---
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
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

    return LaunchDescription([
        robot_model_arg,
        hardware_type_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node,
    ])

