#!/usr/bin/env python3
# Compatible with ROS 2 Humble – patched version of Interbotix xsarm_moveit_interface.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from interbotix_xs_modules.xs_common import get_interbotix_xsarm_models
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param


def launch_setup(context, *args, **kwargs):
    # Launch configurations
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    external_srdf_loc = LaunchConfiguration("external_srdf_loc").perform(context)

    # Package paths
    xsarm_desc_pkg = FindPackageShare("interbotix_xsarm_descriptions")
    xsarm_moveit_pkg = FindPackageShare("interbotix_xsarm_moveit")
    config_path = PathJoinSubstitution([xsarm_moveit_pkg, "config"])

    # --- Use sim time flag ---
    use_sim_time_param = determine_use_sim_time_param(
        context=context, hardware_type_launch_arg=LaunchConfiguration("hardware_type")
    )

    # --- Robot Description (URDF) ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    xsarm_desc_pkg,
                    "urdf",
                    f"{robot_model}.urdf.xacro"
                ]),
                " ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "use_gripper:=true ",
                "show_ar_tag:=false ",
                "show_gripper_bar:=true ",
                "show_gripper_fingers:=true ",
                "use_world_frame:=true ",
                "hardware_type:=", hardware_type
            ]),
            value_type=str
        )
    }

    # --- Robot Description Semantic (SRDF) ---
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model,
                config_path=config_path,
            ),
            value_type=str
        )
    }

    # --- Include MoveIt core launch ---
    xsarm_moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("interbotix_xsarm_moveit"),
                "launch",
                "xsarm_moveit.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_model": robot_model,
            "robot_name": robot_name,
            "hardware_type": hardware_type,
            "external_srdf_loc": external_srdf_loc,
            "use_sim_time": "true" if str(use_sim_time_param) == "True" else "false",

        }.items(),
    )

    # --- Interface node (C++ API) ---
    moveit_interface_node = Node(
        package="interbotix_moveit_interface",
        executable="moveit_interface",
        name="interbotix_moveit_interface",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time_param},
        ],
    )

    # --- RViz (MoveIt config) ---
    rviz_config = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "rviz",
        "xsarm_moveit_interface.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return [xsarm_moveit_launch_include, moveit_interface_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            choices=get_interbotix_xsarm_models(),
            description="Model type of the Interbotix Arm (e.g., 'wx250').",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value=LaunchConfiguration("robot_model"),
            description="Name of the robot, usually same as robot_model.",
        ),
        DeclareLaunchArgument(
            "external_srdf_loc",
            default_value=TextSubstitution(text=""),
            description="Optional custom SRDF path.",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="fake",
            description="Hardware type: actual, fake, or gz_classic.",
        ),
    ]

    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar="true",
            show_gripper_fingers="true",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

