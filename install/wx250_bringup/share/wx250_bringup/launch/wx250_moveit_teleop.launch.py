#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    hardware_type = LaunchConfiguration("hardware_type").perform(context)

    xsarm_desc_pkg = FindPackageShare("interbotix_xsarm_descriptions")
    xsarm_moveit_pkg = FindPackageShare("interbotix_xsarm_moveit")

    # Build absolute xacro paths using .perform() so Humble doesn't choke
    urdf_path = (
        f"{xsarm_desc_pkg.perform(context)}/urdf/{robot_model}.urdf.xacro"
    )
    srdf_path = (
        f"{xsarm_moveit_pkg.perform(context)}/config/srdf/{robot_model}.srdf.xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                urdf_path, " ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "use_gripper:=true ",
                "show_ar_tag:=false ",
                "show_gripper_bar:=true ",
                "show_gripper_fingers:=true ",
                "use_world_frame:=true ",
                "hardware_type:=", hardware_type,
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "xacro ",
                srdf_path, " ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "show_ar_tag:=false ",
                "external_srdf_loc:=",
            ]),
            value_type=str,
        )
    }

    # Publishes TF tree based on /joint_states
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Lets you drag joint sliders if you want
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Give world->wx250/base_link so planning has an anchor frame
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=[
            "0", "0", "0",      # x y z
            "0", "0", "0",      # r p y
            "world", f"{robot_model}/base_link",
        ],
        output="screen",
    )

    # Bring up MoveIt planning backend
    kinematics_yaml = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "kinematics.yaml",
    ])
    ompl_yaml = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "ompl_planning.yaml",
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

    # RViz with MotionPlanning panel
    rviz_config = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "rviz",
        "xsarm_moveit.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config, "-f", "world"],
        parameters=[robot_description, robot_description_semantic],
    )

    return [
        jsp_gui_node,
        rsp_node,
        static_tf_node,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_model",
            default_value="wx250",
            description="Interbotix arm model",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="fake",
            description="actual, fake, or gz_classic",
        ),
        OpaqueFunction(function=launch_setup),
    ])

