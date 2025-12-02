#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # --- resolve args ---
    robot_model = LaunchConfiguration("robot_model").perform(context)      # e.g. "wx250"
    hardware_type = LaunchConfiguration("hardware_type").perform(context)  # e.g. "fake"

    # --- packages ---
    xsarm_desc_pkg = FindPackageShare("interbotix_xsarm_descriptions")
    xsarm_moveit_pkg = FindPackageShare("interbotix_xsarm_moveit")

    # =========================================================
    # 1) robot_description (URDF via xacro)
    #    /interbotix_xsarm_descriptions/urdf/<model>.urdf.xacro
    # =========================================================
    urdf_xacro_path = PathJoinSubstitution([
        xsarm_desc_pkg,
        "urdf",
        f"{robot_model}.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                urdf_xacro_path,
                " ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "use_gripper:=true ",
                "show_ar_tag:=false ",
                "show_gripper_bar:=true ",
                "show_gripper_fingers:=true ",
                "use_world_frame:=true ",
                "hardware_type:=", hardware_type, " ",
            ]),
            value_type=str,
        )
    }

    # =========================================================
    # 2) robot_description_semantic (SRDF via xacro)
    #    /interbotix_xsarm_moveit/config/srdf/<model>.srdf.xacro
    # =========================================================
    srdf_xacro_path = PathJoinSubstitution([
        xsarm_moveit_pkg,
        "config",
        "srdf",
        f"{robot_model}.srdf.xacro",
    ])

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "xacro ",
                srdf_xacro_path,
                " ",
                "robot_name:=", robot_model, " ",
                "base_link_frame:=base_link ",
                "show_ar_tag:=false ",
                "external_srdf_loc:=",
            ]),
            value_type=str,
        )
    }

    # =========================================================
    # 3) robot_state_publisher
    # =========================================================
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # =========================================================
    # 4) static world -> <robot>/base_link
    # =========================================================
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        # x y z R P Y parent child
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "world",
            f"{robot_model}/base_link",
        ],
        output="screen",
    )

    # =========================================================
    # 5) MoveIt: kinematics + OMPL
    # =========================================================
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

    # =========================================================
    # 6) Interbotix MoveIt Interface (the C++ helper)
    # =========================================================
    interbotix_iface_node = Node(
        package="interbotix_moveit_interface",
        executable="moveit_interface",
        name="interbotix_moveit_interface",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": False},
        ],
    )

    # =========================================================
    # 7) RViz with Interbotix MoveIt config
    #    (this rviz file exists in that pkg — you listed it)
    # =========================================================
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
        arguments=["-d", rviz_config, "-f", "world"],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # IMPORTANT:
    # we are **not** returning joint_state_publisher_gui here
    # because we'll run it separately.
    return [
        rsp_node,
        static_tf_node,
        move_group_node,
        interbotix_iface_node,
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

