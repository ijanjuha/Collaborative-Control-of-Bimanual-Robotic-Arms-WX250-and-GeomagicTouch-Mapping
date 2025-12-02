# SPDX-License-Identifier: MIT
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    declare_model = DeclareLaunchArgument("robot_model", default_value="wx250")

    # Interbotix MoveIt (simulation)
    xsarm_moveit_share = get_package_share_directory("interbotix_xsarm_moveit")
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(xsarm_moveit_share, "launch", "xsarm_moveit.launch.py")),
        launch_arguments={
            "robot_model": robot_model,
            "use_sim": "true",
        }.items()
    )

    # Virtual Geomagic (Interactive Marker)
    virtual_geomagic = Node(
        package="wx250_geomagic_teleop",
        executable="virtual_geomagic",
        name="virtual_geomagic",
        output="screen",
    )

    # Teleop bridge (Geomagic -> WX250)
    teleop_node = Node(
        package="wx250_geomagic_teleop",
        executable="geomagic_to_wx250",
        name="geomagic_to_wx250",
        output="screen",
        parameters=[
            {"robot_model": robot_model}
            # NOTE: we intentionally do NOT compute/pass base_frame here
        ]
    )

    return LaunchDescription([declare_model, moveit_launch, virtual_geomagic, teleop_node])

