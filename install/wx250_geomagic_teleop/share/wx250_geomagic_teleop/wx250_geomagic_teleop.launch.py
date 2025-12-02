# SPDX-License-Identifier: MIT
# Launch: WX250 MoveIt + Geomagic teleop node (simulation)
#
# Usage:
#   ros2 launch wx250_geomagic_teleop wx250_geomagic_teleop.launch.py \
#       robot_model:=wx250 use_sim:=true
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim")

    declare_model = DeclareLaunchArgument("robot_model", default_value="wx250")
    declare_sim   = DeclareLaunchArgument("use_sim", default_value="true")

    # Bring up Interbotix MoveIt for the arm
    xsarm_moveit_share = get_package_share_directory("interbotix_xsarm_moveit")
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xsarm_moveit_share, "launch", "xsarm_moveit.launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "use_sim": use_sim,
        }.items()
    )

    # Teleop node
    teleop_node = Node(
        package="wx250_geomagic_teleop",
        executable="geomagic_to_wx250",
        name="geomagic_to_wx250",
        output="screen",
        parameters=[
            {"robot_model": robot_model},
            {"base_frame": f"{robot_model.perform(None)}/base_link"}
        ]
    )

    return LaunchDescription([declare_model, declare_sim, moveit_launch, teleop_node])
