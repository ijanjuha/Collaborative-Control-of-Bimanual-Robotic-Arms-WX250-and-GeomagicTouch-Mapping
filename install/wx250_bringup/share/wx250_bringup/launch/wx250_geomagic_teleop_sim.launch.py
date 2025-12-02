from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    robot_model = 'wx250'

    # 1) Sim controllers
    xsarm_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('interbotix_xsarm_ros_control'),
                         'launch', 'xsarm_ros_control.launch.py')),
        launch_arguments={'robot_model': robot_model, 'use_gripper':'true'}.items()
    )

    # 2) Robot description as a STRING (pre-run xacro to avoid YAML parsing issues)
    urdf_path = os.path.join(get_package_share_directory('interbotix_xsarm_descriptions'),
                             'urdf', f'{robot_model}.urdf.xacro')
    urdf_xml = xacro.process_file(urdf_path).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(urdf_xml, value_type=str)
        }],
        output='screen'
    )

    # 3) MoveIt bringup (uses the same robot_description already on the param server)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('interbotix_xsarm_moveit'),
                         'launch', 'xsarm_moveit.launch.py')),
        launch_arguments={'robot_model': robot_model, 'use_sim':'true', 'use_gripper':'true'}.items()
    )

    # 4) Your teleop node (expects /phantom/* which you already have)
    teleop = Node(package='control', executable='teleop', output='screen')

    return LaunchDescription([xsarm_sim, rsp, moveit, teleop])

