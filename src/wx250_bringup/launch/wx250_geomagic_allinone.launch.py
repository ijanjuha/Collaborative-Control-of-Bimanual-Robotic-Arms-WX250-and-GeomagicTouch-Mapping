#!/usr/bin/env python3

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import launch.conditions
import yaml


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation mode'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    # Get package shares
    xsarm_moveit_share = get_package_share_directory('interbotix_xsarm_moveit')
    xsarm_descriptions_share = get_package_share_directory('interbotix_xsarm_descriptions')
    
    # Robot description using xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            xsarm_descriptions_share,
            'urdf',
            'wx250.urdf.xacro'
        ]),
        ' robot_name:=wx250',
        ' base_link_frame:=base_link',
        ' use_world_frame:=false',
        ' external_urdf_loc:=""',
        ' load_gazebo_configs:=false'
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # SRDF - try multiple possible locations and extensions
    srdf_paths = [
        os.path.join(xsarm_moveit_share, 'config', 'srdf', 'wx250.srdf.xacro'),
        os.path.join(xsarm_moveit_share, 'config', 'srdf', 'wx250.srdf'),
        os.path.join(xsarm_moveit_share, 'srdf', 'wx250.srdf.xacro'),
        os.path.join(xsarm_moveit_share, 'srdf', 'wx250.srdf'),
    ]
    
    srdf_file = None
    for path in srdf_paths:
        if os.path.exists(path):
            srdf_file = path
            break
    
    if srdf_file is None:
        raise FileNotFoundError(f"Could not find wx250.srdf or wx250.srdf.xacro in any of: {srdf_paths}")
    
    print(f"[LAUNCH] Using SRDF from: {srdf_file}")
    
    # Process SRDF (handle both .srdf and .srdf.xacro)
    if srdf_file.endswith('.xacro'):
        robot_description_semantic_content = Command([
            FindExecutable(name='xacro'), ' ',
            srdf_file,
            ' name:=wx250',
            ' base_link_frame:=base_link',
            ' use_world_frame:=false',
        ])
        robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}
    else:
        with open(srdf_file, 'r') as f:
            robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics - search multiple locations (optional, may not exist)
    kinematics_paths = [
        os.path.join(xsarm_moveit_share, 'config', 'kinematics.yaml'),
        os.path.join(xsarm_moveit_share, 'kinematics.yaml'),
    ]
    
    kinematics_file = None
    for path in kinematics_paths:
        if os.path.exists(path):
            kinematics_file = path
            break
    
    if kinematics_file:
        print(f"[LAUNCH] Using kinematics from: {kinematics_file}")
        with open(kinematics_file, 'r') as f:
            kinematics_yaml = yaml.safe_load(f)
    else:
        print(f"[LAUNCH] No kinematics.yaml found - will use default kinematics")
        kinematics_yaml = {
            'robot_description_kinematics': {
                'interbotix_arm': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.005,
                    'kinematics_solver_timeout': 0.005,
                }
            }
        }
    
    # Joint limits - search multiple locations
    joint_limits_paths = [
        os.path.join(xsarm_moveit_share, 'config', 'joint_limits', 'wx250_joint_limits.yaml'),
        os.path.join(xsarm_moveit_share, 'config', 'joint_limits.yaml'),
        os.path.join(xsarm_moveit_share, 'joint_limits.yaml'),
        os.path.join(xsarm_moveit_share, 'config', 'wx250_joint_limits.yaml'),
    ]
    
    joint_limits_file = None
    for path in joint_limits_paths:
        if os.path.exists(path):
            joint_limits_file = path
            break
    
    if joint_limits_file:
        print(f"[LAUNCH] Using joint limits from: {joint_limits_file}")
        with open(joint_limits_file, 'r') as f:
            joint_limits_yaml = yaml.safe_load(f)
    else:
        print(f"[LAUNCH] WARNING: joint_limits.yaml not found, using empty config")
        joint_limits_yaml = {}
    
    # Launch interbotix control (xs_sdk_sim + robot_state_publisher)
    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': 'wx250',
            'robot_name': 'wx250',
            'base_link_frame': 'base_link',
            'use_world_frame': 'false',
            'use_rviz': 'false',
            'use_sim': LaunchConfiguration('use_sim'),
        }.items()
    )
    
    # MoveIt move_group node
    move_group_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    joint_limits_yaml,
                    {'use_sim_time': True},
                    {'publish_robot_description_semantic': True},
                    {'publish_robot_description': True},
                    {
                        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                    }
                ],
                remappings=[
                    ('/joint_states', '/wx250/joint_states'),
                ],
            )
        ]
    )
    
    # Load servo config file path
    home = str(Path.home())
    servo_yaml_path = os.path.join(home, 'interbotix_ws', 'src', 'wx250_bringup', 'config', 'servo_namespaced.yaml')
    
    if not os.path.exists(servo_yaml_path):
        servo_yaml_path = os.path.join(home, 'interbotix_ws', 'src', 'wx250_bringup', 'config', 'servo.yaml')
    
    if not os.path.exists(servo_yaml_path):
        raise FileNotFoundError(f"Servo config not found at: {servo_yaml_path}")
    
    print(f"[LAUNCH] Using servo config: {servo_yaml_path}")
    
    # Custom Python Servo node
    servo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='wx250_bringup',
                executable='custom_servo_node',
                name='servo_node',
                output='screen',
                parameters=[{
                    'move_group_name': 'interbotix_arm',
                    'planning_frame': 'wx250/base_link',
                    'ee_frame': 'wx250/ee_gripper_link',
                    'publish_period': 0.01,
                    'linear_scale': 1.0,
                    'angular_scale': 1.5,
                    'use_sim_time': True,
                }],
            )
        ]
    )
    
    # Bridge node
    bridge_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='wx250_bringup',
                executable='servo_to_interbotix_bridge',
                name='servo_to_interbotix_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Geomagic mapper
    geomagic_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='wx250_bringup',
                executable='geomagic_servo',
                name='geomagic_to_servo',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'scale': 2.0,
                }]
            )
        ]
    )
    

    feedback_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='wx250_bringup',
                executable='robot_to_geomagic_feedback',
                name='robot_feedback',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'gravity_force_scale': 0.5,   # Tune: 0.3 (light) to 0.7 (intense)
                    'damping': 1.0,                # Tune: 0.5 (easy) to 1.5 (hard)
                    'max_force': 3.3,              # Geomagic Touch hardware limit
                }]
            )
        ]
    )
    
    # Optional RViz
    rviz_config = os.path.join(xsarm_moveit_share, 'rviz', 'xsarm_moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        xsarm_control_launch,
        move_group_node,
        servo_node,
        bridge_node,
        geomagic_node,
        feedback_node,  
        rviz_node,
    ])
