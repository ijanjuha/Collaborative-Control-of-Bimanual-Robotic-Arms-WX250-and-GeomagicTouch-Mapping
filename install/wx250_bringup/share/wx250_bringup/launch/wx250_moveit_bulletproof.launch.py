from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml

ROBOT_MODEL = 'wx250'

def read_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # ---- Launch args (so you can tweak without editing code)
    pose_topic_arg   = DeclareLaunchArgument('pose_topic',   default_value='/phantom/pose')
    xy_scale_arg     = DeclareLaunchArgument('xy_scale',     default_value='1.0')
    z_scale_arg      = DeclareLaunchArgument('z_scale',      default_value='1.2')
    rot_scale_arg    = DeclareLaunchArgument('rot_scale',    default_value='1.5')
    frame_id_arg     = DeclareLaunchArgument('planning_frame', default_value='base_link')
    ee_link_arg      = DeclareLaunchArgument('ee_link', default_value='ee_gripper_link')

    pose_topic     = LaunchConfiguration('pose_topic')
    xy_scale       = LaunchConfiguration('xy_scale')
    z_scale        = LaunchConfiguration('z_scale')
    rot_scale      = LaunchConfiguration('rot_scale')
    planning_frame = LaunchConfiguration('planning_frame')
    ee_link        = LaunchConfiguration('ee_link')

    # ---- Paths you already have
    xs_desc = get_package_share_directory('interbotix_xsarm_descriptions')
    urdf_xacro = os.path.join(xs_desc, 'urdf', f'{ROBOT_MODEL}.urdf.xacro')

    # MoveIt SRDF/kin/ompl straight from your SRC tree
    cfg_root  = os.path.expanduser(
        '~/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_moveit/config'
    )
    srdf_xacro = os.path.join(cfg_root, 'srdf', f'{ROBOT_MODEL}.srdf.xacro')
    kin_yaml   = os.path.join(cfg_root, 'kinematics.yaml')
    ompl_yaml  = os.path.join(cfg_root, 'ompl_planning.yaml')

    # Resolve/compute descriptions once (avoid YAML parser hell)
    if not os.path.exists(urdf_xacro):
        raise FileNotFoundError(f'URDF xacro not found: {urdf_xacro}')
    if not os.path.exists(srdf_xacro):
        raise FileNotFoundError(f'SRDF xacro not found: {srdf_xacro}')
    urdf_xml = xacro.process_file(urdf_xacro).toxml()
    srdf_xml = xacro.process_file(srdf_xacro).toxml()

    robot_description            = {'robot_description':            ParameterValue(urdf_xml, value_type=str)}
    robot_description_semantic   = {'robot_description_semantic':   ParameterValue(srdf_xml, value_type=str)}
    robot_description_kinematics = {'robot_description_kinematics': read_yaml(kin_yaml)}
    ompl                         = {'move_group': {'planning_pipelines': ['ompl'], 'ompl': read_yaml(ompl_yaml)}}

    # ---- 1) Interbotix sim controllers (no hardware, no RViz)
    xsarm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('interbotix_xsarm_control'),
                         'launch', 'xsarm_control.launch.py')
        ),
        launch_arguments={
            'robot_model': ROBOT_MODEL,
            'use_sim': 'true',
            'use_gripper': 'true',
            'use_rviz': 'false'
        }.items()
    )

    # ---- 2) MoveIt (bulletproof) with in-memory params
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl,
            {'use_sim_time': True},
        ]
    )

    # RViz: use any MoveIt config if present; otherwise vanilla RViz is fine
    rviz_cfg = os.path.join(cfg_root, 'moveit.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics]
    )

    # ---- 3) Servo node with same robot_description injected + your servo.yaml merged
    servo_yaml_path = os.path.expanduser('~/interbotix_ws/src/wx250_bringup/config/servo.yaml')
    servo_yaml = read_yaml(servo_yaml_path) if os.path.exists(servo_yaml_path) else {'servo_node': {'ros__parameters': {}}}
    servo_params = servo_yaml.get('servo_node', {}).get('ros__parameters', {})
    # Override/ensure frames from launch args (LCs are fine in params)
    servo_params.update({
        'planning_frame': planning_frame,
        'ee_frame_name':  ee_link,
    })

    servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
            servo_params
        ]
    )

    # ---- 4) Geomagic mapper: pose → /servo_node/delta_twist_cmds
    geomagic_mapper = Node(
        package='wx250_bringup',
        executable='geomagic_servo',
        name='geomagic_to_servo',
        output='screen',
        parameters=[
            {'pose_topic':    pose_topic},
            {'frame_id':      planning_frame},
            {'xy_scale':      xy_scale},
            {'z_scale':       z_scale},
            {'rot_scale':     rot_scale},
            {'deadband_lin':  0.002},
            {'deadband_ang':  0.02},
            {'zero_on_start': True},
        ]
    )

    return LaunchDescription([
        pose_topic_arg, xy_scale_arg, z_scale_arg, rot_scale_arg, frame_id_arg, ee_link_arg,
        xsarm_control,
        move_group,
        servo,
        rviz,
        geomagic_mapper
    ])

