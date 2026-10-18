import os
import math
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    mode = LaunchConfiguration('controller_mode').perform(context)
    backend = LaunchConfiguration('sensor_backend').perform(context)
    if mode not in ('attitude', 'position'):
        raise ValueError('controller_mode must be attitude or position')
    if backend not in ('cuda', 'cpu'):
        raise ValueError('sensor_backend must be cuda or cpu')
    with open(LaunchConfiguration('sensor_config').perform(context), encoding='utf-8') as stream:
        sensor_config = yaml.safe_load(stream)
    pitch = float(sensor_config['camera'].get('pitch', 0.0))
    if backend == 'cpu' and pitch != 0.0:
        raise ValueError('The original CPU renderer supports camera pitch=0 only')
    dynamics_share = get_package_share_directory('so3_quadrotor_simulator')
    control_launch = os.path.join(dynamics_share, 'launch', f'simulator_{mode}_control.launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={key: LaunchConfiguration(key)
                              for key in ('init_x', 'init_y', 'init_z')}.items()),
        Node(package='sensor_simulator',
             executable='sensor_simulator_cuda' if backend == 'cuda' else 'sensor_simulator',
             name='sensor_simulator_node', output='screen',
             parameters=[{'config_file': LaunchConfiguration('sensor_config')}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--pitch', str(math.radians(pitch)), '--frame-id', 'quadrotor',
                        '--child-frame-id', 'camera_link']),
        Node(package='yopo', executable='test_yopo_ros.py', name='yopo_net',
             prefix=LaunchConfiguration('python_executable'), output='screen',
             parameters=[{
                 'plan_from_reference': mode == 'position',
                 'velocity': ParameterValue(LaunchConfiguration('velocity'), value_type=float),
                 'visualize': ParameterValue(LaunchConfiguration('visualize'), value_type=bool),
                 'use_tensorrt': ParameterValue(LaunchConfiguration('use_tensorrt'), value_type=bool),
                 'weight': LaunchConfiguration('weight'),
                 'goal': [50.0, 0.0, 2.0],
                 'pitch_angle_deg': pitch,
             }]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(get_package_share_directory('yopo'), 'yopo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    from ament_index_python.packages import get_package_prefix
    yopo_prefix = get_package_prefix('yopo')
    return LaunchDescription([
        DeclareLaunchArgument('controller_mode', default_value='attitude'),
        DeclareLaunchArgument('sensor_backend', default_value='cuda'),
        DeclareLaunchArgument('sensor_config', default_value=os.path.join(
            get_package_share_directory('sensor_simulator'), 'config', 'config.yaml')),
        DeclareLaunchArgument('python_executable', default_value='python3'),
        DeclareLaunchArgument('weight', default_value=os.path.join(
            yopo_prefix, 'lib', 'yopo', 'saved', 'YOPO_1', 'epoch50.pth')),
        DeclareLaunchArgument('velocity', default_value='6.0'),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('use_tensorrt', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='2.0'),
        OpaqueFunction(function=launch_nodes),
    ])
