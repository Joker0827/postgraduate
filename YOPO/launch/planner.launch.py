import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    weight = os.path.join(get_package_prefix('yopo'), 'lib', 'yopo',
                          'saved', 'YOPO_1', 'epoch50.pth')
    args = [
        DeclareLaunchArgument('python_executable', default_value='python3'),
        DeclareLaunchArgument('weight', default_value=weight),
        DeclareLaunchArgument('odom_topic', default_value='/sim/odom'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_image'),
        DeclareLaunchArgument('ctrl_topic', default_value='/so3_control/pos_cmd'),
        DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('velocity', default_value='6.0'),
        DeclareLaunchArgument('pitch_angle_deg', default_value='0.0'),
        DeclareLaunchArgument('goal_altitude', default_value='2.0'),
        DeclareLaunchArgument('plan_from_reference', default_value='false'),
        DeclareLaunchArgument('use_tensorrt', default_value='false'),
        DeclareLaunchArgument('visualize', default_value='false'),
    ]
    parameters = {name: LaunchConfiguration(name) for name in (
        'weight', 'odom_topic', 'depth_topic', 'ctrl_topic', 'goal_topic', 'world_frame')}
    for name in ('velocity', 'pitch_angle_deg', 'goal_altitude'):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    for name in ('plan_from_reference', 'use_tensorrt', 'visualize'):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
    return LaunchDescription(args + [
        Node(package='yopo', executable='test_yopo_ros.py', name='yopo_net',
             prefix=LaunchConfiguration('python_executable'), output='screen',
             parameters=[parameters]),
    ])
