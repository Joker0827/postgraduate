from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hover_thrust', default_value='0.38'),
        DeclareLaunchArgument('odom_topic', default_value='/vins_estimator/imu_propagate'),
        DeclareLaunchArgument('imu_topic', default_value='/mavros/imu/data_raw'),
        DeclareLaunchArgument('position_cmd_topic', default_value='/so3_control/pos_cmd'),
        Node(
            package='so3_control',
            executable='network_control_node',
            name='network_controller_node',
            output='screen',
            parameters=[{
                'mass': 0.98,
                'is_simulation': False,
                'auto_takeoff': False,
                'use_disturbance_observer': True,
                'hover_thrust': ParameterValue(LaunchConfiguration('hover_thrust'), value_type=float),
                'kx_xy': 5.7,
                'kx_z': 6.2,
                'kv_xy': 3.4,
                'kv_z': 4.0,
                'record_log': False,
            }],
            remappings=[
                ('odom', LaunchConfiguration('odom_topic')),
                ('imu', LaunchConfiguration('imu_topic')),
                ('position_cmd', LaunchConfiguration('position_cmd_topic')),
                ('so3_cmd', '/so3_cmd'),
            ],
        ),
    ])
