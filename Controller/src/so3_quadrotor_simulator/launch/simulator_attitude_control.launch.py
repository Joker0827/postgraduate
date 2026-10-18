from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    initial_state = {
        "simulator.init_state_" + axis: ParameterValue(
            LaunchConfiguration("init_" + axis), value_type=float
        )
        for axis in "xyz"
    }
    return LaunchDescription([
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="2.0"),
        Node(
            package="so3_quadrotor_simulator",
            executable="quadrotor_simulator_so3",
            name="quadrotor_simulator_so3",
            output="screen",
            parameters=[initial_state, {"rate.simulation": 1000.0, "rate.odom": 100.0}],
            remappings=[("odom", "/sim/odom"), ("imu", "/sim/imu"), ("cmd", "/so3_cmd")],
        ),
        Node(
            package="so3_control",
            executable="network_control_node",
            name="network_controller_node",
            output="screen",
            parameters=[{
                "is_simulation": True,
                "auto_takeoff": True,
                "takeoff_altitude": ParameterValue(LaunchConfiguration("init_z"), value_type=float),
                "use_disturbance_observer": True,
                "hover_thrust": 0.375,
                "record_log": False,
            }],
            remappings=[
                ("odom", "/sim/odom"), ("imu", "/sim/imu"),
                ("position_cmd", "/so3_control/pos_cmd"), ("so3_cmd", "/so3_cmd"),
            ],
        ),
    ])
