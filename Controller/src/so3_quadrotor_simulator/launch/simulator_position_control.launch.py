from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_state = {}
    control_state = {}
    for axis in "xyz":
        value = ParameterValue(LaunchConfiguration("init_" + axis), value_type=float)
        simulation_state["simulator.init_state_" + axis] = value
        control_state["so3_control.init_state_" + axis] = value
    return LaunchDescription([
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="2.0"),
        Node(
            package="so3_quadrotor_simulator",
            executable="quadrotor_simulator_so3",
            name="quadrotor_simulator_so3",
            output="screen",
            parameters=[simulation_state, {"rate.simulation": 1000.0, "rate.odom": 100.0}],
            remappings=[("odom", "/sim/odom"), ("imu", "/sim/imu"), ("cmd", "/so3_cmd")],
        ),
        Node(
            package="so3_control", executable="so3_control_node", name="so3_control",
            output="screen",
            parameters=[control_state, {
                "mass": 0.98,
                "use_external_yaw": False,
                "record_log": False,
                "gains.kx.x": 5.7, "gains.kx.y": 5.7, "gains.kx.z": 6.2,
                "gains.kv.x": 3.4, "gains.kv.y": 3.4, "gains.kv.z": 4.0,
                "gains.rot.x": 1.0, "gains.rot.y": 1.0, "gains.rot.z": 1.0,
                "gains.ang.x": 0.07, "gains.ang.y": 0.07, "gains.ang.z": 0.1,
                "corrections.z": 0.0, "corrections.r": 0.0, "corrections.p": 0.0,
            }],
            remappings=[
                ("odom", "/sim/odom"), ("imu", "/sim/imu"),
                ("position_cmd", "/so3_control/pos_cmd"), ("so3_cmd", "/so3_cmd"),
            ],
        ),
    ])
