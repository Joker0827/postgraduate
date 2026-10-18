# ROS 2 Quadrotor Controller And Dynamics

This directory contains the ROS 2 Humble SO(3) controller and quadrotor
dynamics simulator. Build the complete workspace from the repository root:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_ros2.sh
source install/setup.bash
```

Start the complete planner, sensor simulator and controller with:

```bash
bash scripts/run_simulation.sh
```

The default launch uses the network attitude controller and disturbance observer.
Use the position controller with:

```bash
bash scripts/run_simulation.sh controller_mode:=position
```

The standalone controller launch is useful when connecting a real camera and
odometry source:

```bash
ros2 launch so3_control controller_network.launch.py \
  odom_topic:=/vins_estimator/imu_propagate \
  imu_topic:=/mavros/imu/data_raw
```

Simulation topics are `/sim/odom`, `/sim/imu`, `/so3_cmd` and
`/so3_control/pos_cmd`. The network controller exposes the
`/network_controller_node/takeoff_land` `quadrotor_msgs/srv/SetTakeoffLand`
service. The simulator publishes `world -> quadrotor` TF and its private UAV
mesh marker on `/quadrotor_simulator_so3/uav`.

The controller's ROS 2 parameter names and message fields are documented in
the root [ROS 2 guide](../../README_ROS2.md). `SO3Command` uses `k_r` and
`k_om`; command publishers should use reliable QoS, while odometry and IMU
publishers use sensor-data best-effort QoS.
