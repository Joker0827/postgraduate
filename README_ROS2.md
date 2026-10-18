# YOPO-Simple ROS 2 Humble 版本

本工作区将原始 YOPO-Simple 的规划器、CUDA/CPU 传感器仿真器、四旋翼动力学、
SO3 控制器、自定义消息接口和可视化工具移植到了 ROS 2。网络结构、预训练权重、
轨迹代价、多项式求解器以及控制器和观测器方程均保持原始 YOPO-Simple 的实现。

## 环境要求

- Ubuntu 22.04 / ROS 2 Humble，支持 WSL2 和 NVIDIA GPU。
- CUDA Toolkit 11.8，或与目标 GPU 兼容的 CUDA 版本。
- Eigen3、PCL、OpenCV、yaml-cpp、ROS `cv_bridge` 和 `pcl_conversions`。
- Python 3.10 规划环境，以及 PyTorch、torchvision、NumPy、SciPy、OpenCV 和
  `ruamel.yaml`。现有的 `yopo_ros2` Conda 环境可以配合 ROS 2 Humble 的 Python 3.10
  绑定使用。

如缺少系统依赖，可以执行：

```bash
sudo apt install python3-colcon-common-extensions python3-empy \
  libeigen3-dev libpcl-dev libopencv-dev libyaml-cpp-dev \
  ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-tf2-ros \
  ros-humble-rviz2
```

使用系统 Python 构建。不会使用 Conda 中的 `empy==4.2` 生成 Humble 接口；构建脚本
会显式选择 `/usr/bin/python3`。

```bash
bash scripts/build_ros2.sh
```

CUDA 构建默认使用 RTX 4060 的架构 `89`。如果使用其他 GPU，请显式传入对应架构，
例如 NVIDIA Jetson Orin 使用 `87`：

```bash
bash scripts/build_ros2.sh -DCMAKE_CUDA_ARCHITECTURES=89
```

没有 CUDA Toolkit 的计算机可以关闭 CUDA：

```bash
bash scripts/build_ros2.sh -DENABLE_CUDA=OFF
```

CPU 渲染器保留了原始的有界地图实现，但其性能和地图行为与 CUDA 渲染器不同。

## 闭环仿真

在工作区根目录执行：

```bash
bash scripts/run_simulation.sh
```

如果存在 `~/miniconda3/envs/yopo_ros2/bin/python`，脚本会优先使用它。也可以通过
`YOPO_PYTHON=/path/to/python` 指定其他兼容 Python 3.10 环境。只有规划器使用该解释器，
ROS 2 launch 和 C++ 节点使用系统环境。

默认配置为原始姿态控制器和扰动观测器，规划速度为 6 m/s，初始高度为 2 m，使用预训练
模型 `epoch50.pth`。RViz 的目标工具会向 `/move_base_simple/goal` 发布目标点，目标高度
默认也是 2 m，与原始规划器一致。

```bash
# 使用位置控制器；控制器会根据参考状态自动规划。
bash scripts/run_simulation.sh controller_mode:=position

# 不启动 RViz，也不显示轨迹。
bash scripts/run_simulation.sh rviz:=false visualize:=false

# 使用 CPU 传感器渲染（相机俯仰角必须为零）。
bash scripts/run_simulation.sh sensor_backend:=cpu

# 更换场景、传感器配置和规划速度。
bash scripts/run_simulation.sh sensor_config:=/absolute/path/config.yaml velocity:=3.0
```

启动器会从传感器配置中读取相机俯仰角，并将其传给规划器。深度输入会缩放到原始网络
使用的 `160x96` 分辨率。传感器配置和预训练模型必须保持兼容的相机视场角和深度比例。


| 话题 / 服务                             | ROS 2 类型                           | 约定                                                      |
| --------------------------------------- | ------------------------------------ | --------------------------------------------------------- |
| `/sim/odom`                             | `nav_msgs/msg/Odometry`              | 世界坐标系中的位置和速度，NWU                             |
| `/sim/imu`                              | `sensor_msgs/msg/Imu`                | 使用原始仿真的加速度约定                                  |
| `/depth_image`                          | `sensor_msgs/msg/Image`              | `32FC1`，单位为米；真实相机也可能使用单位为毫米的 `16UC1` |
| `/so3_control/pos_cmd`                  | `quadrotor_msgs/msg/PositionCommand` | 50 Hz 发布位置、速度、加速度和偏航角                      |
| `/so3_cmd`                              | `quadrotor_msgs/msg/SO3Command`      | SO3 控制器输出                                            |
| `/move_base_simple/goal`                | `geometry_msgs/msg/PoseStamped`      | 使用`world` 坐标系                                        |
| `/network_controller_node/takeoff_land` | `quadrotor_msgs/srv/SetTakeoffLand`  | 姿态控制器起飞/降落状态切换                               |

传感器话题使用带短队列的 best-effort QoS，控制消息使用 reliable QoS。激光雷达点云
使用四旋翼机体坐标系，仿真器会同时发布对应的 TF。ROS 2 版本的 `SO3Command` 字段名
使用 `k_r` 和 `k_om`。

## 数据采集与训练

数据集生成器仍然使用 CUDA，并保留原始的数据采样以及深度图/地图序列化方式。开始采集
时会**替换输出目录**，重新运行前请备份已有数据集。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sensor_simulator dataset_generator
# 使用自定义 YAML，包括 save_path/env_num/image_num：
ros2 run sensor_simulator dataset_generator --config /absolute/path/config.yaml

conda activate yopo_ros2
cd YOPO
python train_yopo.py
```

默认输出目录仍为工作区下的 `dataset/`。训练使用 `YOPO/config/traj_opt.yaml`，代价函数
和训练逻辑没有改变。训练应当从 `YOPO/` 目录启动，以保持原始的相对数据集路径。

## Jetson 部署

请在 Jetson 本机上构建工作区并安装 Python 依赖。同时单独启动 ROS 2 相机、里程计和
MAVROS 驱动。本工作区的本地 `mavros_msgs` 包只提供原始控制器所需的消息和服务子集，
不包含 MAVROS 驱动。如果与完整 MAVROS 一起构建，请使用官方的 `mavros_msgs` 包，并
排除本地子集，避免出现同名包冲突。

使用真实相机和里程计话题单独启动规划器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yopo planner.launch.py \
  python_executable:=/path/to/python3.10 \
  odom_topic:=/vins_estimator/imu_propagate \
  depth_topic:=/camera/depth/image_rect_raw \
  pitch_angle_deg:=0.0
ros2 launch so3_control controller_network.launch.py
```

原始实飞接口要求使用 NWU 里程计，并会像之前一样将姿态命令转换为 MAVROS 格式。实飞前
需要确认真实相机的安装方向、模型视场角、悬停推力、控制增益、坐标系和驱动 QoS。当前
迁移版本的仿真检查不能代替真实飞行验证。

TensorRT 仍然是可选项。请在目标 Jetson 上安装兼容的 TensorRT/torch2trt，在目标设备
上生成引擎，然后设置 `use_tensorrt:=true`，并通过 `weight:=...` 传入引擎的绝对路径。
PyTorch 部署继续使用现有模型。

## 验证

```bash
conda activate yopo_ros2
python -m unittest discover -s tests -v
# 在另一个终端保持仿真运行：
source install/setup.bash
python tests/check_closed_loop.py --duration 30
```

闭环检查程序会读取真实的 ROS 2 里程计、深度图、规划器命令和 SO3 输出，检查数值是否
有限、飞行器是否持续运动，并统计各话题的发布频率。
