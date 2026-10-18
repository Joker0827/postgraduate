#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/log/ros}"
mkdir -p "$ROS_LOG_DIR"

yopo_python="${YOPO_PYTHON:-python3}"
if [[ -z "${YOPO_PYTHON:-}" && -x "$HOME/miniconda3/envs/yopo_ros2/bin/python" ]]; then
  yopo_python="$HOME/miniconda3/envs/yopo_ros2/bin/python"
fi
exec ros2 launch yopo simulation.launch.py "python_executable:=$yopo_python" "$@"
