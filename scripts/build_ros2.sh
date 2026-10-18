#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash

# ROS interface generation uses Ubuntu's Python and empy, independently of Conda.
colcon build --base-paths Controller Simulator YOPO --symlink-install \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
