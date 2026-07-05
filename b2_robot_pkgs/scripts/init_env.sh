#!/bin/bash
# =============================================================================
# init_env.sh
# 容器内环境初始化脚本
# =============================================================================
set -e

echo "========================================"
echo "Initializing ROS Noetic B2 Environment"
echo "========================================"

# Source ROS
source /opt/ros/noetic/setup.bash

# Source project workspace (ws_b2 is the catkin workspace)
if [ -f /home/robot/ws_b2/devel/setup.bash ]; then
    source /home/robot/ws_b2/devel/setup.bash
    echo "[OK] Sourced ws_b2"
else
    echo "[WARN] ws_b2/devel/setup.bash not found. Please run catkin_make in ~/ws_b2"
fi

# Environment variables (override old .bashrc paths)
export DISPLAY=:0
export GAZEBO_MODEL_PATH=/home/robot/ws_b2/src/b2_robot_pkgs/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/home/robot/ws_b2/src/b2_robot_pkgs/worlds:${GAZEBO_RESOURCE_PATH}
export ROS_WORKSPACE=/home/robot/ws_b2

echo "ROS_DISTRO:        ${ROS_DISTRO}"
echo "ROS_PACKAGE_PATH:  ${ROS_PACKAGE_PATH}"
echo "GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}"
echo "========================================"
echo "Environment ready!"
