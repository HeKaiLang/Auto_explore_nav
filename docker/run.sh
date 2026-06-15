#!/bin/bash
# =============================================================================
# Docker 容器运行脚本（方案 B2）
# 支持 GUI (RViz/Gazebo)、ROS 网络通信
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONTAINER_NAME="ros-noetic-b2-container"
IMAGE_NAME="ros-noetic-b2:latest"

echo "========================================"
echo "Running ROS Noetic B2 Container"
echo "Project Root: ${PROJECT_ROOT}"
echo "Container:    ${CONTAINER_NAME}"
echo "========================================"

# 检查容器是否已在运行
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} is already running."
    echo "Attaching to existing container..."
    docker exec -it -u robot "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

# 检查容器是否存在但已停止
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} exists but stopped. Starting..."
    docker start "${CONTAINER_NAME}"
    docker exec -it -u robot "${CONTAINER_NAME}" /bin/bash
    exit 0
fi

# ---------------------------------------------------------------------------
# 检测 WSL2 网络模式并确定容器可用的代理地址
# ---------------------------------------------------------------------------
if timeout 2 bash -c 'cat < /dev/null > /dev/tcp/127.0.0.1/7897' 2>/dev/null; then
    echo "[INFO] WSL2 mirrored mode detected. Using host.docker.internal:7897 for container proxy."
    PROXY="http://host.docker.internal:7897"
else
    HOST_IP="$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')"
    echo "[INFO] WSL2 NAT mode detected. Using ${HOST_IP}:7897 for container proxy."
    PROXY="http://${HOST_IP}:7897"
fi

# 启动新容器
echo "Starting new container..."
docker run -it \
  --name "${CONTAINER_NAME}" \
  --network host \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e HTTP_PROXY="${PROXY}" \
  -e HTTPS_PROXY="${PROXY}" \
  -e NO_PROXY="localhost,127.0.0.1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${PROJECT_ROOT}:/home/robot/ws_b2:rw" \
  -v "${HOME}/.Xauthority:/home/robot/.Xauthority:rw" \
  --privileged \
  "${IMAGE_NAME}" \
  /bin/bash
