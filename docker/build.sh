#!/bin/bash
# =============================================================================
# Docker 镜像构建脚本（方案 B2）
# 支持自动检测 WSL2 宿主机代理（兼容 NAT / Mirrored 模式）
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# ---------------------------------------------------------------------------
# 检测 WSL2 网络模式并确定可用的代理地址
# ---------------------------------------------------------------------------
# 镜像模式下 WSL2 的 127.0.0.1 直接映射到 Windows 主机；
# Docker build 容器内无法直接使用 127.0.0.1 访问宿主机，
# 需使用 host.docker.internal（Docker Desktop WSL2 backend 支持）。
# ---------------------------------------------------------------------------
if timeout 2 bash -c 'cat < /dev/null > /dev/tcp/127.0.0.1/7897' 2>/dev/null; then
    # 镜像模式：127.0.0.1:7897 在 WSL2 内可达
    echo "[INFO] Detected WSL2 mirrored networking mode"
    PROXY="http://host.docker.internal:7897"
else
    # NAT 模式：通过 resolv.conf 获取 Windows 虚拟网卡 IP
    HOST_IP="$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')"
    echo "[INFO] Detected WSL2 NAT networking mode"
    PROXY="http://${HOST_IP}:7897"
fi

echo "========================================"
echo "Building ROS Noetic B2 Docker Image"
echo "Project Root: ${PROJECT_ROOT}"
echo "Script Dir:   ${SCRIPT_DIR}"
echo "Proxy:        ${PROXY}"
echo "========================================"

docker build \
  --build-arg HTTP_PROXY="${PROXY}" \
  --build-arg HTTPS_PROXY="${PROXY}" \
  --build-arg NO_PROXY="localhost,127.0.0.1" \
  -t ros-noetic-b2:latest \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${PROJECT_ROOT}"

echo "========================================"
echo "Build completed: ros-noetic-b2:latest"
echo "========================================"
