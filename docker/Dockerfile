# =============================================================================
# ROS Noetic + 方案 B2 自主探索导航 Docker 镜像
# 基础镜像：ubuntu:20.04
# 用户：robot (uid=1000, gid=1000)
# =============================================================================
FROM ubuntu:20.04

# 避免交互式配置提示
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 构建参数：代理（仅用于 git/rosdep，apt 不经过代理）
# 注意：不要使用 ENV 把代理持久化到镜像，否则容器运行时仍会携带 build 时的代理地址
ARG HTTP_PROXY=""
ARG HTTPS_PROXY=""
ARG NO_PROXY="localhost,127.0.0.1"

# 仅在当前 RUN 层生效，不写入镜像环境变量
RUN echo "Build-time proxy: HTTP_PROXY=${HTTP_PROXY}"

# ---------------------------------------------------------------------------
# 1. 更换清华源并安装基础工具
#    注意：ubuntu:20.04 基础镜像缺少 ca-certificates，先用 HTTP 源；
#    同时 unset 代理环境变量，避免 apt 走代理。
# ---------------------------------------------------------------------------
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && \
    cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    sed -i 's|http://.*ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    sed -i 's|http://.*archive.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    sed -i 's|http://.*security.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        curl wget git build-essential cmake python3-pip python3-venv lsb-release gnupg \
        sudo vim net-tools iputils-ping ca-certificates apt-transport-https \
        software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. 安装 ROS Noetic (desktop-full) 使用清华源
#    已有 ca-certificates，可安全使用 HTTPS
# ---------------------------------------------------------------------------
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && \
    sh -c 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    (curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || \
     curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg) && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. 安装 rosdep 并初始化（支持代理重试）
# ---------------------------------------------------------------------------
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && \
    apt-get update && \
    apt-get install -y --no-install-recommends python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool && \
    rm -rf /var/lib/apt/lists/*

# 跳过 rosdep init/update（容器内手动配置代理后执行）
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    (rosdep init 2>/dev/null || true) && \
    echo "[WARN] rosdep init/update skipped during build. Run manually inside container after configuring proxy."

# ---------------------------------------------------------------------------
# 4. 安装方案 B2 全部 ROS 依赖包
# ---------------------------------------------------------------------------
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-move-base-flex \
        ros-noetic-explore-lite \
        ros-noetic-amcl \
        ros-noetic-map-server \
        ros-noetic-pointcloud-to-laserscan \
        ros-noetic-teb-local-planner \
        ros-noetic-dwa-local-planner \
        ros-noetic-global-planner \
        ros-noetic-twist-mux \
        ros-noetic-smach \
        ros-noetic-smach-ros \
        ros-noetic-robot-state-publisher \
        ros-noetic-joint-state-publisher \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-rviz \
        ros-noetic-tf2-tools \
        ros-noetic-joy \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-robot-localization && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 5. 安装 FAST-LIO 编译依赖
# ---------------------------------------------------------------------------
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        libpcl-dev \
        libeigen3-dev \
        ros-noetic-pcl-ros \
        ros-noetic-eigen-conversions \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-cv-bridge && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 6. 创建非 root 用户 robot (uid=1000, gid=1000)
# ---------------------------------------------------------------------------
RUN groupadd -g 1000 robot && \
    useradd -m -u 1000 -g robot -s /bin/bash robot && \
    usermod -aG sudo robot && \
    echo "robot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robot && \
    chmod 0440 /etc/sudoers.d/robot

# ---------------------------------------------------------------------------
# 7. 在容器内创建 catkin_ws 并克隆仓库
# ---------------------------------------------------------------------------
ENV CATKIN_WS=/home/robot/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src && chown -R robot:robot ${CATKIN_WS}

WORKDIR ${CATKIN_WS}/src

# 克隆 GitHub 仓库（容器内手动配置代理后执行）
RUN mkdir -p ${CATKIN_WS}/src && \
    echo "[WARN] Git clone skipped during build. Run manually inside container after configuring proxy."

# ---------------------------------------------------------------------------
# 8. 编译 catkin_ws
# ---------------------------------------------------------------------------
WORKDIR ${CATKIN_WS}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" || \
    echo "[WARN] catkin_make skipped (empty src). Run manually after cloning repos."

# ---------------------------------------------------------------------------
# 9. 配置 robot 用户的 .bashrc
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/robot/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> /home/robot/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/robot/ws_b2/models" >> /home/robot/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=/home/robot/ws_b2/worlds" >> /home/robot/.bashrc && \
    echo "export ROS_WORKSPACE=/home/robot/ws_b2" >> /home/robot/.bashrc && \
    echo "export DISPLAY=:0" >> /home/robot/.bashrc

# ---------------------------------------------------------------------------
# 10. 设置环境变量（ Dockerfile 构建阶段不依赖宿主机 GAZEBO 路径）
# ---------------------------------------------------------------------------
ENV GAZEBO_MODEL_PATH=/home/robot/ws_b2/models
ENV GAZEBO_RESOURCE_PATH=/home/robot/ws_b2/worlds
ENV ROS_WORKSPACE=/home/robot/ws_b2

# ---------------------------------------------------------------------------
# 11. 清理代理配置
# ---------------------------------------------------------------------------
RUN git config --global --unset http.proxy 2>/dev/null || true && \
    git config --global --unset https.proxy 2>/dev/null || true

# 默认工作目录
WORKDIR /home/robot/ws_b2

# 默认以 robot 用户运行
USER robot

CMD ["/bin/bash"]
