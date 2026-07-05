#!/bin/bash
# =============================================================================
# setup_real_robot.sh
# 实机部署脚本：为已搭载 FAST-LIO 的机器人安装自主探索导航依赖
# =============================================================================
set -e

ROS_DISTRO=${ROS_DISTRO:-noetic}
WORKSPACE="${HOME}/ws_b2_real"  # 实机工作空间路径，可修改
SRC_DIR="${WORKSPACE}/src"

echo "========================================"
echo "  自主探索框架 — 实机部署脚本"
echo "  ROS Distro: ${ROS_DISTRO}"
echo "  Workspace:  ${WORKSPACE}"
echo "========================================"
echo ""

# -----------------------------------------------------------------------------
# 1. 检查 ROS 环境
# -----------------------------------------------------------------------------
if [ -z "${ROS_ROOT}" ]; then
    echo "[ERROR] 未检测到 ROS 环境，请先 source /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

echo "[1/6] ROS 环境检测通过: ${ROS_ROOT}"

# -----------------------------------------------------------------------------
# 2. 创建工作空间
# -----------------------------------------------------------------------------
if [ ! -d "${WORKSPACE}" ]; then
    echo "[2/6] 创建工作空间: ${WORKSPACE}"
    mkdir -p "${SRC_DIR}"
    cd "${WORKSPACE}"
    catkin_init_workspace 2>/dev/null || true
else
    echo "[2/6] 工作空间已存在: ${WORKSPACE}"
fi

# -----------------------------------------------------------------------------
# 3. 安装系统依赖（apt）
# -----------------------------------------------------------------------------
echo "[3/6] 安装系统依赖包..."

sudo apt-get update

# 核心导航包
sudo apt-get install -y \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-mbf-costmap-nav \
    ros-${ROS_DISTRO}-mbf-msgs \
    ros-${ROS_DISTRO}-explore-lite \
    ros-${ROS_DISTRO}-teb-local-planner \
    ros-${ROS_DISTRO}-global-planner \
    ros-${ROS_DISTRO}-dwa-local-planner \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-clear-costmap-recovery \
    ros-${ROS_DISTRO}-rotate-recovery \
    ros-${ROS_DISTRO}-nav-core \
    ros-${ROS_DISTRO}-costmap-2d

# 工具包（通常已安装，但保险起见）
sudo apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-tf2-ros

echo "[3/6] 系统依赖安装完成"

# -----------------------------------------------------------------------------
# 4. 克隆用户自定义包
# -----------------------------------------------------------------------------
echo "[4/6] 克隆/更新用户自定义包..."

cd "${SRC_DIR}"

# 如果已经存在，先备份或删除
if [ -d "b2_robot_pkgs" ]; then
    echo "      b2_robot_pkgs 已存在，执行 git pull..."
    cd b2_robot_pkgs && git pull && cd ..
else
    echo "      克隆 b2_robot_pkgs..."
    git clone https://github.com/HeKaiLang/Auto_explore_nav.git b2_robot_pkgs
fi

echo "[4/6] 用户自定义包准备完成"

# -----------------------------------------------------------------------------
# 5. 清理仿真专用依赖（避免编译错误）
# -----------------------------------------------------------------------------
echo "[5/6] 清理仿真专用依赖..."

# 检查并移除 Gazebo 相关依赖（如果 b2_robot_pkgs 的 package.xml 里有）
B2_PKG_XML="${SRC_DIR}/b2_robot_pkgs/package.xml"
if [ -f "${B2_PKG_XML}" ]; then
    # 备份原文件
    cp "${B2_PKG_XML}" "${B2_PKG_XML}.bak"

    # 注释掉 gazebo 相关依赖（如果有的话）
    sed -i 's|<depend>gazebo_ros</depend>|<!-- <depend>gazebo_ros</depend> -->|g' "${B2_PKG_XML}" 2>/dev/null || true
    sed -i 's|<depend>gazebo_ros_pkgs</depend>|<!-- <depend>gazebo_ros_pkgs</depend> -->|g' "${B2_PKG_XML}" 2>/dev/null || true

    echo "      已清理 b2_robot_pkgs/package.xml 中的 Gazebo 依赖"
fi

# 提示：Mid360_px4_sim_plugin 是仿真插件，实机不需要
echo "      [提示] Mid360_px4_sim_plugin 为仿真插件，实机无需安装"
echo "      [提示] livox_ros_driver 假设已在实机上安装（FAST-LIO 依赖）"

echo "[5/6] 清理完成"

# -----------------------------------------------------------------------------
# 6. 编译工作空间
# -----------------------------------------------------------------------------
echo "[6/6] 编译工作空间..."
cd "${WORKSPACE}"
catkin_make -j$(nproc)

echo ""
echo "========================================"
echo "  编译完成！"
echo "========================================"
echo ""

# -----------------------------------------------------------------------------
# 7. 实机部署关键提示
# -----------------------------------------------------------------------------
cat << 'EOF'

【重要】实机部署后，请手动完成以下修改：

1. 修改 simulation_bringup.launch → real_robot_bringup.launch
   - 删除/注释掉 Gazebo 启动部分（empty_world、spawn_model）
   - 删除/注释掉 Gazebo 插件（diff_drive、IMU、LiDAR 仿真插件）
   - 保留 robot_state_publisher 和 joint_state_publisher
   - 保留 robot_description 的 xacro 加载（用于 TF 和可视化）

2. 确认实机底盘驱动
   - 实机通常已有底盘驱动节点发布 /odom 和 /cmd_vel
   - 确认底盘驱动的 /odom 坐标系是否与你的 TF 树一致
   - 如果底盘驱动发布 odom → base_link 的 TF，需要检查是否与 FAST-LIO 冲突

3. 修改 exploration.launch
   - 将 <include file="$(find fast_lio)/launch/mapping_mid360.launch"/> 保留
   - 删除/注释掉 Gazebo 相关的 static_transform_publisher（如果有）
   - 确认 body → base_link 的 static TF 是否仍然需要

4. 确认实机 TF 树
   - 必须存在: camera_init → body → base_link → livox_link
   - 如果实机底盘驱动发布 odom → base_link，确保不与 FAST-LIO 冲突
   - 建议让 FAST-LIO 的 camera_init 作为唯一全局坐标系

5. 修改 twist_mux 配置
   - 确认 /nav_cmd_vel 是 MBF 的输出话题
   - 确认 /joy_cmd_vel 或 /keyboard_cmd_vel 是否存在于实机
   - 实机可能需要添加急停（e-stop）锁

6. 测试顺序（建议）
   (1) 启动 FAST-LIO: roslaunch fast_lio mapping_mid360.launch
   (2) 检查 TF: rosrun rqt_tf_tree rqt_tf_tree
   (3) 检查 /scan: rostopic hz /scan
   (4) 启动 bringup: roslaunch b2_robot_pkgs real_robot_bringup.launch
   (5) 启动探索: roslaunch b2_robot_pkgs exploration.launch
   (6) 检查 costmap: rosrun b2_robot_pkgs check_costmap.py

7. 安全提示
   - 首次测试请使用低速参数（max_vel_x: 0.2）
   - 确保有远程急停手段（无线开关或键盘）
   - 在开阔场地测试，避免 explore_lite 引导机器人撞墙

EOF

echo ""
echo "请执行: source ${WORKSPACE}/devel/setup.bash"
echo ""
