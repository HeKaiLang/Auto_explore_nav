# mine_map_deploy 部署包

> 矿山/隧道场景：SLAM 点云 → 全局地图 → 2D 栅格图 → 手柄端 HTTP 地图服务

---

## 文件结构

```
mine_map_deploy/
├── CMakeLists.txt                 # 编译配置 (2 个 C++ 可执行文件)
├── package.xml                    # ROS 包依赖声明
├── README.md                      # 本文件
├── config/
│   └── global_map_listener.yaml   # 全局地图节点运行时参数
├── launch/
│   ├── global_map_listener.launch # 启动 global_map_listener 节点
│   └── image_publisher.launch     # 启动 image_publisher 节点
├── scripts/
│   └── grid_http_server.py        # HTTP 地图服务 (Python, 手柄端入口)
└── src/
    ├── global_map_listen.cpp      # 全局点云累积节点 (C++)
    └── image_publisher.cpp        # 点云→2D 栅格图投影节点 (C++)
```

---

## 总体数据链路

```
┌─────────────────┐
│  FAST-LIO 建图   │  发布 /cloud_registered (sensor_msgs::PointCloud2, frame: camera_init)
│  laserMapping   │  每帧 ~数万点，坐标已在世界坐标系下
└────────┬────────┘
         │
    ┌────▼────────────────────────────────────────────┐
    │  global_map_listener  (C++)                      │
    │  =============================================== │
    │  订阅: /cloud_registered                         │
    │  功能: 增量插入体素哈希表 (VoxelHash, 天然去重)    │
    │        leaf_size 控制体素尺寸                     │
    │        max_voxel_count 防止 OOM                  │
    │  发布: /global_map (sensor_msgs::PointCloud2,     │
    │         latch, 仅在新增体素时发布)                 │
    │  服务: /save_global_map → 导出 PCD 文件           │
    └────┬────────────────────────────────────────────┘
         │
    ┌────▼────────────────────────────────────────────┐
    │  image_publisher  (C++)                          │
    │  =============================================== │
    │  订阅: /global_map (内部缓存最新点云)              │
    │  服务: /generate_grid → 触发地图生成               │
    │  功能: z_min~z_max 高度过滤 + XY 正交投影          │
    │        生成 nav_msgs::OccupancyGrid              │
    │  发布: /global_map_grid (latch, 手柄端拉取前已就绪) │
    └────┬────────────────────────────────────────────┘
         │
    ┌────▼────────────────────────────────────────────┐
    │  grid_http_server  (Python)                      │
    │  =============================================== │
    │  订阅: /global_map_grid → 转 PNG 图片             │
    │        /yolo/detection_area → marker 叠加         │
    │  HTTP: :8765/map.png      → 带 marker 的地图 PNG  │
    │        :8765/trigger      → 触发 /generate_grid   │
    │        :8765/markers.json → YOLO 检测点 JSON      │
    │        :8765/select (POST)→ 手柄选中点回传         │
    │  发布: /selected_markers  (回传选中点)             │
    └─────────────────────────────────────────────────┘
```

---

## 节点详解

### 节点 1: `global_map_listener` (C++)

**源文件**: `src/global_map_listen.cpp`

**核心功能**: 订阅 FAST-LIO 前端输出的配准点云 `/cloud_registered`，以体素哈希表增量累积全局 3D 地图。
体素哈希表天然去重——同一体素格内只保留最新观测点，内存仅与已探索空间体积相关，不会随时间无限增长。

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cloud_registered` | `sensor_msgs::PointCloud2` | FAST-LIO 前端输出的配准点云，坐标系 `camera_init` |

#### 发布话题

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `<publish_topic>` (默认 `/global_map`) | `sensor_msgs::PointCloud2` | latch | 全局累积点云，仅在新体素加入时发布 |

> `publish_enable=false` 时禁用发布；`publish_rate=0` 时同样不发布。

#### 提供服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/save_global_map` | `std_srvs::Trigger` | 将当前全局地图导出为 PCD 文件 |

返回格式: `{success: bool, message: "Saved N points to <path>"}`

#### ROS 参数 (~ 私有命名空间)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `leaf_size` | float | `0.1` | 体素降采样栅格尺寸 [m]。越小地图越精细但内存越大 |
| `save_dir` | string | `"."` | PCD 文件保存目录 |
| `publish_enable` | bool | `true` | 是否启用定时发布 |
| `publish_topic` | string | `"/global_map"` | 发布话题名 |
| `publish_rate` | double | `1.0` | 发布频率 [Hz] |
| `max_voxel_count` | size_t | `0` | 体素数量上限，0=不限。达到上限后拒绝新体素，仅更新已有体素 |
| `frames_per_filter` | int | `50` | (旧版保留) 累积帧数阈值 |

#### 实现要点

- **VoxelKey 哈希**: 将 3D 坐标量化为整数体素索引 `(ix, iy, iz)`，使用自定义哈希函数（乘法哈希），O(1) 查找/插入
- **懒惰发布**: 记录 `published_voxel_count_`，仅在体素数变化时才重新生成 `PointCloud2` 消息并发布
- **线程安全**: `std::mutex map_mutex_` 保护哈希表读写，回调与定时器互斥
- **安全上限**: `max_voxel_count_` 防止极端大场景 OOM

#### 启动方式

```bash
roslaunch mine_map_deploy global_map_listener.launch
# 或
rosrun mine_map_deploy global_map_listener _leaf_size:=0.05 _publish_rate:=2.0
```

---

### 节点 2: `image_publisher` (C++)

**源文件**: `src/image_publisher.cpp`

**核心功能**: 接收 3D 全局点云，通过高度过滤 + XY 正交投影生成 2D 占据栅格地图 (`nav_msgs::OccupancyGrid`)。
生成操作由外部服务触发，不会自动连续生成（避免 CPU 浪费）。

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `<input_topic>` (默认 `/global_map`) | `sensor_msgs::PointCloud2` | 输入 3D 点云，内部缓存最新一帧 |

#### 发布话题

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `<output_topic>` (默认 `/global_map_grid`) | `nav_msgs::OccupancyGrid` | latch | 2D 占据栅格地图 |

#### 提供服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `<trigger_service>` (默认 `/generate_grid`) | `std_srvs::Trigger` | 触发一次地图生成并发布 |

#### ROS 参数 (~ 私有命名空间)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `"/global_map"` | 输入点云话题 |
| `output_topic` | string | `"/global_map_grid"` | 输出栅格话题 |
| `trigger_service` | string | `"/generate_grid"` | 触发服务名 |
| `frame_id` | string | `"camera_init"` | 栅格坐标系（点云无 frame 时使用） |
| `resolution` | double | `0.1` | 栅格分辨率 [m/像素] |
| `z_min` | double | `-1.0` | 高度过滤下限 [m]，低于此高度的点被丢弃 |
| `z_max` | double | `1.0` | 高度过滤上限 [m]，高于此高度的点被丢弃 |

#### 实现要点

- **按需生成**: 仅在被触发时计算，避免每帧重算栅格图
- **高度过滤**: `z_min ~ z_max` 截取地面附近点云，滤除天花板/地面以下噪点
- **占据判定**: 投影到栅格的像素直接设为 `100`（占据），其余为 `-1`（未知）
- **自动边界**: 先遍历所有有效点计算 `(min_x, min_y, max_x, max_y)`，再据此分配栅格尺寸

#### 启动方式

```bash
roslaunch mine_map_deploy image_publisher.launch
# 或
rosrun mine_map_deploy image_publisher _resolution:=0.05 _z_min:=-0.5 _z_max:=0.5
```

---

### 节点 3: `grid_http_server` (Python)

**源文件**: `scripts/grid_http_server.py`

**核心功能**: 将 ROS 中的 `OccupancyGrid` 实时转换为 PNG 图片，通过 HTTP 服务提供给手柄端。
手柄端无需安装 ROS，仅需浏览器或 HTTP 客户端。

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/global_map_grid` | `nav_msgs::OccupancyGrid` | 2D 栅格地图，收到后自动转 PNG |
| `/yolo/detection_area` | `visualization_msgs::Marker` | YOLO 检测结果，叠加到地图上 |

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/selected_markers` | `visualization_msgs::Marker` (SPHERE_LIST) | 手柄端回传的选中点，发布供其他节点使用 |

#### HTTP API

| 方法 | 路径 | Content-Type | 说明 |
|------|------|-------------|------|
| `GET` | `/map.png` | `image/png` | 最新栅格地图 PNG（含 YOLO 标记叠加） |
| `GET` | `/trigger` | `application/json` | 触发 `/generate_grid` 服务，返回 `{success, message}` |
| `GET` | `/markers.json` | `application/json` | 原始 YOLO 检测标记点列表 |
| `POST` | `/select` | `application/json` | 手柄端回传选中点 |

**POST /select 请求体**:
```json
{
  "points": [
    {"wx": 1.23, "wy": 4.56, "label": "defect"},
    {"wx": -3.21, "wy": 0.12, "label": "person"}
  ]
}
```

**GET /markers.json 响应体**:
```json
{
  "markers": [
    {"wx": 1.23, "wy": 4.56, "r": 255, "g": 50, "b": 30, "label": "defect"},
    {"wx": -3.21, "wy": 0.12, "r": 30, "g": 200, "b": 60, "label": "person"}
  ]
}
```

#### 颜色映射

- 栅格空闲 (0) → 白色 (255,255,255)
- 栅格占据 (100) → 黑色 (0,0,0)
- 栅格未知 (-1) → 灰色 (127,127,127)
- YOLO 标记 → 按原始颜色 (`Marker.colors`) 绘制圆形 + 坐标标签

#### 启动方式

```bash
rosrun mine_map_deploy grid_http_server.py
# 监听 0.0.0.0:8765，无需额外参数
```

---

## 编译

```bash
# 1. 将 mine_map_deploy 放入 catkin workspace 的 src 目录
cp -r mine_map_deploy ~/catkin_mine_deploy/src/

# 2. 编译 (生成 global_map_listener 和 image_publisher 两个可执行文件)
cd ~/catkin_mine_deploy
catkin_make
source devel/setup.bash
```

**编译产物**:
- `devel/lib/mine_map_deploy/global_map_listener`
- `devel/lib/mine_map_deploy/image_publisher`

---

## 运行

### 前提: FAST-LIO 已在运行

确保 `laserMapping` 节点已启动并发布 `/cloud_registered`：
```bash
roslaunch fast_lio mapping_avia.launch   # 或你的 FAST-LIO launch 文件
```

### 终端 1: 启动全局点云累积 (必须最先启动)
```bash
source ~/catkin_mine_deploy/devel/setup.bash
roslaunch mine_map_deploy global_map_listener.launch
```

### 终端 2: 启动栅格图生成节点
```bash
source ~/catkin_mine_deploy/devel/setup.bash
roslaunch mine_map_deploy image_publisher.launch
```

### 终端 3: 启动 HTTP 地图服务
```bash
source ~/catkin_mine_deploy/devel/setup.bash
# 需要可执行权限
chmod +x ~/catkin_mine_deploy/src/mine_map_deploy/scripts/grid_http_server.py
rosrun mine_map_deploy grid_http_server.py
```

---

## 手柄端/客户端调用示例

| 接口 | 方法 | 地址 | 说明 |
|------|------|------|------|
| 获取地图图片 | GET | `http://<机器人IP>:8765/map.png` | 返回最新 PNG 栅格地图 |
| 触发地图重生成 | GET | `http://<机器人IP>:8765/trigger` | 调用 `/generate_grid` 重新生成 |
| 获取检测标记 | GET | `http://<机器人IP>:8765/markers.json` | 返回 YOLO 检测的标记点 JSON |
| 回传选中点 | POST | `http://<机器人IP>:8765/select` | 手柄端选中点回传机器人 |

```bash
# 手柄端获取地图图片
curl http://192.168.1.100:8765/map.png -o map.png

# 触发地图重生成 (运动后需刷新)
curl http://192.168.1.100:8765/trigger

# 获取 YOLO 检测标记
curl http://192.168.1.100:8765/markers.json

# 回传选中点
curl -X POST http://192.168.1.100:8765/select \
  -H "Content-Type: application/json" \
  -d '{"points":[{"wx":1.5,"wy":3.2,"label":"defect"}]}'
```

---

## 典型工作流

```bash
# ==== 机器人端 ====

# 1. 启动激光雷达驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# 2. 启动 FAST-LIO 建图
roslaunch fast_lio mapping_mid360.launch

# 3. 启动 mine_map 三节点
roslaunch mine_map_deploy global_map_listener.launch &
roslaunch mine_map_deploy image_publisher.launch &
rosrun mine_map_deploy grid_http_server.py &

# ==== 手柄端 ====

# 4. 浏览器打开 http://<机器人IP>:8765/map.png 查看实时地图
# 5. 每次机器人移动后 GET /trigger 刷新栅格图
# 6. GET /markers.json 查看 YOLO 检测结果
```

---

## 依赖

### C++ (rosdep)
```
roscpp  sensor_msgs  nav_msgs  std_srvs
pcl_conversions  pcl_ros  cv_bridge  image_transport
OpenCV  PCL (>=1.8)
```

### Python 3
```
rospy  numpy  Pillow (PIL)  tf
```

### 安装缺失依赖
```bash
pip3 install numpy Pillow
sudo apt install ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport
```

---
# Calon的补充
## ROS 控制节点架构

本方案将原始单节点串口控制拆分为三个 ROS 节点，实现**控制源解耦**与**IMU 闭环**，支持通过 `twist_mux` 接入多路控制源（手柄、导航、急停等）。


---

### 1. `udp_control_bridge`

**功能**：接收 UDP 手柄数据，解析为 5 字节控制帧，发布到 ROS 话题。

| 项目 | 说明 |
|------|------|
| **输入** | UDP 端口 `7950` @ `192.168.166.49` |
| **输出** | `std_msgs/UInt8MultiArray` → `/robot_control_bytes` |
| **频率** | 100 Hz |
| **协议** | 5 字节 `[0xff, taitou, shifang, qianjin, zuozhuan]`，映射逻辑与原代码保持一致 |
| **超时保护** | 0.2 s 未收到 UDP 数据则自动发布停车帧 `[0xff, 0x0a, 0x0a, 0x0a, 0x0a]` |

**与原始代码的差异**：
- 移除直接串口写入，改为 ROS 话题发布。
- 保留全部原始映射算法（抬头、释放、前进、转向）。
- 串口初始化代码保留但注释，可随时回退。

---

### 2. `control_to_cmdvel_node`

**功能**：将 `/robot_control_bytes` 解析为 `geometry_msgs/Twist`，供 `twist_mux` 使用。

| 项目 | 说明 |
|------|------|
| **输入** | `/robot_control_bytes` (5 字节) |
| **输出** | `/cmd_vel` (`geometry_msgs/Twist`) |
| **映射关系** | `linear.x` ← 第 4 字节 (`qianjin`)<br>`angular.z` ← 第 5 字节 (`zuozhuan`) |
| **标定参数** | `scale_linear` (m/s 每字节偏移)<br>`scale_angular` (rad/s 每字节偏移)<br>`byte_mid` (中位值，默认 `0x0a`) |

**适用场景**：手柄遥控时，将底层字节协议转换为标准 ROS 速度指令，接入导航/避障/急停等多路控制源的仲裁层。

---

### 3. `cmdvel_to_serial_node`

**功能**：订阅 `twist_mux` 输出的 `/cmd_vel` 与 IMU 角速度，执行 **PID 角速度闭环 + 绳长保护**，最终通过串口下发 5 字节控制帧。

| 项目 | 说明 |
|------|------|
| **输入** | `/cmd_vel` (`geometry_msgs/Twist`)<br>`/imu/data` (`sensor_msgs/Imu`，取 `angular_velocity.z`) |
| **输出** | `/dev/ttyUSB0` @ 115200，5 字节协议帧 |
| **控制频率** | 100 Hz |

#### 3.1 角速度闭环（IMU PID）

- **期望**：`cmd_vel.angular.z`
- **反馈**：IMU 实测 `yaw rate`
- **控制器**：位置式离散 PID
  - `u = Kp·e + Ki·∫e·dt + Kd·de/dt`
  - 积分限幅（Anti-Windup）：`integral_ ∈ [-integral_max, integral_max]`
- **前馈 + 反馈**：转向字节以开环映射为前馈，PID 输出作为补偿量叠加。

#### 3.2 绳长保护（软件限幅）

由于转向舵机为 **360° 连续旋转电机**（非论文中的位置舵机），绳长会随转向指令持续累积。节点内部维护估算绳长差 `rope_delta`：

- 每周期积分：`rope_delta += K_rope · (steering_byte - 0x0a) · dt`
- 限幅：`|rope_delta| ≤ rope_max_mm`（默认 36 mm，对应论文单侧极限）
- 饱和策略：达到限幅时，禁止同向转向指令，强制回中 `0x0a`，直到收到反向指令。

#### 3.3 上电回中

节点启动后，连续发送 1 秒中位帧 `[0xff, 0x0a, 0x0a, 0x0a, 0x0a]`，使舵机停转、棘轮机构自然回中，消除上电时绳子的初始张紧误差。

#### 3.4 线速度控制

驱动舵机（履带）采用 **开环映射**：`cmd_vel.linear.x` 直接按比例转换为字节偏移，经 `[0, 255]` 限幅后输出。

---

### 参数配置（YAML）

所有标定参数集中存放，节点通过私有命名空间 `~` 读取：

```yaml
# control_to_cmdvel_node
control_to_cmdvel_node:
  scale_linear: 0.05      # m/s per byte offset
  scale_angular: 0.25     # rad/s per byte offset
  byte_mid: 10            # 0x0a

# cmdvel_to_serial_node
cmdvel_to_serial_node:
  serial_port: "/dev/ttyUSB0"
  serial_baudrate: 115200
  byte_mid: 10

  # 速度映射（与 control_to_cmdvel_node 互为倒数）
  scale_linear_inv: 20.0
  scale_angular_inv: 4.0

  # PID 角速度闭环
  pid_kp: 2.0
  pid_ki: 0.1
  pid_kd: 0.0
  pid_integral_max: 5.0

  # 绳长保护
  rope_max_mm: 36.0       # 论文单侧极限 ~36 mm
  rope_k_scale: 0.05      # mm/byte/loop，需根据舵机转速标定
  loop_rate_hz: 100.0

