# ROS 2 Multi-Sensor SLAM Simulation

基于 ROS 2 Jazzy 和 Gazebo Sim 的差速轮式机器人多传感器 SLAM 仿真项目。
项目按照“成熟算法基线 → 自动回归 → 自研模块替换”的路线推进，已完成
2D LiDAR 建图与导航、自研 C++ 2D SLAM、轮速 + IMU 二维 EKF、RTAB-Map
3D LiDAR 成熟基线，以及自研 3D SLAM、在线导航和 Frontier Exploration
完整闭环。

## 运行效果

### Gazebo 仿真环境

![Gazebo 中的差速机器人和室内 SLAM 测试环境](docs/images/gazebo-simulation.png)

### AMCL 定位与 Nav2 导航

![RViz 中的地图、激光、代价地图和导航状态](docs/images/nav2-navigation.png)

## 当前状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 差速轮式机器人 | 已完成 | Xacro、惯性、碰撞体、驱动轮、万向轮、IMU 和互斥 LiDAR 变体 |
| Gazebo Sim | 已完成 | 室内、长走廊、重复结构、大场景和结构化 3D 世界 |
| 2D 官方基线 | 已完成并冻结 | SLAM Toolbox、地图自动保存、AMCL、Nav2 |
| 自研 C++ 2D SLAM | 已完成基线 | 相关匹配、退化检测、关键帧、回环、Ceres 位姿图和地图重建 |
| 2D 自动回归 | 已完成 | 闭环、快速旋转、退化走廊、重复结构、155 m 大场景和固定 rosbag |
| 轮速 + IMU EKF | 已完成并默认启用 | `robot_localization` 融合平面轮速和 IMU 偏航角速度 |
| 3D LiDAR 模型与接口 | 已完成 | 点云、TF、QoS、RViz 和输入契约 |
| RTAB-Map 3D SLAM | 已完成成熟基线 | ICP、proximity 回环、位姿图、数据库和二维导航投影 |
| 3D 在线导航 | 已完成基线验收 | Nav2 直接使用 RTAB-Map 地图和 3D 点云障碍层 |
| 自研 3D SLAM | 已完成首版闭环 | GICP 前端、回环后端、全局地图、Nav2 在线入口和版本化快照恢复 |
| Frontier Exploration | 已完成双链路验收 | 自研 3D 与 RTAB-Map 共用候选评分、Nav2 调度、重规划、自动保存和回归 |
| 视觉融合 | 计划中 | 后续接入相机，研究 3D LiDAR + 相机 + IMU |

## 快速安装

环境要求为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。完整依赖、WSL2
说明和首次检查见 [安装与首次运行](docs/getting-started.md)。

```bash
git clone https://github.com/chubbyk-uu/ros2-multisensor-slam-sim.git
cd ros2-multisensor-slam-sim

export SLAM_WS="$PWD"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

3D 基线还需要：

```bash
sudo apt install \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-mola-lidar-odometry \
  ros-jazzy-mola-bridge-ros2 \
  ros-jazzy-mola-metric-maps
```

## 快速开始：2D

### 建图

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

另开终端遥控：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

建图结束后在 launch 终端按一次 `Ctrl+C`。默认会先保存：

- `maps/slam_map.yaml`、`maps/slam_map.pgm`
- `maps/slam_map.posegraph`、`maps/slam_map.data`

`maps/` 是每次运行的输出目录，其中的地图不进入版本控制。

### 定位与导航

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

在 RViz 使用 `Nav2 Goal` 设置目标。导航前退出键盘遥控，避免多个速度发布者。

想跳过建图直接看导航效果，可以使用随仓库分发的演示地图：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  map:="${SLAM_WS}/maps/reference/slam_map.yaml"
```

### 自研 2D SLAM

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

详细的地图保存、AMCL 初始化、导航回归、自研算法输出和场景测试见
[2D 工作流](docs/2d-workflows.md)。

## 快速开始：3D

### RTAB-Map 在线建图

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py
```

默认删除旧数据库并从新地图开始。正常退出后数据库保存在
`~/.ros/rtabmap_3d.db`；继续已有地图时传入 `reset_database:=false`。

### RTAB-Map + Nav2 在线导航

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py
```

该入口不需要预先保存二维地图，也不启动 AMCL。RTAB-Map 发布实时三维地图、
二维导航投影和 `map -> odom`；Nav2 全局规划使用二维投影，局部障碍层直接
使用 `/lidar_3d/points`。

### 结构化自动验收

```bash
ros2 launch slam_robot_slam_3d structured_navigation_regression.launch.py
```

回归先完成两圈 3D 建图，再验证高门洞可通过、下细上粗柱按最大外轮廓绕行，
以及 `map -> odom` 始终满足平面机器人约束。自动回归默认不打开 Gazebo 和
RViz 窗口。

### 自研 3D SLAM + Nav2

```bash
ros2 launch slam_robot_slam_3d custom_3d_navigation_simulation.launch.py
```

该入口边建图边导航，并在正常退出时原子保存自包含快照到
`~/.ros/custom_slam_3d.snapshot`。继续建图或从保存末端位姿只读定位的命令见
[3D 工作流](docs/3d-workflows.md)。当前版本使用 0.05/0.10 m 双分辨率点云与 v4
快照；快照会校验占据投影的体素、高度、距离和平面运动契约，旧 v1/v2 快照或
契约不一致的数据需重新建图。首版只读定位尚不支持从任意初始位姿全局重定位。

### 3D 自主探索

```bash
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py
```

该入口从未知地图开始，探索器只向 Nav2 发送标准导航目标，不发布速度；完成后
自动保存 `~/.ros/custom_slam_3d.snapshot`。需要从 SDF 中自动选择随机且安全的
初始位姿时使用：

```bash
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py \
  random_spawn:=true spawn_seed:=42
```

采样器读取实际 collision 几何，只在原点所在的可通行连通区取样；安全包络包含
底盘平面半径、0.15 m 余量、含 3D 雷达的 0.35 m 整机高度、0.10 m 顶部余量及
Gazebo 生成时的 0.03 m 抬升。终端和 RViz 会在完成及快照保存成功时明确提示，
此后再按 `Ctrl+C`。使用 RTAB-Map 成熟基线运行同一探索策略：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_exploration_simulation.launch.py
```

无界面端到端回归使用相同世界和评分器：

```bash
ros2 launch slam_robot_slam_3d frontier_exploration_regression.launch.py
ros2 launch slam_robot_slam_3d rtabmap_frontier_exploration_regression.launch.py
```

自研探索回归检查覆盖率、地图宽高/面积、目标与恢复预算、子图重启和自动快照；
固定包双圈回环召回使用 `custom_3d_loop_regression.launch.py` 单独验收；
存档恢复使用 `snapshot_resume_regression`，在固定包上分两段验证“存档 → 重启 →
继续建图”。

3D 地图显示、数据库复用、结构化世界、MOLA 对照和完整验收标准见
[3D 工作流](docs/3d-workflows.md)。

## 系统约束

- 2D 与 3D LiDAR 使用互斥机器人模型，不同时生成。
- 默认局部里程计使用轮速 + IMU EKF；可通过 `odometry_mode:=wheel` 回退
  纯轮式模式。
- `map -> odom` 只能由当前 SLAM 或定位节点中的一个发布。
- `/ground_truth/odom` 仅用于测试驾驶和评分，禁止进入估计链路。
- 当前 Gazebo 3D 点云没有逐点时间字段，因此 RTAB-Map 和 MOLA 是低速几何
  基线，不是完整 LIO。
- 3D 平面机器人强制 RTAB-Map 轨迹为 `x/y/yaw`，但输入和累计地图仍为三维。

主要 TF：

```text
map -> odom -> base_footprint -> base_link
                                   ├── imu_link
                                   └── 2D 或 3D lidar_link
```

完整话题、数据流、TF 发布职责和包边界见
[系统架构](docs/architecture.md)。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [安装与首次运行](docs/getting-started.md) | 依赖、构建、模型与基础仿真检查 |
| [2D 工作流](docs/2d-workflows.md) | 建图、保存、AMCL、Nav2、自研 SLAM 和回归 |
| [3D 工作流](docs/3d-workflows.md) | 自研 3D、RTAB-Map、在线导航、自主探索、结构化验收和 MOLA |
| [系统架构](docs/architecture.md) | 包职责、数据流、话题和 TF |
| [常见问题](docs/troubleshooting.md) | Gazebo、RViz、地图重置、扫描错位和假障碍 |
| [开发计划](plan.md) | 阶段目标、完成状态和后续路线 |
| [性能与标定](docs/performance.md) | CPU、实时性、精度、回归和轮距标定记录 |
| [固定数据集](docs/datasets.md) | rosbag 约束、指纹和离线复现 |
| [审查整改](docs/review_remediation.md) | 自研 2D SLAM 工程审查处理记录 |

包级接口文档：

- [机器人模型](src/slam_robot_description/README.md)
- [Gazebo 仿真](src/slam_robot_gazebo/README.md)
- [统一启动入口](src/slam_robot_bringup/README.md)
- [自研 2D SLAM](src/slam_robot_slam/README.md)
- [3D SLAM](src/slam_robot_slam_3d/README.md)
- [导航](src/slam_robot_navigation/README.md)

## License

本项目采用 [Apache License 2.0](LICENSE)。
