# ROS 2 多传感器 SLAM 仿真

基于 ROS 2 Jazzy 与 Gazebo Sim 的差速轮式机器人 SLAM 仿真项目。项目按“成熟算法
基线 → 自动回归 → 自研模块替换”的路线，已完成 2D LiDAR 建图与导航、自研 2D
SLAM、轮速 + IMU EKF、RTAB-Map 3D 基线，以及自研 3D SLAM、在线导航、快照恢复和
Frontier Exploration。

![Gazebo 中的差速机器人和室内 SLAM 测试环境](docs/images/gazebo-simulation.png)

![RViz 中的地图、激光、代价地图和导航状态](docs/images/nav2-navigation.png)

## 安装

环境要求为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic（Gazebo Sim 8），也支持
装有对应图形驱动的 WSL2。

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

3D 基线另需 RTAB-Map 与 MOLA 包。完整依赖列表、WSL2 说明和首次仿真检查见
[安装与首次运行](docs/getting-started.md)。

## 快速开始

构建完成后，按[安装与首次运行](docs/getting-started.md)做一次基础仿真检查，然后
选择一条入口运行：

```bash
# 2D 建图；另开终端用 teleop_twist_keyboard 驾驶
ros2 launch slam_robot_bringup mapping_simulation.launch.py

# RTAB-Map 3D 在线建图
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py

# 自研 3D SLAM + Nav2 自主探索；完成后自动保存快照
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py
```

2D 建图、保存地图、AMCL 与 Nav2 见[2D 工作流](docs/2d-workflows.md)；RTAB-Map、
自研 3D、快照、随机出生和探索见[3D 工作流](docs/3d-workflows.md)。

## 当前状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 差速轮式机器人与 Gazebo 世界 | 已完成 | Xacro、惯性、碰撞体、IMU 与互斥 2D/3D LiDAR 变体；室内、长走廊、重复结构、大场景和结构化 3D 世界 |
| 2D 官方基线 | 已完成并冻结 | SLAM Toolbox、地图自动保存、AMCL、Nav2 |
| 自研 C++ 2D SLAM | 已完成基线 | 相关匹配、退化检测、关键帧、后台回环、Ceres 位姿图和地图重建 |
| 轮速 + IMU EKF | 已完成并默认启用 | `robot_localization` 融合平面轮速与 IMU 偏航角速度 |
| RTAB-Map 3D 基线 | 已完成 | ICP、proximity 回环、位姿图与二维导航投影；Nav2 直接使用其地图和 3D 障碍层 |
| 自研 3D SLAM | 已完成首版闭环 | GICP 前端、Scan Context 回环、后台 SE(2) 位姿图、全局地图与版本化快照恢复 |
| Frontier Exploration | 已完成双链路验收 | 自研与 RTAB-Map 共用候选评分、Nav2 调度、自动快照与安全随机出生 |
| 视觉融合 | 计划中 | 后续接入相机，研究 3D LiDAR + 相机 + IMU |

最近独立验证结果和下一项工作见[项目状态](docs/status.md)。完整系统数据流、包边界
和 TF 拓扑以[系统架构](docs/architecture.md)为准。

项目约束：2D 与 3D LiDAR 模型互斥；同一时间仅一条链路发布 `map -> odom`；
`/ground_truth/odom` 仅用于测试与评分，绝不进入 SLAM、EKF 或导航估计。

## 如何验证

本项目不以“能跑起来”作为结论。每条链路都有固定输入回归、门限和版本库内的结果
JSON：

```bash
colcon test && colcon test-result --all   # 共 777 项，含 gtest、pytest 与静态检查
```

- **固定包回归**：`bags/structured_3d_reference` 等固定 MCAP 排除传感器随机性，
  用于比较扫描匹配、回环、地图重建和快照恢复，输入契约与哈希见
  [固定数据集](docs/datasets.md)。
- **活动 campaign**：重复运行报告分布而非单次末值，并区分核心 FAIL、
  `INFRA_UNSTABLE` 与 `VOID`。
- **故障注入**：Nav2 启动期未 activate 与运行期失活均须提前退出并归类为
  `INFRA_UNSTABLE`，不得伪装成探索完成。
- **持久化**：快照以写后回读的 FNV-1a 校验和 + `fsync` 确认落盘，原子替换本身
  不算持久化。

结论、门限与结果 JSON 见[验收记录](docs/acceptance.md)，实验设计原则见
[验证方法](docs/methodology.md)。

## 文档

按任务查找文档、验收证据、故障归因、方法说明和历史档案，请从
[文档索引](docs/README.md)进入。

## License

本项目采用 [Apache License 2.0](LICENSE)。
