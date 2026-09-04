# ROS 2 多传感器 SLAM 仿真

基于 ROS 2 Jazzy 与 Gazebo Sim 的差速轮式机器人 SLAM 仿真项目。项目按“成熟算法
基线 → 自动回归 → 自研模块替换”的路线，已完成 2D LiDAR 建图与导航、自研 2D
SLAM、轮速 + IMU EKF、RTAB-Map 3D 基线，以及自研 3D SLAM、在线导航、快照恢复和
Frontier Exploration；最终还提供 RTAB-Map RGB-D 建图/定位与 3D LiDAR 避障的
松耦合导航入口。

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

# RTAB-Map RGB-D 在线基线（轮速 + IMU 提供局部里程计）
ros2 launch slam_robot_slam_3d rtabmap_rgbd_simulation.launch.py

# RTAB-Map RGB-D 建图 + 3D LiDAR 避障，可在 RViz 点击目标
ros2 launch slam_robot_slam_3d rtabmap_rgbd_navigation_simulation.launch.py

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
| RGB-D 基础设施 | 已完成 | 前向模型、图像/深度/内参桥接、独立大消息 DDS 保护、RViz 与数据契约回归 |
| RTAB-Map RGB-D 基线 | 已完成 | 官方 RGB-D 同步、方向性纹理、视觉回环、深度投影地图与独立大消息 DDS 保护；两圈固定回放与 30 Hz 在线活动验收均通过 |
| RGB-D 建图 + 3D LiDAR 避障 | 已完成并收口 | RTAB-Map 使用 RGB-D 建图/定位，Nav2 使用其二维地图规划；独立 3D LiDAR 为代价地图和碰撞监视器提供障碍数据 |

最近独立验证结果和项目范围见[项目状态](docs/status.md)。完整系统数据流、包边界
和 TF 拓扑以[系统架构](docs/architecture.md)为准。

项目约束：2D 与 3D LiDAR 模型互斥；同一时间仅一条链路发布 `map -> odom`；
`/ground_truth/odom` 仅用于测试与评分，绝不进入 SLAM、EKF 或导航估计。

RGB-D 导航入口采用明确的松耦合职责分工：RTAB-Map 只消费 RGB-D 与轮速 + IMU
里程计，负责建图、定位及 `map -> odom`；3D LiDAR 不进入 RTAB-Map，只进入 Nav2
障碍层和碰撞监视器。自研 3D LiDAR SLAM 仍作为另一条独立算法链路保留；当前项目
不再计划视觉辅助回环、视觉约束 LiDAR 位姿图、RGB-D 补充障碍或双目扩展。

## 如何验证

本项目不以“能跑起来”作为结论。每条链路都有固定输入回归、门限和版本库内的结果
JSON：

```bash
colcon test && colcon test-result --all   # gtest、pytest 与静态检查
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

## 后续可选改进

以下项目是 `v1.0.0` 已知且接受的能力边界，不影响当前验收结论；若继续维护，可按
实际使用需求逐项推进：

- [ ] 为自研 3D `localization` 增加任意初始位置的全局重定位；当前只支持从快照
  末端位姿附近恢复，详见[自研 3D 快照定位](src/slam_robot_slam_3d/README.md#从末端位姿继续建图)。
- [ ] 为自研 2D SLAM 序列化关键帧、约束和位姿图，使其能够恢复后继续建图；当前
  只保存供 AMCL/Nav2 使用的 YAML/PGM，详见[自研 2D 地图保存](docs/2d-workflows.md#保存自研地图并交给-nav2)。
- [ ] 为超出单次 LiDAR 视野的临时封路增加持久语义，例如 Nav2 keepout filter；
  当前障碍层可能在射线清除后遗忘视野外封口，详见[代价地图边界](docs/incidents.md#代价地图会忘记看不见的封路)。
- [ ] 继续定位 Gazebo 在全部 verdict 与快照完成后的偶发退出段错误；当前将其与运行期
  算法失败、清理失败和残留进程分别记录，详见[已知外部边界](docs/incidents.md#已知外部边界)。
- [ ] 将安全随机出生采样器扩展到真正的多层可行驶世界；当前仅支持单层共面支撑面，
  非共面支撑结构按障碍处理，详见[3D 探索工作流](docs/3d-workflows.md#自主-frontier-exploration)。

## License

本项目采用 [Apache License 2.0](LICENSE)。
