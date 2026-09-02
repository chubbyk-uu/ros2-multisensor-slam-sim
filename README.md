# ROS 2 多传感器 SLAM 仿真

基于 ROS 2 Jazzy 与 Gazebo Sim 的差速轮式机器人 SLAM 仿真项目。项目按“成熟算法
基线 → 自动回归 → 自研模块替换”的路线，已完成 2D LiDAR 建图与导航、自研 2D
SLAM、轮速 + IMU EKF、RTAB-Map 3D 基线，以及自研 3D SLAM、在线导航、快照恢复和
Frontier Exploration。

![Gazebo 中的差速机器人和室内 SLAM 测试环境](docs/images/gazebo-simulation.png)

![RViz 中的地图、激光、代价地图和导航状态](docs/images/nav2-navigation.png)

## 快速开始

先按[安装与首次运行](docs/getting-started.md)完成依赖、构建、`source` 和基础仿真
检查。随后选择一条入口运行：

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

已完成能力、最近独立验证结果和下一项工作见[项目状态](docs/status.md)。完整系统
数据流、包边界和 TF 拓扑以[系统架构](docs/architecture.md)为准。

项目约束：2D 与 3D LiDAR 模型互斥；同一时间仅一条链路发布 `map -> odom`；
`/ground_truth/odom` 仅用于测试与评分，绝不进入 SLAM、EKF 或导航估计。

## 文档

按任务查找文档、验收证据、故障归因、方法说明和历史档案，请从
[文档索引](docs/README.md)进入。

## License

本项目采用 [Apache License 2.0](LICENSE)。
