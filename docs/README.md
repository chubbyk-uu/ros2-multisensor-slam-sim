# 文档索引

本文档集按“怎么使用、系统是什么、如何证明、为何出错、历史记录”组织。当前能力和
项目范围请先看[项目状态](status.md)。

## 使用项目

| 文档 | 内容 |
| --- | --- |
| [安装与首次运行](getting-started.md) | 依赖、构建、WSL2、模型与基础仿真检查 |
| [2D 工作流](2d-workflows.md) | 2D 建图、保存、AMCL、Nav2、自研 2D SLAM 与回归 |
| [3D 工作流](3d-workflows.md) | RTAB-Map、自研 3D、快照、在线导航、自主探索与 MOLA |
| [常见问题](troubleshooting.md) | 操作性排障：Gazebo、RViz、地图、扫描与导航 |

## 理解系统

| 文档 | 内容 |
| --- | --- |
| [系统架构](architecture.md) | 包职责、数据流、传感器、TF 与设计边界 |
| [项目路线图](../plan.md) | 当前目标、完成阶段与未完成工作 |
| [固定数据集](datasets.md) | rosbag 输入契约、指纹与离线复现 |

## 证据与工程决策

| 文档 | 内容 |
| --- | --- |
| [性能与标定](performance.md) | 构建基线、耗时、实时性、IMU 与轮距标定 |
| [验收记录](acceptance.md) | 每条链路的门限、结果 JSON 与通过依据 |
| [验证方法](methodology.md) | 固定包、A/B、随机化、campaign 与故障注入方法 |
| [故障归因与工程事件](incidents.md) | 根因、修复、验证与已知外部边界 |

## 历史档案

[历史档案](archive/README.md)保留逐日性能/验收原始记录、已完成的任务清单和 2D
工程审查整改全文。归档内容仍可作为证据引用，但不再是当前使用入口。

## 包级接口

- [机器人模型](../src/slam_robot_description/README.md)
- [Gazebo 仿真](../src/slam_robot_gazebo/README.md)
- [统一启动入口](../src/slam_robot_bringup/README.md)
- [自研 2D SLAM](../src/slam_robot_slam/README.md)
- [3D SLAM](../src/slam_robot_slam_3d/README.md)
- [导航](../src/slam_robot_navigation/README.md)
