# 项目协作规范

## 项目目标

本项目基于 ROS 2 Jazzy 和 Gazebo Sim，构建轮式机器人 SLAM 仿真系统。
当前优先完成 2D 激光 SLAM，后续再扩展 3D 激光、视觉 SLAM 和多传感器融合。

## 开发原则

- 按“机器人模型、Gazebo 仿真、传感器、SLAM、导航”的顺序渐进开发。
- 每次只引入一个可验证的功能，先通过测试再进入下一阶段。
- 优先保证坐标系、时间戳、里程计和传感器数据正确，再调整 SLAM 参数。
- 通用参数应放入 YAML、Xacro 或 launch 参数中，避免散落在代码里。
- 保持各 ROS 2 包职责单一，不把模型、仿真、算法和导航配置混在一起。

## ROS 2 约定

- 使用 Gazebo Sim 和 `ros_gz`，不使用 Gazebo Classic 接口。
- 仿真节点统一启用 `use_sim_time`。
- 默认使用标准话题名：`/cmd_vel`、`/odom`、`/scan`、`/map`、`/tf` 和 `/tf_static`。
- 默认 TF 树为：

  ```text
  map -> odom -> base_footprint -> base_link -> lidar_mount_link -> lidar_link
  ```

- `map -> odom` 由 SLAM 或定位节点发布。
- `odom -> base_footprint` 由里程计发布。
- 机器人固定结构和关节变换由 `robot_state_publisher` 发布。
- 同一段 TF 只能有一个发布者。
- `base_footprint` 位于驱动轮轴线中点的地面投影，作为差速运动学旋转中心。
- 底盘几何中心和质心可以偏离 `base_footprint`，但质心投影必须位于支撑区域内。

## 文件与代码规范

- 正式 ROS 2 节点和 SLAM 核心算法使用 C++17。
- Python 主要用于 launch、数据分析、测试工具和算法原型。
- 机器人模型使用 Xacro，尺寸、质量和传感器位置应参数化。
- Launch 文件负责组合节点，YAML 文件负责可调参数。
- 不直接修改 `build/`、`install/` 和 `log/` 中的生成文件。
- 新增节点、话题、坐标系或启动方式时，同步更新项目文档。
- 不删除或覆盖已有用户文件；修改前先检查当前工作区状态。

## 验证要求

- 修改机器人模型后，检查 Xacro 展开、TF 树和 RViz 显示。
- 修改仿真后，检查生成、运动、碰撞和仿真实时率。
- 修改传感器后，检查话题频率、`frame_id`、时间戳和 RViz 数据对齐。
- 修改 SLAM 后，检查地图重影、漂移、回环和地图保存。
- 完成阶段性功能后，记录启动命令、测试方法和已知问题。
