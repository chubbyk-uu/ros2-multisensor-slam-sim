# 3D 建图、导航与验收

当前在线 3D 基线使用 RTAB-Map；MOLA GICP 作为不伪造逐点时间字段的纯激光
里程计对照。两者使用相同的 3D LiDAR 机器人变体，但不能同时发布
`map -> odom`。

## RTAB-Map 在线 3D SLAM

启动 3D 机器人、轮速 + IMU EKF、点云检查、RTAB-Map 和 RViz：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py
```

另开终端驾驶：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

RViz 的 `RTAB-Map Cloud` 显示累计三维地图；`Current 3D Scan` 显示实时
`/lidar_3d/points`。RTAB-Map 同时发布 `/rtabmap/map` 二维占据栅格，供
Nav2 使用。

默认 `reset_database:=true`，每次启动删除旧数据库并从新地图开始。正常退出
后数据库保存在 `~/.ros/rtabmap_3d.db`。继续已有地图时才使用：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  reset_database:=false
```

无界面运行：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  gui:=false rviz:=false
```

这是一台平面差速机器人，因此 RTAB-Map 强制轨迹为 `x/y/yaw` 三自由度。
该约束只作用于轨迹和 `map -> odom`，累计点云仍为三维；不能关闭后继续将
同一地图用于 Nav2，否则微小横滚/俯仰误差会把远处地面误标成障碍。

当前 Gazebo 点云没有逐点时间字段，不能做严格 deskew。因此该链路是低速
几何 ICP + 回环 + 位姿图基线，不是完整 LIO。详细参数与 QoS 约束见
[3D SLAM 包说明](../src/slam_robot_slam_3d/README.md)。

## RTAB-Map + Nav2 在线导航

一条命令启动在线建图和导航：

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py
```

该入口不启动 Map Server 或 AMCL。RTAB-Map 发布 `map -> odom` 和实时二维
投影地图；Nav2 使用该地图做全局规划，并直接把 3D 点云送入局部体素障碍层。
RViz 同时显示累计 3D 地图、实时点云、二维栅格、代价地图和路径。

障碍物安全扫掠高度为 `0.05–0.45 m`：机器人含雷达总高 `0.35 m`，另留
`0.10 m` 余量。这个二维导航模型会绕开安全高度内的最大障碍外轮廓，同时
允许通过净空高于 `0.45 m` 的门洞。

需要单独调试 Nav2 时，先确保 RTAB-Map 已运行，再启动：

```bash
ros2 launch slam_robot_navigation online_slam_navigation.launch.py
```

地图形成后可检查二维栅格契约：

```bash
ros2 run slam_robot_slam_3d grid_contract_check
```

## 结构化世界与自动验收

手动在 3D 结构化世界中建图和导航：

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py \
  world:=$(ros2 pkg prefix slam_robot_gazebo)/share/slam_robot_gazebo/worlds/structured_loop_3d.sdf
```

世界包含立体墙面、斜柱、高门洞和下细上粗柱。Gazebo 中可以拖动视角；RViz
使用可旋转 Orbit 视角。

两圈闭环建图回归：

```bash
ros2 launch slam_robot_slam_3d structured_loop_regression.launch.py
```

建图后继续执行 Nav2 高度语义验收：

```bash
ros2 launch slam_robot_slam_3d structured_navigation_regression.launch.py
```

完整验收要求：

- 形成真实 proximity 回环和非零全局修正。
- `map -> odom` 高度峰值不超过 `0.02 m`，横滚/俯仰不超过 `0.5°`。
- 机器人通过净空 `0.55 m` 的门洞。
- `0.07 m` 细底座上方的 `0.40 m` 柱帽按最大外轮廓进入代价地图。
- 真实轨迹与柱心保持至少 `0.55 m`，并产生至少 `0.25 m` 绕行。

自动回归默认无 Gazebo 和 RViz 窗口，这是为了减少资源占用；人工观察时使用
上面的手动启动命令。测量结果见 [性能与标定](performance.md)。

## MOLA 纯激光里程计对照

```bash
ros2 launch slam_robot_slam_3d mola_lo_simulation.launch.py
```

该入口关闭 deskew，不生成伪造的逐点时间，也不使用真值、轮式里程计或 IMU
作为位姿输入。`use_imu_gravity:=true` 仅提供重力方向先验，不等同于完整
LIO。MOLA 输出局部里程计和局部点云，不提供与 RTAB-Map 等价的完整在线全局
SLAM 能力。

## 后续路线

RTAB-Map 是成熟算法验收基线，后续逐步以自研 3D 前端、局部子图、回环后端
和地图输出模块替换，同时保留相同的输入契约、TF 职责和回归场景。自研链路
完成后将接入现有 Nav2 在线入口：自研 SLAM 发布实时二维导航投影并独占
`map -> odom`，Nav2 继续使用 3D 点云局部避障。

最终阶段增加 Frontier Exploration。探索器只负责从实时占据栅格中提取、
筛选和排序未知边界目标，再通过 Nav2 标准 `NavigateToPose` 动作导航；它不
替代 Nav2 的规划控制，也不直接发布 `/cmd_vel`。自动验收从未知地图开始，
检查覆盖率、不可达目标处理、回环修正后的重规划、碰撞和地图自动保存。

相机随后用于增强地点识别和全局重定位，IMU 用于支持具备逐点时间数据的
LIO；不计划融合 2D 与 3D LiDAR。详细分阶段任务和验收标准见项目根目录
`plan.md` 的 8.3、8.4 节。
