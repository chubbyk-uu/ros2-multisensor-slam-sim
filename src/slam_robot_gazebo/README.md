# slam_robot_gazebo

本包提供 Gazebo Sim 世界、差速驱动系统和 ROS-Gazebo 桥接配置。

启动：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py
```

无界面启动：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py gui:=false
```

当前已桥接 `/clock`、`/cmd_vel`、`/odom`、`/ground_truth/odom`、`/joint_states`、`/scan` 和 `/tf`。
默认加载 `worlds/slam_world.sdf`，其中包含外围墙、非对称隔墙、箱体、圆柱路标和回环通道。
2D LiDAR 通过 `/scan` 发布 720 点、10 Hz 的 360° `sensor_msgs/LaserScan`，坐标系为 `lidar_link`。

`worlds/degenerate_corridor.sdf` 是自研 SLAM 的受控退化场景：33 m 长、
2.6 m 净宽的平行墙走廊，中段在 LiDAR 量程内没有纵向几何特征，入口和
出口各保留一个避开中心线的非对称锚点。可通过 bringup 包的
`corridor_slam_regression.launch.py` 启动。

`worlds/repeated_rooms.sdf` 包含两间相距 8 m、主要墙体和四个大型路标
完全重复的房间，并在每间房各保留一个微弱的独有特征。中心通道供自动
往返路线使用；该场景通过 `repeated_structure_slam_regression.launch.py`
验证相似地点不会触发假回环，同时真正返回后仍能建立闭环。

`/ground_truth/odom` 由 Gazebo OdometryPublisher 根据世界位姿生成，父坐标系为 `world`，子坐标系为 `base_footprint`。它只用于算法误差评估，不能作为 SLAM 或导航输入。

默认会打开 RViz；仅运行 Gazebo 时可传入 `rviz:=false`。

在 WSL2 中 launch 默认启用 Mesa D3D12 并选择 NVIDIA 显卡，避免回退到 CPU 软件渲染。可使用 `wsl_gpu_adapter:=AMD` 或 `wsl_gpu_adapter:=Intel` 选择其他适配器，也可使用 `use_wsl_gpu:=false` 禁用该设置。

当前性能配置：

- 物理循环为 250 Hz，仿真实时率目标为 1.0。
- Gazebo 阴影和激光射线可视化关闭；`/scan` 数据不受影响。
- RViz 刷新率为 20 Hz，TF 显示默认关闭。
- 当前 Jazzy 随附的 JointStatePublisher 可能忽略 `<update_rate>`，因此 `/joint_states` 会跟随物理循环发布；这不影响 SLAM。
