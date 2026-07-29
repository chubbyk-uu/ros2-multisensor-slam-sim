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

当前已桥接 `/clock`、`/cmd_vel`、`/odom`、`/joint_states`、`/scan` 和 `/tf`。
默认加载 `worlds/slam_world.sdf`，其中包含外围墙、非对称隔墙、箱体、圆柱路标和回环通道。
2D LiDAR 通过 `/scan` 发布 720 点、10 Hz 的 360° `sensor_msgs/LaserScan`，坐标系为 `lidar_link`。

默认会打开 RViz；仅运行 Gazebo 时可传入 `rviz:=false`。

在 WSL2 中 launch 默认启用 Mesa D3D12 并选择 NVIDIA 显卡，避免回退到 CPU 软件渲染。可使用 `wsl_gpu_adapter:=AMD` 或 `wsl_gpu_adapter:=Intel` 选择其他适配器，也可使用 `use_wsl_gpu:=false` 禁用该设置。

当前性能配置：

- 物理循环为 250 Hz，仿真实时率目标为 1.0。
- Gazebo 阴影和激光射线可视化关闭；`/scan` 数据不受影响。
- RViz 刷新率为 20 Hz，TF 显示默认关闭。
- 当前 Jazzy 随附的 JointStatePublisher 可能忽略 `<update_rate>`，因此 `/joint_states` 会跟随物理循环发布；这不影响 SLAM。
