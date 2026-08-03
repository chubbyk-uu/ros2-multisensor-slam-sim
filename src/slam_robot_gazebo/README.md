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

当前已桥接 `/clock`、`/cmd_vel`、里程计、`/ground_truth/odom`、
`/joint_states`、LiDAR、`/imu/data_raw` 和所需 TF。
默认加载 `worlds/slam_world.sdf`，其中包含外围墙、非对称隔墙、箱体、圆柱路标和回环通道。
2D LiDAR 通过 `/scan` 发布 720 点、10 Hz 的 360° `sensor_msgs/LaserScan`，坐标系为 `lidar_link`。

3D LiDAR 通过独立入口启动：

```bash
ros2 launch slam_robot_gazebo lidar_3d_simulation.launch.py
```

它发布 10 Hz、720 × 16 的 `/lidar_3d/points`，坐标系为
`lidar_3d_link`。`sensor_variant:=2d|3d` 保证两套雷达互斥，因而 3D
配置没有 `/scan`，默认 2D 配置也不承担 3D 点云开销。两种配置均发布
100 Hz `/imu/data_raw`，噪声、启动偏置及频率可通过 launch 参数调整。

默认 `odometry_mode:=wheel` 由 Gazebo 唯一发布 `/odom` 和
`odom -> base_footprint`。可选 `odometry_mode:=wheel_imu` 将 Gazebo 的裸
轮速消息桥接为 `/wheel/odom_raw`，再由轻量适配节点补充有限、非零且可调
的协方差，发布 `/wheel/odom` 和 `/imu/data`。`robot_localization` 只融合
轮速平移和 IMU 偏航角速度，唯一发布 `/odom` 与该 TF。两种模式使用不同
bridge YAML，但始终只启动一个 `ros_gz_bridge` 进程。

适配参数位于 `config/sensor_covariance.yaml`。其中轮速横向速度标准差
`0.005 m/s` 小于纵向的 `0.02 m/s`，用有限协方差表达差速底盘的非完整
约束；IMU 角速度和线加速度标准差分别为 `0.0021 rad/s` 与
`0.0207 m/s²`，绝对姿态明确标记为未知。不能只修改 bridge 消息类型：
Gazebo DiffDrive 实际发布的是不带协方差的 `gz.msgs.Odometry`。

可自动比较正常附着和单侧低摩擦条件下的轮速、融合里程计与自研 2D
SLAM：

```bash
ros2 launch slam_robot_bringup imu_fusion_regression.launch.py \
  odometry_mode:=wheel_imu profile:=normal
ros2 launch slam_robot_bringup imu_fusion_regression.launch.py \
  odometry_mode:=wheel_imu profile:=slip \
  left_wheel_friction:=0.15 right_wheel_friction:=1.2
```

回归会检查协方差有效性、直行和平转误差；正常路面要求融合航向不明显
劣化，打滑场景则要求它相对轮速航向至少改善 `2°`。任一条件不满足都会
返回非零退出码。

`worlds/degenerate_corridor.sdf` 是自研 SLAM 的受控退化场景：33 m 长、
2.6 m 净宽的平行墙走廊，中段在 LiDAR 量程内没有纵向几何特征，入口和
出口各保留一个避开中心线的非对称锚点。可通过 bringup 包的
`corridor_slam_regression.launch.py` 启动。

`worlds/repeated_rooms.sdf` 包含两间相距 8 m、主要墙体和四个大型路标
完全重复的房间，并在每间房各保留一个微弱的独有特征。中心通道供自动
往返路线使用；该场景通过 `repeated_structure_slam_regression.launch.py`
验证相似地点不会触发假回环，同时真正返回后仍能建立闭环。

`worlds/large_warehouse.sdf` 是约 `26 × 20 m` 的非对称仓储场景，外围
留有 3 m 宽的矩形回归路线，内部布置错列货架、立柱和独特锚点。通过
`large_scale_slam_regression.launch.py` 可执行两圈约 155 m 的长时间
回归，验证关键帧、Path、回环重建和资源占用随运行时间增长时的稳定性。

`/ground_truth/odom` 由 Gazebo OdometryPublisher 根据世界位姿生成，父坐标系为 `world`，子坐标系为 `base_footprint`。它只用于算法误差评估，不能作为 SLAM 或导航输入。

默认会打开 RViz；仅运行 Gazebo 时可传入 `rviz:=false`。

在 WSL2 中 launch 默认启用 Mesa D3D12 并选择 NVIDIA 显卡，避免回退到 CPU 软件渲染。可使用 `wsl_gpu_adapter:=AMD` 或 `wsl_gpu_adapter:=Intel` 选择其他适配器，也可使用 `use_wsl_gpu:=false` 禁用该设置。

当前性能配置：

- 物理循环为 250 Hz，仿真实时率目标为 1.0。
- Gazebo 阴影和激光射线可视化关闭；`/scan` 数据不受影响。
- RViz 刷新率为 20 Hz，TF 显示默认关闭。
- 当前 Jazzy 随附的 JointStatePublisher 可能忽略 `<update_rate>`，因此 `/joint_states` 会跟随物理循环发布；这不影响 SLAM。
