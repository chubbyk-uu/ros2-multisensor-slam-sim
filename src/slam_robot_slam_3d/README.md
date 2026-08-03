# slam_robot_slam_3d

本包负责 3D 激光 SLAM 的算法适配。当前第一条成熟基线使用 MOLA 官方
GICP 流水线，以 Gazebo 的 `/lidar_3d/points` 运行纯激光里程计（LO）。

## 依赖

ROS 2 Jazzy 下安装：

```bash
sudo apt install \
  ros-jazzy-mola-lidar-odometry \
  ros-jazzy-mola-bridge-ros2 \
  ros-jazzy-mola-metric-maps
```

后两个包包含 MOLA 在运行时动态加载的 ROS 桥和地图插件，需要显式安装。

## 启动

启动 Gazebo 3D 机器人、输入检查、MOLA 和官方 RViz 配置：

```bash
ros2 launch slam_robot_slam_3d mola_lo_simulation.launch.py
```

无图形界面运行：

```bash
ros2 launch slam_robot_slam_3d mola_lo_simulation.launch.py \
  gui:=false rviz:=false
```

只接入已经存在的点云和 TF：

```bash
ros2 launch slam_robot_slam_3d mola_lo.launch.py
```

可用键盘驾驶机器人：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

## 数据契约与输出

默认输入为 `lidar_3d_link` 坐标系下的 `/lidar_3d/points`。启动器先检查
`x/y/z` 字段、点数、消息布局和 `frame_id`；检查失败时不会启动算法。
Gazebo 点云没有逐点时间字段，因此当前明确关闭去畸变并设置
`MOLA_IGNORE_NO_POINT_STAMPS=true`。这是一条有效的几何 LO 基线，但不是
完整 LIO 验证，也不会伪造时间戳。

主要输出：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/lidar_odometry/pose` | `nav_msgs/Odometry` | `map` 中的 3D `base_link` 位姿 |
| `/lidar_odometry/pose_quality` | `std_msgs/Float32` | 当前匹配质量 |
| `/lidar_odometry/localmap_points` | `sensor_msgs/PointCloud2` | `map` 坐标系局部地图 |
| `/mola_diagnostics/lidar_odom/status` | `std_msgs/String` | MOLA 状态信息 |

TF 发布职责保持唯一：MOLA 只发布 `map -> odom`，Gazebo 里程计发布
`odom -> base_footprint`，`robot_state_publisher` 发布
`base_footprint -> base_link -> lidar_3d_link`。MOLA 自带的 footprint TF
已关闭，避免覆盖机器人真实的 `0.135 m` 底盘高度。

默认 `use_imu_gravity:=false`，因此算法不会订阅真实 IMU。设置
`use_imu_gravity:=true` 只会把 `/imu/data_raw` 用作 ICP 重力方向先验，
仍然没有逐点 IMU 去畸变，不应称为完整 LIO。后续 LIO 回归将使用包含
厂家逐点时间字段的真实 rosbag 或 PCAP。

对于平面轮式机器人，可按需传入 `enforce_planar_motion:=true`。默认保持
MOLA 官方 GICP 的六自由度估计，以便先建立不额外裁剪能力的基线。
