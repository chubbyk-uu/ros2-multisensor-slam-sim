# slam_robot_slam_3d

本包负责 3D 激光 SLAM 的算法适配。在线完整基线使用 RTAB-Map，以 Gazebo
的 `/lidar_3d/points`、局部 `/odom`、ICP 约束、回环和位姿图构建全局 3D
地图；MOLA GICP 流水线保留为独立的纯激光里程计（LO）对照。

## 依赖

ROS 2 Jazzy 下安装：

```bash
sudo apt install \
  ros-jazzy-mola-lidar-odometry \
  ros-jazzy-mola-bridge-ros2 \
  ros-jazzy-mola-metric-maps
```

后两个包包含 MOLA 在运行时动态加载的 ROS 桥和地图插件，需要显式安装。

在线 RTAB-Map 基线还需要：

```bash
sudo apt install ros-jazzy-rtabmap-ros
```

## 启动

启动 Gazebo 3D 机器人、点云输入检查、RTAB-Map 和专用 RViz：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py
```

默认从新地图开始。RTAB-Map 会在收到每个关键帧时持续写入 SQLite 数据库，
因此按一次 `Ctrl+C` 正常退出后，`~/.ros/rtabmap_3d.db` 可用于后续检查或
继续建图。该默认路径不依赖启动终端的当前目录。要复用已有数据库：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  reset_database:=false
```

无图形界面运行：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  gui:=false rviz:=false
```

在线基线的输入、输出和 TF 职责如下：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/lidar_3d/points` | `sensor_msgs/PointCloud2` | 3D LiDAR 输入，`lidar_3d_link` 坐标系 |
| `/odom` | `nav_msgs/Odometry` | Gazebo 或 2D EKF 的局部运动预测，不是真值 |
| `/rtabmap/mapData` | `rtabmap_msgs/MapData` | RViz 中显示的增量 3D 点云地图 |
| `/rtabmap/mapGraph` | `rtabmap_msgs/MapGraph` | 位姿图与回环边 |
| `/rtabmap/map` | `nav_msgs/OccupancyGrid` | 由 3D 点云投影得到的二维导航候选地图 |

RTAB-Map 启动器默认使用 `odometry_mode:=wheel_imu`：轮速与 IMU 偏航角速度
先由 `robot_localization` EKF 融合成 `/odom`，再作为 RTAB-Map 的局部运动
预测。RTAB-Map 不启动 `icp_odometry`；因此 EKF 唯一发布
`odom -> base_footprint`，RTAB-Map 唯一发布 `map -> odom`。这既避免 TF
冲突，也把“成熟全局 SLAM 基线”与下一阶段“替换局部前端”的实验分开。若需
纯轮式对照，可传入 `odometry_mode:=wheel`。

EKF 提供相邻关键帧的初始运动预测，但它本身不是激光里程计。配置因此按照
RTAB-Map 官方激光建图流程启用 `RGBD/NeighborLinkRefining=true`，用同一套
点到面 ICP 精修每条相邻关键帧边，减少圆柱、墙角等局部结构的厘米级重影；
回环仍由空间邻近检测后单独进行 ICP 验证。

本车是平面差速平台，所以配置启用 `Reg/Force3DoF` 和
`RGBD/ForceOdom3DoF`：机器人轨迹只估计 `x/y/yaw`，而输入扫描和累计地图
仍保持三维。不能关闭这两个参数后继续给 Nav2 使用同一地图；六自由度 ICP
的微小倾斜会使远处地面点在 `map` 中产生明显高度误差并污染实时障碍层。

当前点云只有统一消息时间戳，没有逐点时间字段。IMU 因此只用于 EKF 局部
运动预测，而不用于逐点 deskew；这仍不应称为完整 LIO。`/ground_truth/odom`
只用于评估，绝不作为输入。

专用 RViz 默认显示经 RTAB-Map 关键帧拼接后的地图，而不叠加实时原始点云；
同时仅在显示端过滤 `z < 0.05 m` 的地面点，并按高度着色。这样便于观察墙体
和障碍物，但不会改变送入 RTAB-Map 的点云、数据库或建图结果。需要检查原始
扫描时，可在 RViz 中重新启用 `Current 3D Scan`。

RTAB-Map 同时生成 `0.05 m/cell` 的二维占据栅格：低于 `0.05 m` 的点视为
地面，`0.05–0.45 m` 的点视为障碍物。机器人含 3D 雷达总高为 `0.35 m`，
投影上限额外保留 `0.10 m` 安全余量。建图后可用以下命令检查该地图是否已有
足够的尺寸、自由区和障碍区：

```bash
ros2 run slam_robot_slam_3d grid_contract_check
```

为在线导航避免首次订阅地图时重建全部历史关键帧栅格，RTAB-Map 会在每个
关键帧创建并存储其局部二维栅格。

`grid_contract_check` 是**事后**校验工具，不是启动门控。这一点和
`pointcloud_contract_check` 不同：点云在节点起来时就应该存在，所以它用
`OnProcessExit` 门控 RTAB-Map 启动；而栅格要等机器人跑出一段距离才存在，
在启动期检查只会必然超时。除基本布局外，它支持分辨率、米制边界以及
自由/占据单元数量阈值；结构化世界自动回归在路线结束后执行等价硬判据。

运行可重复的两圈结构化 3D 回归：

```bash
ros2 launch slam_robot_slam_3d structured_loop_regression.launch.py
```

回归沿固定 `72 m` 环线运行两圈，同时比较真值、局部 `/odom` 和
`map -> odom` 修正后的 SLAM 位姿。通过条件包括完成 `138 m` 以上路线、
至少一次 proximity 约束和非零全局修正、SLAM 闭合误差、图节点数量、地图
范围和自由/占据单元数量。`/ground_truth/odom` 只用于路线控制和评分，不进入
RTAB-Map。

该栅格是 RTAB-Map + Nav2 在线导航入口的地图输入：

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py
```

导航入口默认加载 `rtabmap_navigation_3d.rviz`，在 Nav2 地图、代价地图和
路径之外启用 `/lidar_3d/points` 实时扫描及 RTAB-Map 累计三维点云，并使用
可旋转的 Orbit 斜视视角。它不会改变算法输入或地图，只改变可视化。

该入口与既有 SLAM Toolbox / AMCL 导航入口**互斥**，不能同时启动——两者都会
发布 `map -> odom`。接口、TF 职责、栅格契约、受控闭环建图及高度语义多目标
导航均已有自动验收。完整验收命令为：

```bash
ros2 launch slam_robot_slam_3d structured_navigation_regression.launch.py
```

它先跑两圈形成稳定地图，再验证 `0.55 m` 高门洞可通过，以及 3D 点云把
`0.07 m` 细底座上方的 `0.40 m` 粗柱帽按最大外轮廓投影并迫使路径绕行。

纯 LiDAR 模式仍是一条完整可运行的 RTAB-Map SLAM 基线，但没有相机时视觉
词袋回环和基于图像的重定位会被禁用，只保留空间邻近预测加 3D ICP 验证。
因此单圈末端漂移较大时可能错过重访；两圈回归会显式要求 proximity 约束和
`map -> odom` 修正，避免把局部里程计碰巧闭合误判为 SLAM 生效。后续相机
接入用于增强全局地点识别，不是当前 3D 点云建图的启动前提。

### 输入同步与 QoS

`/odom` 和 `/lidar_3d/points` 由两条独立时钟链路打时间戳，因此配置使用
`approx_sync: true` 而不是 RTAB-Map 默认的精确时间同步；
`approx_sync_max_interval` 取 `0.02 s`，是 `50 Hz` 里程计最坏 `10 ms` 偏差的
两倍，且只有 `100 ms` 点云周期的五分之一，不会跨接到相邻扫描。仿真中精确
同步碰巧也不丢帧（两个时间戳栅格恰好成整数倍），但这不是设计保证，改动
EKF 频率就会静默失效，测量数据见
[性能说明](../../docs/performance.md)。

`ros_gz_bridge` 以 `BEST_EFFORT` 发布点云，所以 `qos_scan_cloud` 必须保持
`2`（best effort）；改成可靠订阅会因 QoS 不兼容而一条消息都收不到。
`/odom` 由 `robot_localization` 以 `RELIABLE` 发布，因此 `qos_odom` 设为 `1`。

### MOLA 里程计对照

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
