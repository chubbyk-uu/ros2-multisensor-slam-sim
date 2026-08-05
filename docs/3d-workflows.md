# 3D 建图、导航与验收

当前在线 3D 基线使用 RTAB-Map；MOLA GICP 作为不伪造逐点时间字段的纯激光
里程计对照。两者使用相同的 3D LiDAR 机器人变体，但不能同时发布
`map -> odom`。

## 固定 3D 数据集

自研算法与 RTAB-Map 的统一输入已经固化为结构化世界两圈 MCAP：

```bash
ros2 launch slam_robot_slam_3d structured_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_3d_reference"
```

数据包不记录动态 `/tf` 或任何 SLAM 输出。使用前运行：

```bash
ros2 run slam_robot_slam_3d dataset_contract_check \
  "${SLAM_WS}/bags/structured_3d_reference"
```

完整话题契约、哈希和回放方式见[固定数据集](datasets.md)。

## 自研点云预处理

先启动预处理节点，再回放同一固定输入：

```bash
ros2 launch slam_robot_slam_3d custom_3d_preprocessing.launch.py

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=10.0
```

节点输出 `/custom_slam_3d/points_filtered`，其 `frame_id` 和时间戳与原始扫描
一致。过滤顺序为非有限值、量程、本体包围盒、`0.10 m` 体素；参数全部在
`custom_3d_slam.yaml`。该阶段不发布任何 TF。用下面的工具收集多帧诊断并
检查平均处理预算：

```bash
ros2 run slam_robot_slam_3d preprocessing_regression \
  --ros-args -p minimum_samples:=1000
```

地面分类和离群点过滤尚未接入。首版 scan-to-local-map 前端直接使用该过滤
点云和轮速 + IMU 运动初值；后续根据配准对地面约束的需求确定地面分类输出，
而不直接把地面全部删除。

## 自研 scan-to-local-map 前端

固定包验证分三个终端运行：

```bash
ros2 launch slam_robot_slam_3d custom_3d_front_end.launch.py

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=1.0

ros2 run slam_robot_slam_3d front_end_regression
```

前端采用标准 PCL GICP，并以 `/odom` 的帧间增量作为初值；它不是把 EKF 位姿
直接当作激光结果。关键帧按平移/旋转阈值插入，局部子图只保留最近 12 帧并
再次体素化，因此运行时间和内存不会随全局路程无限增长。诊断包含匹配状态、
对应点数、RMSE、相对运动初值修正量、关键帧数和完整回调耗时。最终对应点
还利用 GICP 目标邻域协方差的最小特征向量作为表面法向，构造平面运动的
`x/y/yaw` 点到平面信息矩阵。平移弱特征方向采用双阈值连续处理：严重退化
时完全保留轮速 + IMU 预测，过渡区逐步恢复 GICP 修正，强方向不受影响。
诊断同时给出特征值、弱方向、实际缩放量和退化标志。

固定包的正式时序验收使用 `rate:=1.0`。点云比里程计先到时，节点会暂存最新
扫描并在匹配时间窗内的里程计到达后处理，而不是立即丢帧。`rate:=2.0` 只作
计算压力测试，不能替代在线时间契约。当前节点不发布 TF、全局地图或回环
结果，不能接 Nav2。

专用 70 m 平行墙世界把端墙和少量锚点移出雷达量程，用下面的自动回归验证
严格弱几何段及进入/离开退化区的状态切换：

```bash
ros2 launch slam_robot_slam_3d corridor_3d_regression.launch.py
```

这与 2D 使用的 `degenerate_corridor.sdf` 是两套场景；3D 版本针对 20 m
量程和 16 线扫描单独设计，不用结构丰富的 `structured_loop_3d.sdf` 冒充
秩亏环境。

正常结构中的快速旋转与单侧轮胎打滑使用：

```bash
ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=rotation

ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=slip
```

两种 profile 都检查匹配接受率、真值误差、相对 `/odom` 劣化、退化误报、
回调间隔和 P95。打滑 profile 还要求低摩擦实际造成可测的里程计误差。
首版 GICP、关键帧和局部子图参数由固定包、长走廊、快速旋转和打滑四类测试
共同约束；进入回环后端阶段后，修改这些参数必须复跑该集合。

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

RTAB-Map 是成熟算法验收基线；自研点云预处理、GICP 前端和首版有界局部子图
以及弱几何退化处理已经完成，后续逐步加入回环后端和地图输出模块替换，
同时保留相同的输入契约、TF 职责和回归场景。自研链路
完成后将接入现有 Nav2 在线入口：自研 SLAM 发布实时二维导航投影并独占
`map -> odom`，Nav2 继续使用 3D 点云局部避障。

最终阶段增加 Frontier Exploration。探索器只负责从实时占据栅格中提取、
筛选和排序未知边界目标，再通过 Nav2 标准 `NavigateToPose` 动作导航；它不
替代 Nav2 的规划控制，也不直接发布 `/cmd_vel`。自动验收从未知地图开始，
检查覆盖率、不可达目标处理、回环修正后的重规划、碰撞和地图自动保存。

相机随后用于增强地点识别和全局重定位，IMU 用于支持具备逐点时间数据的
LIO；不计划融合 2D 与 3D LiDAR。详细分阶段任务和验收标准见项目根目录
`plan.md` 的 8.3、8.4 节。
