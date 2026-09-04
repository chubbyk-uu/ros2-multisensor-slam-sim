# slam_robot_slam_3d

本包负责 3D 激光 SLAM 的算法适配。在线完整基线使用 RTAB-Map，以 Gazebo
的 `/lidar_3d/points`、局部 `/odom`、ICP 约束、回环和位姿图构建全局 3D
地图；MOLA GICP 流水线保留为独立的纯激光里程计（LO）对照。

## 依赖

ROS 2 Jazzy 下安装：

```bash
sudo apt install \
  libpcl-dev \
  ros-jazzy-pcl-conversions \
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

### 固定数据集

录制结构化世界的两圈固定输入：

```bash
ros2 launch slam_robot_slam_3d structured_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_3d_reference"
```

录制内容只有 3D 点云、轮速、IMU、统一 `/odom` 运动初值、静态外参、机器人
描述、仿真时钟和仅供评分的真值，不包含 RTAB-Map 或自研算法输出。检查并
回放：

```bash
ros2 run slam_robot_slam_3d dataset_contract_check \
  "${SLAM_WS}/bags/structured_3d_reference"

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=2.0
```

固定包的消息数量、哈希和使用边界见
[数据集说明](../../docs/datasets.md)。

### 自研 3D 点云预处理

第一段自研 C++17 数据链路已经可独立运行：

```bash
ros2 launch slam_robot_slam_3d custom_3d_preprocessing.launch.py
```

节点订阅 `/lidar_3d/points`，依次执行有限值检查、欧氏量程裁剪、雷达坐标系下
的机器人本体包围盒裁剪和 `0.05 m` PCL `VoxelGrid` 体素降采样，再发布
建图级 `/custom_slam_3d/points_filtered`。前端从同一消息内部派生 `0.10 m`
注册点云，因此无需增加第二个 ROS 话题或做时间同步。默认参数集中在
`config/custom_3d_slam.yaml`，不发布 TF，也不与 RTAB-Map 或 MOLA 争用
`map -> odom`。

输出保留 `x/y/z/intensity` 和原始消息头；`ring` 不参与当前几何配准，因此不
复制到降采样点云。原始点云仍完整保留在 `/lidar_3d/points`。每帧的输入、
各级过滤点数和处理耗时发布到
`/custom_slam_3d/preprocessing_diagnostics`。固定包回放时可运行硬判据：

```bash
ros2 run slam_robot_slam_3d preprocessing_regression \
  --ros-args -p minimum_samples:=1000
```

默认链路不启用地面或离群点过滤。仓库保留低位回波和 PCL 统计离群点两个默认关闭的
实验开关，并以同一固定包完成基线、单项和组合 A/B；三种过滤组合均因精度、退化或
预处理预算被拒绝，因此不能作为默认参数。实验命令与完整数据见
[3D 工作流](../../docs/3d-workflows.md#预处理固定包对照)和
[性能记录](../../docs/performance.md#固定包地面与离群点-ab)。

### 自研 scan-to-local-map 前端

首版前端使用 PCL Generalized ICP（GICP）将每帧 `0.10 m` 注册点云配准到最近 12 个
关键帧组成的有界局部子图。`/odom`（默认轮速 + IMU EKF）只提供帧间运动
初值；匹配结果还必须通过收敛、对应点数、RMSE 和相对初值最大修正量门限。
子图带单调版本号；版本不变时匹配器复用 PCL GICP 的目标点云和目标协方差，
新增关键帧、清空或重播种后自动失效。点云只在其时间戳被前后两条 `/odom`
包夹时处理，并在两者之间插值；等待中的点云使用有界 FIFO。
固定数据集运行方式：

```bash
# 终端 1
ros2 launch slam_robot_slam_3d custom_3d_front_end.launch.py

# 终端 2
ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=1.0

# 终端 3
ros2 run slam_robot_slam_3d front_end_regression
```

前端发布 `/custom_slam_3d/laser_odom`、已配准当前帧
`/custom_slam_3d/registered_scan`、有界 `/custom_slam_3d/local_map`、全局
`/custom_slam_3d/map_cloud` 和逐帧 `/custom_slam_3d/front_end_diagnostics`。
`map_cloud` 由不可变全局关键帧按最近一次成功的位姿图优化结果后台分批重放，
使用 `global_map.voxel_leaf_size` 增量体素化；新请求只保留最新快照，因此不会
阻塞 10 Hz 前端或形成无界重建队列。同一关键帧另存距离裁剪后的 `0.05 m`
占据点云，并以
`occupancy_grid.minimum_obstacle_height` 到
`occupancy_grid.maximum_obstacle_height` 的高度带重放射线，持续发布标准
`/map` `nav_msgs/OccupancyGrid`；它保留未知空间，不把未观测的点云空洞误判为
自由空间。当前高度带为 `0.05–0.45 m`，覆盖最高 `0.35 m` 的机器人及余量。
局部前端坐标系仍为 `custom_slam_3d_odom`，回环后端成功优化后会发布唯一且
可配置的标准 `map -> odom`（默认开启）；EKF 继续唯一发布
`odom -> base_footprint`。
该链路已有 Scan Context 检索、GICP 几何复核、后台 SE(2) 位姿图、全局点云和
二维导航栅格。自研 Nav2 在线入口为：

```bash
ros2 launch slam_robot_slam_3d custom_3d_navigation_simulation.launch.py
```

它不启动 RTAB-Map、Map Server 或 AMCL：自研 SLAM 独占 `map -> odom`，Nav2
订阅 `/map`，局部代价地图继续订阅实时 `/lidar_3d/points` 高度带避障。

自研状态默认原子保存到 `~/.ros/custom_slam_3d.snapshot`。版本 5 快照包含每个
关键帧的注册/占据两份扫描、传感器外参、前端/轮速位姿、协方差、已提交回环约束
和优化位姿；版本 1–4 不再兼容，加载时 FATAL 退出并要求重新建图。降级读取本可
实现（旧文件的 `0.10 m` 点云同时充当两份即可），拒绝是因为不值得为个位数的存量
快照维护兼容分支——**升级即作废全部已有快照**。快照不依赖
启动目录，也不会提交到 Git。正常退出在线导航入口时自动保存：

```bash
# 新建地图（默认），退出时保存
ros2 launch slam_robot_slam_3d custom_3d_navigation_simulation.launch.py

# 从末端位姿继续建图
ros2 launch slam_robot_slam_3d custom_3d_navigation_simulation.launch.py \
  mode:=mapping load_snapshot:=true

# 只定位：地图、关键帧和回环约束保持只读
ros2 launch slam_robot_slam_3d custom_3d_navigation_simulation.launch.py \
  mode:=localization load_snapshot:=true save_on_shutdown:=false
```

只定位模式要求仿真机器人从保存时的末端位姿恢复，随后使用只读全局点云作
scan-to-map 匹配。它不支持在地图任意位置启动后的全局重定位；该能力不在当前
收口范围，不能用“成功加载地图”冒充已经完成。

快照当前为版本 5。版本 1/2 只保存一份关键帧点云，无法恢复当前注册/占据点云
分工，因此明确拒绝并要求重新建图；项目目前没有需要维护的外部旧快照用户。版本 3
另有原因：它以两种不兼容的字节布局发布过，旧文件在新布局下会把关键帧数当成占据
投影契约读，并报出误导性的“契约无效”，因此版本号递增而不是复用；版本 4 没有
当前要求的完整性校验，也明确拒绝。
占据点云只持久化当前投影真正可用的地面自由射线和高度带内障碍点，高于障碍带
或超出投影距离的点不会写入。v4 同时保存输入体素、障碍高度带、最大射线距离和
平面运动模式；v5 保留这些投影契约并增加写后回读校验和与持久化保证。恢复时任一
配置不一致都会明确失败，避免历史 8 m 数据在新配置下被静默当成 10 m 数据。

恢复时会把全部历史关键帧的前端位姿刚体变换到地图系。关键帧存的是前端自身
坐标系下的位姿，而恢复后新建的关键帧在地图系；不做这一步，跨越接缝的第一条
位姿图边会把整个累积修正当成一次测量写进图里。刚体变换保持所有相对位姿不变，
而位姿图的边正是由相对位姿构造的。整条链路由
`snapshot_resume_regression` 端到端验收（见下文回归章节）；`localization`
恢复另由固定包 v5 回归覆盖，当前 `11/11` 判据通过。

### 在线自主探索

```bash
# 自研 3D SLAM + Nav2 + Frontier Exploration
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py

# 从 SDF collision 自动选取可复现的三维安全随机出生点
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py \
  random_spawn:=true spawn_seed:=42

# 同一探索器切换到 RTAB-Map 成熟基线
ros2 launch slam_robot_slam_3d rtabmap_3d_exploration_simulation.launch.py
```

探索器使用实时二维投影地图选择目标，Nav2 的局部 voxel layer 仍直接消费
3D 点云完成高度带避障。自研入口在完成时调用
`/scan_to_map_odometry_3d/save_snapshot`；RTAB-Map 入口不调用该服务，由其
SQLite 数据库持续持久化。两条链路共享同一 C++ frontier 检测和 Nav2 动作
调度实现。

结构化世界的无人工目标回归：

```bash
ros2 launch slam_robot_slam_3d frontier_exploration_regression.launch.py
ros2 launch slam_robot_slam_3d rtabmap_frontier_exploration_regression.launch.py
```

差速机器人默认启用平面运动约束：GICP 用完整三维几何求解，输出基座位姿再
投影到 `x/y/yaw`，避免不可观测的微小横滚、俯仰和高度误差污染后续二维导航
投影。前端还从最终对应点的局部表面法向构造 `x/y/yaw` 点到平面信息矩阵，
发布平移与完整矩阵的特征值、平移比值、偏航信息量和 `degenerate` 诊断。
退化处理不会拒绝整次匹配：它把 GICP 修正分解到平移信息矩阵的强、弱特征
方向，保留强方向修正；弱方向在严重退化区完全采用轮速 + IMU 预测，在双
阈值之间连续恢复 GICP 权重。`hasConverged()` 只表示优化器收敛，不能替代
几何可观测性判断。诊断另行区分 `translation/planar/yaw` 退化，并发布抑制后
实际修正量和各向异性平移协方差；连续真实配准失败达到配置门限时会清空并
以当前扫描重播种局部子图，恢复事件和累计次数均进入诊断。

超过 40 m 的专用平行墙世界会把入口、出口和锚点移出 20 m 雷达量程，用于
验收严格秩亏和恢复过程：

```bash
ros2 launch slam_robot_slam_3d corridor_3d_regression.launch.py
```

该回归同时比较真值、前端和 `/odom` 基线，并检查退化段检出率、正常段
误报、弱方向漂移、接受率、回调间隔和 P95 处理预算。

正常结构中的三档快速旋转和左轮低摩擦专项回归使用同一入口：

```bash
ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=rotation

ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=slip
```

`rotation` 依次测试 `0.30/0.60/0.90 rad/s`；`slip` 自动把左轮摩擦系数降到
`0.15`，并要求里程计基线确实出现可测打滑误差，避免故障注入失效后假通过。
这两项和长走廊回归共同构成首版前端参数变更的必跑集合。

回环后端的正向固定包验收使用单一入口，明确要求至少一次成功位姿图提交，且
不允许后台提交丢弃或异常：

```bash
ros2 launch slam_robot_slam_3d custom_3d_loop_regression.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=1.0
```

反向保护使用长走廊回归：候选检索不按前端当前位置截断；GICP 后才以
`maximum_front_end_translation_disagreement` 检查图一致性，防止重复走廊把数十米
的错误匹配写入位姿图。

存档恢复用同一固定包分两段验收：前段建图到 `--split` 并经 `~/save_snapshot`
保存，后段用全新前端 `load_snapshot:=true` 从该时刻继续放完余下的包：

```bash
ros2 run slam_robot_slam_3d snapshot_resume_regression \
  --bag "${SLAM_WS}/bags/structured_3d_reference"
```

后段按前段写下的锚点度量地图系位姿，因此接缝上的任何跳变都直接显示为误差，
不会被新原点吸收。两条判据要求两段各自至少提交一次位姿图优化——否则这次运行
既没有非平凡的坐标修正可重基准，也没有让跨接缝的那条边进过优化器，会判 FAIL
并提示调整 `--split`，而不是报一个没验证到东西的 PASS。

### 在线 RTAB-Map

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
冲突，也让成熟 RTAB-Map 与自研 3D 前端保持为可独立验证的链路。若需
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
`map -> odom` 修正，避免把局部里程计碰巧闭合误判为 SLAM 生效。该纯 LiDAR
链路保持独立，不再接入相机约束。

### 在线 RTAB-Map RGB-D

本节是项目采用的 RGB-D 建图与定位链路，用于验证视觉注册、视觉回环和深度地图。
组合导航时由独立 3D LiDAR 负责 Nav2 障碍感知；两类传感器按任务松耦合分工，
不构成联合 SLAM。

启动 Gazebo、轮速 + IMU EKF、官方 RGB-D 同步器、RTAB-Map 和专用 RViz：

```bash
ros2 launch slam_robot_slam_3d rtabmap_rgbd_simulation.launch.py
```

该入口直接消费 `/camera/color/image_raw`、`/camera/depth/image_raw` 和
`/camera/color/camera_info`，由 `rtabmap_sync/rgbd_sync` 生成原子
`/rtabmap/rgbd_image`。RTAB-Map 订阅外部 `/odom`，不启动
`rgbd_odometry`，因此 EKF 仍是 `odom -> base_footprint` 的唯一发布者，
RTAB-Map 仍是 `map -> odom` 的唯一发布者。默认数据库为
`~/.ros/rtabmap_rgbd.db`；继续已有地图时使用 `reset_database:=false`。

配置使用 `Reg/Strategy=0` 的视觉注册和视觉词袋地点识别；二维导航候选图明确
由深度图生成（`Grid/Sensor=1`）。3D LiDAR 仍随机器人生成，用于 RViz 检查及
组合导航的障碍感知，但不进入 RTAB-Map 输入。

RGB-D 组合消息同样属于大消息，因此相机 bridge、`rgbd_sync` 和 RTAB-Map 均使用
`fastdds_rgbd.xml`；可通过 `rgbd_dds_profiles_file:=...` 覆盖，或在启动前设置
`FASTRTPS_DEFAULT_PROFILES_FILE`。默认不桥接 `/camera/depth/points`，避免无意义
的稠密点云复制。

无图形界面冒烟：

```bash
ros2 launch slam_robot_slam_3d rtabmap_rgbd_simulation.launch.py \
  gui:=false rviz:=false
```

短程在线冒烟只证明数据链路、视觉关键帧、数据库、TF 和深度二维栅格工作；下述
固定回放和在线两圈入口分别覆盖可复现算法行为与完整 `30 Hz` 并发运行。

录制体积受控的一圈 RGB-D 固定输入：

```bash
ros2 launch slam_robot_slam_3d structured_rgbd_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_rgbd_reference"

ros2 run slam_robot_slam_3d rgbd_dataset_contract_check \
  "${SLAM_WS}/bags/structured_rgbd_reference"

ros2 launch slam_robot_slam_3d play_rgbd_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_rgbd_reference"
```

固定包相机为 `10 Hz`、MCAP 为 `zstd_fast`，在线入口仍保持 `30 Hz`。包中保留
RGB-D 与 3D LiDAR 是为了让独立链路在需要时使用字节一致输入做对照，不代表
RGB-D 链路消费了 LiDAR。话题白名单、计数、哈希和压缩 A/B 见
[数据集说明](../../docs/datasets.md#结构化-rgb-d-多传感器参考包)。

视觉回环正向回归必须使用两圈纹理包：

```bash
ros2 launch slam_robot_slam_3d structured_rgbd_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_rgbd_textured_loop_reference" laps:=2

ros2 launch slam_robot_slam_3d rtabmap_rgbd_fixed_regression.launch.py \
  bag:="${SLAM_WS}/bags/structured_rgbd_textured_loop_reference"
```

该入口自动检查两圈输入、视觉特征覆盖、视觉回环、轨迹误差、资源、二维地图，且在
输入里程计明显漂移时要求 SLAM 给出实质改善。一圈包没有同向重走路段，只作为数据
契约样本，不能拿它要求视觉回环。当前验收结果见
[验收记录](../../docs/acceptance.md#rtab-map-rgb-d-在线基线)。

完整在线活动闭环：

```bash
ros2 launch slam_robot_slam_3d rtabmap_rgbd_online_regression.launch.py
```

该入口无界面运行纹理世界两圈，自动记录 CameraInfo 长程频率、RTAB-Map 轨迹、视觉
回环、地图、CPU、RSS 和数据库，并在结束后判定退出。默认要求相机至少 `9000` 帧且
实测不低于 `27 Hz`；当前正式结果为 `11079` 帧、`30.304 Hz`，八项检查全部通过。

交互式在线建图并用 Nav2 点击导航：

```bash
ros2 launch slam_robot_slam_3d rtabmap_rgbd_navigation_simulation.launch.py
```

该组合入口只负责编排：内部仍复用独立 RGB-D SLAM 与通用在线 Nav2 launch，不复制
算法参数。它是最终的“RGB-D 视觉建图 + 3D LiDAR 避障”松耦合导航架构：RTAB-Map 的深度
二维投影用于全局规划，独立 3D LiDAR 用于局部体素避障和碰撞监视。LiDAR 没有进入
视觉 SLAM，RGB-D 也没有约束 LiDAR 位姿图，因此不能称为 LiDAR—视觉融合。导航
RViz 提供 `Nav2 Goal`，并同时显示 RGB、深度、彩色累计点云、实时 LiDAR、代价地图
和路径。目标必须落在当前地图已经观测到的自由区域。

无界面的最终组合动态障碍验收：

```bash
ros2 launch slam_robot_slam_3d \
  rtabmap_rgbd_navigation_safety_regression.launch.py
```

前向 RGB-D 会先通过三个可达目标渐进扩展已知自由区并返回原点，再执行公共动态
障碍判据；3D LiDAR 只负责障碍层和碰撞监视。成功时终端输出 `VERDICT PASS` 并
自动退出。

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
仍然没有逐点 IMU 去畸变，不应称为完整 LIO。完整 LIO 不在当前项目范围；若未来
重新评估，必须先使用包含厂家逐点时间字段的真实 rosbag 或 PCAP。

对于平面轮式机器人，可按需传入 `enforce_planar_motion:=true`。默认保持
MOLA 官方 GICP 的六自由度估计，以便先建立不额外裁剪能力的基线。
