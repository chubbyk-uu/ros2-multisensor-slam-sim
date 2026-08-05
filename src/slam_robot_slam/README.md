# slam_robot_slam

本包同时保存 `slam_toolbox` 官方基线和自研 C++ 2D SLAM。两条链路彼此独立，便于使用相同仿真数据比较结果。

独立启动 SLAM Toolbox：

```bash
ros2 launch slam_robot_slam mapping.launch.py
```

该命令要求仿真或真实机器人已经提供 `/scan`、`odom -> base_footprint` 和机器人内部 TF。

主要配置：

- 地图分辨率：0.05 m/cell。
- 地图更新周期：2 s。
- 激光量程：0.12～12 m。
- 关键帧距离阈值：0.15 m。
- 关键帧旋转阈值：0.05 rad。
- 启用扫描匹配和回环检测。
- 关闭交互式位姿图编辑，减少不必要的后台开销。

通过本包的 `mapping.launch.py` 或项目统一建图 launch 启动时，自动保存默认开启。按一次 `Ctrl+C` 后，会在 SLAM Toolbox 退出前保存导航占据栅格和可恢复的位姿图。默认前缀是启动命令当前目录下的 `maps/slam_map`；`maps/` 是运行输出目录，其中的地图不进入版本控制，因此自动保存不会弄脏工作区。

自定义前缀：

```bash
ros2 launch slam_robot_slam mapping.launch.py \
  map_output_prefix:="${SLAM_WS}/maps/room_01"
```

关闭自动保存可传入 `auto_save_map:=false`。原有工具仍可用于运行中手动保存检查点：

```bash
ros2 run slam_robot_slam save_slam_map \
  "${SLAM_WS}/maps/slam_map"
```

成功时会生成 `.yaml`、`.pgm`、`.posegraph` 和 `.data` 四个文件。自动保存期间不要重复按 `Ctrl+C`，应等待终端显示 `Save completed`。

其中 `SLAM_WS` 应指向仓库根目录。

## 自研 C++ SLAM

2D 基线已完成激光预处理、局部扫描匹配、占据栅格、后台回环优化、
优化后地图重建和固定数据集离线回归：

- LaserScan 有效量程、`NaN` 和 `Inf` 过滤。
- 可配置的点间隔降采样。
- 极坐标到二维笛卡尔点集转换。
- `/custom_slam/scan_points` PointCloud2 发布。
- Gazebo `/ground_truth/odom` 真值评估基准。
- 通用 MCAP 录制入口，以及不含算法输出和动态 `/tf` 的固定回归数据集。
- 独立点到线 ICP 对照实现及单元测试。
- 轮式里程计位姿预测和移动阈值过滤。
- Karto 风格的相关栅格粗到细搜索。
- 当前扫描到最近 20 个关键帧局部子图的匹配。
- 从保留原始光束索引的当前扫描估计局部表面法向，以归一化点到线平移
  Hessian 识别弱观测方向，并按信息比值连续融合激光修正与里程计预测。
- 匹配分数、最少重合点和失败回退机制。
- 将相关分数限定在有局部子图支撑的点上，并独立检查支撑点占比。
- 基于关键帧的射线清空、末端占用和 log-odds 概率更新。
- 自动扩展的 0.05 m 分辨率占据栅格。
- 使用 16×16 稠密块保存已观测栅格，减少逐单元哈希的内存和访问开销。
- 保存全部成功关键帧，最近 20 帧只用于局部匹配。
- 使用 Ceres 2.2 建立二维位姿图、顺序约束和可带 Huber 核的回环约束。
- 排除近期关键帧后按空间距离筛选历史回环候选。
- 使用候选附近多关键帧子地图和独立高门限相关匹配验证回环。
- 在后台工作线程中完成回环子地图匹配和 Ceres 优化。
- 主线程以事务方式合并优化结果，并同步关键帧路径与当前定位。
- 随关键帧保存降采样射线、命中状态和传感器原点。
- 按优化位姿分批重放全部射线，完成后原子替换占据栅格。
- 地图重建完成当前快照后只处理排队的最新快照，避免新回环反复清零进度。
- 两条 Path 降频、按运动量采样、限制长度并分批裁剪历史位姿。
- 真值、轮式里程计和匹配轨迹自动对比工具。

仿真已经运行时，可独立启动预处理节点：

```bash
ros2 launch slam_robot_slam custom_slam.launch.py
```

一条命令启动完整开发环境：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

默认在收到一次 `Ctrl+C` 后，先把 `/custom_slam/map` 保存为启动目录下的
`maps/custom_slam_map.yaml` 和 `maps/custom_slam_map.pgm`，再停止地图发布
节点。可使用 `map_output_prefix` 更改文件前缀，或传入
`auto_save_map:=false` 关闭。当前自研位姿图不支持序列化，因而不会生成
`.posegraph` 和 `.data`；保存的 YAML/PGM 用于 AMCL 和 Nav2 静态地图定位。
该输出不进入版本控制；随仓库分发的自研 SLAM 演示地图在
`maps/reference/custom_slam_map.yaml/.pgm`。

专用 RViz 使用 `map` 固定坐标系：黑白栅格为自研地图，红色为原始
`/scan`，绿色为预处理点集，青色为局部子图匹配后的扫描，黄色为匹配
轨迹，紫色为位姿图关键帧路径。当前阶段发布 `/custom_slam/map` 和
`map -> odom`，但不会发布标准 `/map`。自研入口与 SLAM Toolbox 入口
不能同时运行，避免重复发布同一段 TF。

参数位于 `config/laser_preprocessor.yaml`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` | `/scan` | 原始 LaserScan |
| `output_topic` | `/custom_slam/scan_points` | 处理后的 PointCloud2 |
| `minimum_range` | `0.12` | 最小有效距离，单位 m |
| `maximum_range` | `12.0` | 最大有效距离，单位 m |
| `point_stride` | `1` | 每隔多少个测量点取一个 |

扫描匹配参数位于 `config/scan_matcher.yaml`。默认前端不是逐帧
ICP，而是参考 slam_toolbox/Karto 和 Cartographer 的成熟结构：

```text
轮式里程计预测
  -> 最小移动距离/转角过滤
  -> 当前扫描与局部关键帧相关栅格粗匹配
  -> 小范围精匹配
  -> 分数与重合点检查
  -> 成功关键帧、顺序约束
  -> 后台：历史候选筛选、回环子地图匹配与 Ceres 全图优化
  -> 主线程：事务式合并优化结果
  -> 激光轨迹与位姿图关键帧路径
```

位姿图残差参考
[Ceres 官方二维 Pose Graph 示例](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_2d/)
的局部坐标约束形式；求解器使用与 SLAM Toolbox 默认建议一致的
`SPARSE_NORMAL_CHOLESKY` 和 Levenberg-Marquardt，并固定首节点消除
规范自由度。

核心输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/custom_slam/laser_odom` | `nav_msgs/Odometry` | 匹配前端估计位姿 |
| `/custom_slam/laser_path` | `nav_msgs/Path` | 匹配轨迹 |
| `/custom_slam/pose_graph_path` | `nav_msgs/Path` | 成功关键帧及顺序边路径 |
| `/custom_slam/aligned_scan_points` | `sensor_msgs/PointCloud2` | 对齐到 `map` 的当前扫描 |
| `/custom_slam/matcher_diagnostics` | `diagnostic_msgs/DiagnosticArray` | 前端逐扫描分类、逐次真实匹配可观测性及累计计数 |
| `/custom_slam/map` | `nav_msgs/OccupancyGrid` | `map` 坐标系下的关键帧占据栅格 |

TF 发布职责：

```text
map -> odom                 自研扫描匹配前端
odom -> base_footprint      Gazebo 差速驱动里程计
base_footprint -> lidar_link  robot_state_publisher
```

`map -> odom` 按 `T_map_base × inverse(T_odom_base)` 计算。这样轮式里程计
即使累计漂移，原始激光和机器人模型在 RViz 的 `map` 坐标系下仍会与
匹配地图对齐。

地图参数也位于 `config/scan_matcher.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `map_frame` | `map` | 自研地图、轨迹和匹配位姿坐标系 |
| `map.resolution` | `0.05` | 地图分辨率，单位 m/cell |
| `map.ray_stride` | `2` | 每隔多少束激光执行一次地图更新 |
| `map.publish_period` | `0.5` | 地图发布周期，单位 s |
| `map.rebuild_keyframes_per_cycle` | `4` | 每次后台重建处理的关键帧数 |
| `map.rebuild_period` | `0.02` | 地图重建批次周期，单位 s |
| `maximum_path_poses` | `5000` | 激光轨迹最多保留的降采样位姿数 |
| `path.publish_period` | `0.5` | 两条 Path 的发布周期，单位 s |
| `path.minimum_translation` | `0.03` | 激光轨迹新增位姿的最小平移，单位 m |
| `path.minimum_rotation` | `0.03` | 激光轨迹新增位姿的最小旋转，单位 rad |
| `pose_graph.maximum_path_poses` | `2000` | 位姿图 Path 最多发布的关键帧数 |
| `output.pose_translation_stddev` | `0.05` | 输出平面位置标准差，单位 m |
| `output.pose_rotation_stddev` | `0.05` | 输出偏航角标准差，单位 rad |
| `output.degenerate_translation_stddev` | `0.30` | 退化方向的位置标准差，单位 m |
| `output.dead_reckoning_translation_stddev_per_sqrt_meter` | `0.10` | 纯里程计每平方根米增加的平移标准差 |
| `output.dead_reckoning_rotation_stddev_per_sqrt_radian` | `0.10` | 纯里程计每平方根弧度增加的偏航标准差 |
| `matcher_diagnostics_topic` | `/custom_slam/matcher_diagnostics` | 前端诊断话题 |
| `matcher.degeneracy.enabled` | `true` | 启用扫描法向平移可观测性处理 |
| `matcher.degeneracy.normal_half_window` | `3` | 局部直线拟合在中心光束两侧使用的连续点数 |
| `matcher.degeneracy.maximum_neighbor_distance_base` | `0.05` | 同一表面相邻点距离的固定容差，单位 m |
| `matcher.degeneracy.maximum_neighbor_distance_ratio` | `0.05` | 随量程增长的相邻点距离容差比例 |
| `matcher.degeneracy.minimum_normal_count` | `20` | 有效局部表面法向的最低数量 |
| `matcher.degeneracy.minimum_effective_normal_count` | `5.0` | 单方向最低有效法向数量 |
| `matcher.degeneracy.minimum_information_ratio` | `0.05` | 弱/强方向有效法向数量比下限 |
| `matcher.degeneracy.weak_direction_correction_scale` | `0.0` | 弱方向激光修正比例的配置下限 |
| `map.hit_probability` | `0.70` | 命中单元的概率更新 |
| `map.miss_probability` | `0.40` | 射线穿过单元的空闲概率更新 |
| `map.minimum_probability` | `0.12` | log-odds 累积下限 |
| `map.maximum_probability` | `0.97` | log-odds 累积上限 |
| `pose_graph.sequential_translation_stddev` | `0.05` | 顺序边平移标准差，单位 m |
| `pose_graph.sequential_rotation_stddev` | `0.05` | 顺序边旋转标准差，单位 rad |
| `loop_closure.minimum_keyframe_separation` | `30` | 当前帧与候选的最小关键帧间隔 |
| `loop_closure.minimum_travel_distance` | `3.0` | 当前帧与候选间的最小累计行程，单位 m |
| `loop_closure.check_interval` | `10` | 每隔多少个关键帧检查一次回环 |
| `loop_closure.minimum_loop_closure_interval` | `30` | 两次已接受回环的最小关键帧间隔 |
| `loop_closure.search_radius` | `0.6` | 历史候选搜索半径，单位 m |
| `loop_closure.maximum_candidates` | `3` | 每次最多验证的最近候选数 |
| `loop_closure.candidate_submap_half_width` | `5` | 候选前后用于子地图的关键帧数 |
| `loop_closure.minimum_candidate_chain_size` | `10` | 候选附近要求的连续历史节点数 |
| `loop_closure.reject_degenerate_matches` | `true` | 不为平移可观测秩小于 2 的回环建立约束 |
| `loop_closure.maximum_correction_translation` | `0.50` | 回环最大平移修正，单位 m |
| `loop_closure.maximum_correction_rotation` | `0.25` | 回环最大旋转修正，单位 rad |
| `loop_closure.matcher.minimum_score` | `0.55` | 回环匹配的最小相关分数 |
| `loop_closure.matcher.minimum_support_fraction` | `0.50` | 回环扫描最小子图支撑占比 |
| `loop_closure.matcher.minimum_matched_points` | `100` | 回环匹配的最少重合点 |
| `loop_closure.matcher.degeneracy.enabled` | `true` | 将当前关键帧采集时的法向可观测性用于回环验证 |

`loop_closure.minimum_candidate_chain_size` 与关键帧间距存在隐式耦合。
候选链只统计落在 `loop_closure.search_radius` 内的连续历史节点，因此
需要满足：

```text
minimum_candidate_chain_size <= 1 + 2 * floor(search_radius / 关键帧间距)
```

当前取值下关键帧间距必须不超过 `0.12 m`，实测超过该值后全部候选都会被
链长门限拒绝。关键帧间距约为 `max(minimum_translation_for_update,
最大线速度 / 激光频率)`，当前为 `0.05～0.06 m`，约有两倍余量。若把
`minimum_translation_for_update` 提高到 `0.12` 以上，或把机器人限速提高
到 `1.2 m/s` 以上，回环会在没有警告的情况下停止工作，此时应同步下调
链长门限或增大搜索半径。

运行固定路线真值测试：

```bash
ros2 run slam_robot_slam scan_matcher_benchmark
```

测试工具只读取 `/ground_truth/odom` 计算误差，不会把真值反馈给匹配器。

运行两圈长闭环回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup custom_slam_development.launch.py \
  gui:=false use_rviz:=false

# 终端 2
ros2 run slam_robot_slam loop_closure_regression
```

回归工具向 `/cmd_vel` 发布两圈恒曲率开环指令，Gazebo 真值只用于结束后
计算误差。工具会自动检查回环接受、地图重建、地图持续发布、位姿图规模、
前端最大消息间隔、最终位姿误差和关键错误日志；同时要求至少收到一次有效
匹配诊断，且正常几何下 `translation_observable_rank < 2` 的次数严格为零。
任一检查失败时返回非零。

运行快速原地旋转回归：

```bash
# 终端 1 沿用上面的无界面开发环境

# 终端 2
ros2 run slam_robot_slam rotation_regression
```

工具先用 `0.30 rad/s` 预热一圈地图，再分别以 `0.30`、`0.60` 和
`1.00 rad/s` 原地旋转两圈。它会检查真值闭合误差、前端消息间隔、
地图面积和占用格增长、关键帧数量、匹配拒绝、误回环及严重日志。
预热用于让占用格统计建立在完整角度观测上；预热期间同样不允许拒配、
误回环、退化误报或严重日志。三个正式阶段也分别要求有效匹配数大于零且
退化误报数为零，避免累计计数掩盖单档问题。速度、圈数和各阈值可通过
`--help` 查看并覆盖。

2026-07-30 在后台回环架构整改前建立的圆形闭环基线，于关键帧
`2 -> 100` 自动接受真实回环：
相关分数 `0.954`、重合点 `354`，Ceres 代价从 `0.020000` 降到
`0.000154`，4 次迭代收敛。接入地图重建后的回归从 101 个关键帧启动，
在前端继续运行期间追到 103 帧，并在 `580.8 ms` 内分批完成地图替换；
期间没有再次出现激光对应里程计样本过旧的警告。停止后自研位置与
Gazebo 真值的平面位置差约 `0.003 m`。

后台回环、事务合并和分块地图整改后，使用默认参数完成 12.57 m 两圈
长闭环回归：共建立 210 个关键帧，接受 4 次回环并完成 4 次地图重建。
换用扫描法向可观测性检测后再次回归，后台任务完成延迟为
`44.5～45.5 ms`，最大前端消息间隔保持 `0.100 s`。停止后匹配位姿相对
真值误差为 `0.0100 m / 0.415°`；209 次真实匹配全部为 rank 2，零退化
误报，全部自动判定通过。
重建后的 `/custom_slam/map` 也已通过 Map Saver 保存为
`242×202 @ 0.05 m` 的 YAML/PGM 地图。

回环退化保护启用后再次运行同一路线，仍建立 210 个关键帧、接受并重建
4 次正常回环；最终验收误差为 `0.0161 m / 0.333°`，前端最大间隔仍为
`0.100 s`。回环线程使用关键帧采集时的单帧法向可观测性，不会在历史
子图拼接点之间估计法向；rank 小于 2 的候选直接拒绝，不以各向同性
`0.05 m` 标准差写入位姿图。

位姿图内部不再存两个标量权重，而是为每条约束保存对称 3×3 信息矩阵。
Ceres 残差通过 Cholesky 因子白化；默认保守拒绝退化回环，若研究时显式
关闭 `reject_degenerate_matches`，处理器会把单帧弱方向旋转到候选关键帧
坐标系，并按法向信息比降低该方向的约束强度。

同日快速旋转回归三档全部通过，共新增 547 个关键帧。最大前端消息间隔
均为 `0.100 s`，没有匹配拒绝、误回环或退化误报；三档真实匹配数分别为
210、209 和 128，匹配闭合误差分别为 `0.0207 m / 0.216°`、
`0.0222 m / 0.018°` 和 `0.0071 m / 0.108°`。

运行单向退化长走廊回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup corridor_slam_regression.launch.py

# 终端 2
ros2 run slam_robot_slam corridor_regression

# 在同一场景执行 25 m 出走、原地掉头和 25 m 返回
ros2 run slam_robot_slam corridor_regression --return-trip

# 覆盖研究模式：允许退化回环，并验证实际写入各向异性信息矩阵
ros2 launch slam_robot_bringup corridor_slam_regression.launch.py \
  reject_degenerate_loop_closures:=false
ros2 run slam_robot_slam corridor_regression --return-trip \
  --expect-anisotropic-loop-closure
```

工具以 `0.40 m/s` 行驶约 25 m，连续测量纵向、横向和航向误差，并检查
前端消息间隔、拒配、错误回环、关键帧数量、地图覆盖范围、关键日志及
真实匹配尝试的诊断计数是否自洽。`matcher_diagnostics` 将扫描分类为初始化、
点数不足、运动不足、匹配接受或匹配拒绝；只有后两类才计入匹配次数和
rank 直方图。真实匹配还报告两个有效法向特征值、法向数量、比值、弱
方向角、是否执行退化过滤、实际连续修正比例及累计计数。rank 1 的修正
比例同时受信息特征值比和绝对有效法向数约束，因此不会出现 rank 1 但
修正比例仍为 1 的状态。回归工具据此
打印信息比值与弱方向角直方图，
不再从可能重复发布的里程计协方差反推退化检测次数。
拒配或运动量低于匹配阈值时，输出协方差会按相邻里程计增量继续增长；
成功匹配后才重置为本次激光观测协方差，因此纯航位推算不再复用陈旧值。
走廊中段还会按真值位置划出不含入口/出口锚点的退化区间，要求其中至少
100 次真实匹配且 rank 1 检出比例不低于 90%。真值只用于回归分段。

完全平行墙在行进方向上没有几何约束，前端检测到一维退化后保留该方向的
轮式里程计预测；横向与航向仍由激光相关匹配校正。法向只在原始光束索引
连续、相邻距离满足量程自适应门限的局部窗口内拟合，不会跨过被量程过滤
的光束、门口或物体边缘。弱方向实际修正比例为
`max(配置下限, min(1, information_ratio / 0.05,
minimum_information / 5.0))`，避免单阈值硬开关在临界区反复推拉位姿。

往返模式额外验证返回位置闭合、强/弱观测协方差切换、真实回环接受和
地图重建完成。测试驾驶器使用 Gazebo 真值维持目标航向，以隔离差速底盘
在长距离反向行驶时的物理偏航；真值仍只存在于回归工具中，不会发布给
SLAM 节点或参与位姿估计。

2026-08-03 可观测性处理前的纵向峰值为 `0.855 m`。启用处理后再次使用
默认参数回归：真值行驶 `25.104 m`，建立 315 个关键帧，纵向峰值降至
`0.165 m`、最终误差 `0.140 m`，横向峰值 `0.026 m`、航向峰值约
`0.001°`；当时记录的 392 个样本是带各向异性协方差的激光里程计消息，
其中可能包含运动不足或拒配路径重复发布的旧协方差，不能解释为 392 次
独立退化检测，也不能用于推导检出率。前端最大间隔保持 `0.100 s`，无
拒配、无错误回环，地图范围为 `33.10 × 2.95 m`。后续回归以逐次匹配
诊断为准。

最终 50 m 往返回归通过：两段真值行程分别为 `25.002 m / 25.005 m`，
返回位置距起点约 `0.034 m`；建立 670 个关键帧。物理中段 375 次真实
匹配全部为 rank 1；默认回环保护仅在 rank 2 锚点区接受 3 条真实回环，
并完成 3 次地图重建。匹配器全程纵向/横向/航向峰值为
`0.206 m / 0.061 m / 0.200°`，最终相对真值误差为
`0.098 m / 0.024 m / 0.091°`。前端最大间隔 `0.100 s`，无匹配拒绝或
严重日志，地图范围为 `33.10 × 3.00 m`。

运行重复结构假回环保护回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup repeated_structure_slam_regression.launch.py

# 终端 2
ros2 run slam_robot_slam repeated_structure_regression
```

专用世界包含两间相距 8 m、主体几何重复的房间。工具先从房间 A 驶入
房间 B，此阶段任何回环都属于地点混淆并会直接失败；掉头返回 A 后才要求
接受真实回环。工具保存位姿图关键帧时间戳和 50 Hz Gazebo 真值历史，
对每条已接受回环插值出两端真值位置，默认要求距离不超过 `0.80 m`。

2026-08-03 默认参数回归通过：出走阶段零回环，返回阶段接受 4 条回环，
其真值距离分别为 `0.048 / 0.048 / 0.013 / 0.048 m`，无假回环且完成
4 次地图重建。309 个关键帧下最终匹配误差为 `0.009 m / 0.044°`，
全程峰值 `0.026 m / 0.263°`，最大前端间隔 `0.100 s`，地图范围
`13.20 × 5.15 m`。

运行大场景长时间回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup large_scale_slam_regression.launch.py

# 终端 2
ros2 run slam_robot_slam large_scale_regression
```

工具默认在 `26 × 20 m` 仓储世界行驶两圈并返回起点，自动检查位姿图
超过 2000 节点后的 Path 裁剪、多次后台回环、重建队列追赶、地图范围、
前端间隔、误差和关键日志。它还从 Linux `/proc` 读取
`scan_matcher_odometry_node` 的 CPU 时间及 RSS；这些资源指标不包含
Gazebo、RViz 或消息桥。真值仅用于测试驾驶与评估，不进入 SLAM。

2026-08-03 默认参数完整回归通过：真值行程 `155.2 m`，仿真/墙钟均为
`499.1 s`；节流日志观测到 2409 个位姿图节点，接受 42 条回环并在工具
结束前完成 30 次地图重建。匹配最终误差为 `0.112 m / 0.016°`，全程
峰值 `0.170 m / 1.104°`，最大前端间隔 `0.100 s`。回环 worker 最大
`83.7 ms`，地图重建最大 `12.12 s`；SLAM 进程平均/峰值 CPU 为
`10.3% / 19.0%`，RSS 从 `38.6 MiB` 增至峰值 `78.3 MiB`，实时率
`1.000`。位姿图/激光 Path 峰值为 `2000 / 4746`，地图范围
`26.20 × 20.20 m`，无拒配或严重日志。

录制固定大场景参考包：

```bash
# 终端 1
ros2 launch slam_robot_bringup large_scale_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/large_scale_reference"

# 终端 2
ros2 run slam_robot_slam large_scale_regression
```

专用入口只记录 `/clock`、`/scan`、`/odom`、真值、机器人描述和
`/tf_static`，不会记录自研输出或包含旧 `map -> odom` 的动态 `/tf`。
路线通过后在录制终端按一次 `Ctrl+C`，等待 MCAP 完成刷盘。

运行固定包离线回归：

```bash
# 终端 1：必须先启动判定器
ros2 run slam_robot_slam large_scale_bag_regression

# 终端 2
ros2 launch slam_robot_slam play_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/large_scale_reference" \
  rate:=2.0 use_rviz:=false
```

2026-08-03 的 543.352 s 参考包在 271.7 s 墙钟内完成 2× 回放；5434 帧
扫描产生 5433 帧匹配，当前算法重新检测 42 条回环并完成 30 次地图重建。
最终误差 `0.120 m / 0.063°`，峰值 `0.230 m / 0.619°`，最大前端间隔
`0.100 s`，无拒配或严重日志。数据包指纹和完整复现说明见
[docs/datasets.md](../../docs/datasets.md)。

重建期间再次收到回环时，当前重建不会归零；节点会冻结当前版本并将
最新优化快照排队。当前版本完成后再处理最新版本，因此长时间运行也会
持续产出完整地图。地图与 Path 定时器使用仿真时钟；两条 Path 以 2 Hz
发布并按移动阈值降采样，避免消息大小和带宽随每帧激光无界增长。

激光时间戳落在两个轮式里程计样本之间时，前端会对平移和最短角距离
进行插值；只有缺少双边样本时才在 `maximum_odom_age` 内使用最近样本。
自研激光里程计消息显式发布平面位姿协方差；检测到一维退化时，协方差
椭圆沿弱观测方向增大；拒配和运动不足期间则按相邻里程计增量累计过程
噪声。未估计的三维自由度和速度标为高不确定度。当前仓库没有把
`/custom_slam/laser_odom` 作为 EKF 输入，这份各向异性协方差用于诊断、
回归和后续融合接口，不改变现有 `map -> odom` 的发布职责。

录制算法输入和真值：

```bash
ros2 launch slam_robot_slam record_slam_data.launch.py
```

默认记录 `/clock`、`/scan`、`/odom`、`/ground_truth/odom`、`/tf`、`/tf_static` 和 `/robot_description`。`/ground_truth/odom` 只用于评估，禁止反馈给估计器。

离线回放并同时启动预处理节点和 RViz：

```bash
ros2 launch slam_robot_slam play_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/first_run"
```

不需要 RViz 时传入 `use_rviz:=false`；`rate` 控制播放倍率，`loop:=true` 可以循环播放。

运行测试：

```bash
colcon build --packages-select slam_robot_slam --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select slam_robot_slam
colcon test-result --verbose
```

`slam_robot_slam` 在未显式指定构建类型时默认使用 `RelWithDebInfo`
（`-O2 -g -DNDEBUG`）。扫描匹配和射线积分属于计算密集型代码，不应使用
无优化的默认编译配置运行性能回归。

运行节点会在入口拒绝角度/量程元数据非法的 LaserScan、非有限里程计以及
非法 TF，不让坏数据进入相关栅格或位姿图。参数错误会输出 `FATAL` 原因并
以非零状态退出；单帧数据处理错误只丢弃该帧并输出节流日志。
