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

通过本包的 `mapping.launch.py` 或项目统一建图 launch 启动时，自动保存默认开启。按一次 `Ctrl+C` 后，会在 SLAM Toolbox 退出前保存导航占据栅格和可恢复的位姿图。默认前缀是启动命令当前目录下的 `maps/slam_map`。

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

当前已实现激光预处理、局部扫描匹配、基础占据栅格和位姿图骨架：

- LaserScan 有效量程、`NaN` 和 `Inf` 过滤。
- 可配置的点间隔降采样。
- 极坐标到二维笛卡尔点集转换。
- `/custom_slam/scan_points` PointCloud2 发布。
- Gazebo `/ground_truth/odom` 真值评估基准。
- MCAP rosbag 数据集录制入口。
- 独立点到线 ICP 对照实现及单元测试。
- 轮式里程计位姿预测和移动阈值过滤。
- Karto 风格的相关栅格粗到细搜索。
- 当前扫描到最近 20 个关键帧局部子图的匹配。
- 匹配分数、最少重合点和失败回退机制。
- 基于关键帧的射线清空、末端占用和 log-odds 概率更新。
- 自动扩展的 0.05 m 分辨率占据栅格。
- 保存全部成功关键帧，最近 20 帧只用于局部匹配。
- 使用 Ceres 2.2 建立二维位姿图、顺序约束和可带 Huber 核的回环约束。
- 真值、轮式里程计和匹配轨迹自动对比工具。

仿真已经运行时，可独立启动预处理节点：

```bash
ros2 launch slam_robot_slam custom_slam.launch.py
```

一条命令启动完整开发环境：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

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
| `map.hit_probability` | `0.70` | 命中单元的概率更新 |
| `map.miss_probability` | `0.40` | 射线穿过单元的空闲概率更新 |
| `map.minimum_probability` | `0.12` | log-odds 累积下限 |
| `map.maximum_probability` | `0.97` | log-odds 累积上限 |
| `pose_graph.sequential_translation_stddev` | `0.05` | 顺序边平移标准差，单位 m |
| `pose_graph.sequential_rotation_stddev` | `0.05` | 顺序边旋转标准差，单位 rad |

运行固定路线真值测试：

```bash
ros2 run slam_robot_slam scan_matcher_benchmark
```

测试工具只读取 `/ground_truth/odom` 计算误差，不会把真值反馈给匹配器。
当前前端已持续建立关键帧节点和顺序边，Ceres 后端也已通过合成回环
测试；但尚未自动搜索并验证真实回环，历史关键帧还不能随优化结果重投影
并重建地图，因此仍不是完整的自研 SLAM。

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
colcon test --packages-select slam_robot_slam
colcon test-result --verbose
```
