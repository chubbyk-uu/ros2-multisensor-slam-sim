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

当前已实现激光预处理和局部扫描匹配前端：

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
- 真值、轮式里程计和匹配轨迹自动对比工具。

仿真已经运行时，可独立启动预处理节点：

```bash
ros2 launch slam_robot_slam custom_slam.launch.py
```

一条命令启动完整开发环境：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

专用 RViz 使用 `odom` 固定坐标系：红色为原始 `/scan`，绿色为预处理点集，青色为局部子图匹配后的扫描，黄色为匹配轨迹。当前阶段不会发布 `/map` 或 `map -> odom`，不会与 SLAM Toolbox 抢占 TF。

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
  -> 激光轨迹
```

核心输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/custom_slam/laser_odom` | `nav_msgs/Odometry` | 匹配前端估计位姿 |
| `/custom_slam/laser_path` | `nav_msgs/Path` | 匹配轨迹 |
| `/custom_slam/aligned_scan_points` | `sensor_msgs/PointCloud2` | 对齐到 `odom` 的当前扫描 |

运行固定路线真值测试：

```bash
ros2 run slam_robot_slam scan_matcher_benchmark
```

测试工具只读取 `/ground_truth/odom` 计算误差，不会把真值反馈给匹配器。
目前仍缺少全局占据栅格、位姿图优化和回环检测，因此这是 SLAM
前端，不是完整的自研 SLAM。

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
