# 2D 建图、导航与自研 SLAM

本文汇总 2D LiDAR 的日常操作。环境安装和构建见
[安装与首次运行](getting-started.md)，TF 与节点职责见
[系统架构](architecture.md)。

## 键盘驾驶

仿真启动后，在另一个已加载工作区的终端运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

使用 Nav2 前应退出键盘遥控，避免两个节点同时向 `/cmd_vel` 发布速度。

## SLAM Toolbox 建图

一条命令启动 Gazebo、默认轮速 + IMU EKF、SLAM Toolbox 和建图 RViz：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

需要纯轮式里程计对照时：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  odometry_mode:=wheel
```

建议先直行检查扫描与墙体对齐，再低速转弯，最后绕场景形成闭环。运行中可
检查：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

### 自动保存地图

建图完成后，在 launch 终端按一次 `Ctrl+C`。关机钩子会先保存再退出，请等待
出现 `Save completed`，不要连续按多次 `Ctrl+C`。保存阶段会同步等待服务和
磁盘写入，异常时最多约 45 秒后放弃保存并继续关闭 SLAM 进程。

默认生成：

- `maps/slam_map.yaml`、`maps/slam_map.pgm`：Map Server 和 AMCL 使用。
- `maps/slam_map.posegraph`、`maps/slam_map.data`：恢复 SLAM Toolbox 位姿图。

`maps/` 根目录下的地图都是每次运行的产物，不进入版本控制，因此自动保存
不会弄脏工作区。随仓库分发的演示地图单独放在 `maps/reference/`，说明见
[地图目录](../maps/README.md)。

指定输出前缀：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  map_output_prefix:="${SLAM_WS}/maps/room_01"
```

运行中手动保存检查点：

```bash
ros2 run slam_robot_slam save_slam_map \
  "${SLAM_WS}/maps/checkpoint"
```

临时关闭退出自动保存：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  auto_save_map:=false
```

## AMCL 与 Nav2 导航

确认 `maps/slam_map.yaml` 和 `maps/slam_map.pgm` 存在，然后启动：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

还没有自己建过图时，可以先用演示地图跑通链路：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  map:="${SLAM_WS}/maps/reference/slam_map.yaml"
```

机器人默认从建图原点出生，AMCL 使用 `(0, 0, 0)` 初始化。在 RViz 点击
`Nav2 Goal` 并拖出朝向即可导航。出生点改变时使用 `2D Pose Estimate`，或：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  initial_pose_x:=1.0 initial_pose_y:=-0.5 initial_pose_yaw:=1.57
```

无界面运行：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  gui:=false use_rviz:=false
```

导航回归与动态障碍物测试：

```bash
ros2 run slam_robot_navigation navigation_regression.py
ros2 run slam_robot_navigation navigation_regression.py \
  --scenario dynamic-obstacle
```

测试检查目标结果、恢复次数、AMCL 终点误差和代价地图标记。动态场景结束后
会自动移除测试箱体。完整参数见
[导航包说明](../src/slam_robot_navigation/README.md)。

## 自研 C++ 2D SLAM

启动 Gazebo、自研前后端和专用 RViz：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

该入口不启动 SLAM Toolbox。主要输出为：

| 名称 | 说明 |
| --- | --- |
| `/custom_slam/map` | 自研 log-odds 占据栅格 |
| `/custom_slam/laser_odom` | 激光匹配位姿与各向异性协方差 |
| `/custom_slam/laser_path` | 限频、限长的匹配轨迹 |
| `/custom_slam/aligned_scan_points` | 局部子图匹配后的扫描 |
| `/custom_slam/pose_graph_path` | 优化后的关键帧路径 |

RViz 中红色为原始扫描，绿色为预处理点，青色为匹配后的扫描，黄色为匹配
轨迹，紫色为位姿图路径。真值只用于回归评分，不进入算法。

### 保存自研地图并交给 Nav2

自研入口默认在收到一次 `Ctrl+C` 后，将 `/custom_slam/map` 保存为：

- `maps/custom_slam_map.yaml`
- `maps/custom_slam_map.pgm`

等待终端显示 `[custom_auto_save_map] Save completed` 后再启动导航。自研 SLAM
目前不序列化位姿图，因此不会生成 SLAM Toolbox 使用的 `.posegraph` 和
`.data`。保存器异常时最多等待 30 秒，随后仍会关闭自研 SLAM 节点，避免留下
继续发布 `/custom_slam/map` 的孤儿进程。指定其他输出前缀：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py \
  map_output_prefix:="${SLAM_WS}/maps/custom_room_01"
```

关闭自动保存：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py \
  auto_save_map:=false
```

使用保存的自研地图启动 AMCL 与 Nav2：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  map:="${SLAM_WS}/maps/custom_slam_map.yaml"
```

自研 SLAM 的演示地图同样随仓库分发，位于
`maps/reference/custom_slam_map.yaml`。

导航前必须先关闭自研 SLAM，避免它和 AMCL 同时发布 `map -> odom`。

当前实现包括轮式预测、相关扫描匹配、法向几何退化检测、关键帧局部子图、
后台回环匹配、各向异性位姿图约束、Ceres 优化和优化后射线重放。算法结构、
参数、话题与已知边界见
[自研 SLAM 包说明](../src/slam_robot_slam/README.md)。

## 自研 SLAM 回归入口

| 场景 | 仿真启动 | 判定器 |
| --- | --- | --- |
| 基础路线 | `custom_slam_development.launch.py` | `scan_matcher_benchmark` |
| 退化长走廊 | `corridor_slam_regression.launch.py` | `corridor_regression` |
| 重复结构 | `repeated_structure_slam_regression.launch.py` | `repeated_structure_regression` |
| 快速旋转 | `custom_slam_development.launch.py` | `rotation_regression` |
| 155 m 大场景 | `large_scale_slam_regression.launch.py` | `large_scale_regression` |

典型双终端回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup large_scale_slam_regression.launch.py

# 终端 2
ros2 run slam_robot_slam large_scale_regression
```

固定数据集录制、SHA-256 和 2× 离线回放见
[固定 SLAM 数据集](datasets.md)。性能、精度和标定结果见
[性能与标定](performance.md)，工程审查整改见
[审查整改记录](archive/2026-07-30-review-remediation.md)。

## 普通 rosbag 录制与回放

录制调试数据：

```bash
ros2 launch slam_robot_slam record_slam_data.launch.py
```

默认写入 `bags/slam_data_YYYYMMDD_HHMMSS/`。指定目录：

```bash
ros2 launch slam_robot_slam record_slam_data.launch.py \
  output:="${SLAM_WS}/bags/first_run"
```

不启动 Gazebo，直接回放并运行预处理节点：

```bash
ros2 launch slam_robot_slam play_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/first_run"
```

`/ground_truth/odom` 可以被录制用于离线评估，但禁止作为 SLAM 输入。
