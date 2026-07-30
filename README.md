# ROS 2 Multi-Sensor SLAM Simulation

基于 ROS 2 Jazzy 和 Gazebo Sim 的差速轮式机器人多传感器 SLAM 仿真项目。当前已完成 2D LiDAR 仿真、SLAM Toolbox 建图、地图自动保存、AMCL 定位和 Nav2 自主导航；后续将继续实现自研 2D SLAM，并扩展 3D LiDAR、视觉和多传感器融合。

## 运行效果

### Gazebo 仿真环境

![Gazebo 中的差速机器人和室内 SLAM 测试环境](docs/images/gazebo-simulation.png)

### AMCL 定位与 Nav2 导航

![RViz 中的地图、激光、代价地图和导航状态](docs/images/nav2-navigation.png)

## 当前进度

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 差速轮式机器人模型 | 已完成 | Xacro、惯性、碰撞体、驱动轮、万向轮和 LiDAR 安装结构 |
| Gazebo Sim 仿真 | 已完成 | 非对称室内场景、差速驱动和 `ros_gz` 消息桥接 |
| 2D LiDAR | 已完成 | 360°、720 点、10 Hz、0.12～12 m |
| 2D 激光建图 | 已完成 | SLAM Toolbox `online_async`、回环检测和退出时自动保存 |
| 定位与导航 | 已完成 | Map Server、AMCL、Nav2 官方完整组件和 RViz |
| 导航自动回归 | 已完成 | 多目标导航与动态障碍物重规划 |
| 自研 C++ 2D SLAM | 进行中 | 已完成真值基准、数据集、预处理和局部相关扫描匹配前端 |
| 3D LiDAR / 视觉 / 融合 | 计划中 | 在 2D 基线稳定后逐步接入 |

详细开发路线见 [plan.md](plan.md)，性能和旋转标定结果见 [docs/performance.md](docs/performance.md)。

## 环境与依赖

- Ubuntu 24.04 或对应的 WSL2 环境
- ROS 2 Jazzy
- Gazebo Sim 8（ROS 2 Jazzy 对应的 Gazebo Harmonic）
- `ros_gz`
- SLAM Toolbox
- Navigation2
- Xacro、`robot_state_publisher` 和 RViz

假设 ROS 2 Jazzy 已正确安装，克隆项目后执行：

```bash
git clone https://github.com/chubbyk-uu/ros2-multisensor-slam-sim.git
cd ros2-multisensor-slam-sim

export SLAM_WS="$PWD"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

首次使用模型关节 GUI 时，如系统尚未安装，可执行：

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

后续命令默认在仓库根目录运行。启动文件会以当前目录为基准查找或保存 `maps/`，因此建议每个新终端先执行：

```bash
cd "$SLAM_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 快速开始

### 1. 查看机器人模型

```bash
ros2 launch slam_robot_description display.launch.py
```

RViz 固定坐标系应为 `base_footprint`，并显示完整机器人和 TF。没有 GUI 关节工具时可用：

```bash
ros2 launch slam_robot_description display.launch.py use_gui:=false
```

模型语法校验：

```bash
xacro src/slam_robot_description/urdf/slam_robot.urdf.xacro \
  -o /tmp/slam_robot.urdf
check_urdf /tmp/slam_robot.urdf
```

### 2. 启动基础仿真

```bash
ros2 launch slam_robot_gazebo simulation.launch.py
```

默认同时打开 Gazebo 和 RViz。无图形界面时：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py \
  gui:=false rviz:=false
```

基础接口：

| 名称 | 用途 |
| --- | --- |
| `/clock` | Gazebo 仿真时间 |
| `/cmd_vel` | 差速底盘速度指令 |
| `/odom` | 轮式里程计 |
| `/ground_truth/odom` | Gazebo 无噪声真值，仅用于算法评估 |
| `/joint_states` | 轮子关节状态 |
| `/scan` | 2D 激光扫描 |
| `/tf`、`/tf_static` | 动态与静态坐标变换 |

检查 LiDAR 和 TF：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

### 3. 建图

一条命令启动 Gazebo、机器人、SLAM Toolbox 和建图 RViz：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

另开终端遥控：

```bash
cd "$SLAM_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

建议先直行观察地图对齐，再以较低角速度转弯，最后绕场景形成闭环。建图期间可检查：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

同一时间只能运行一个仿真或建图入口。多个 `/clock`、`/odom` 或 TF 发布源会导致时间戳异常和激光错位。

### 4. 保存地图

建图完成后，在运行 launch 的终端按一次 `Ctrl+C`。默认会先保存地图和位姿图，再关闭节点。请等待终端出现：

```text
[auto_save_map] Saving map to: .../maps/slam_map
Map saved with prefix: .../maps/slam_map
[auto_save_map] Save completed; shutting down.
```

默认生成：

- `maps/slam_map.yaml` 和 `maps/slam_map.pgm`：供 Map Server 和 AMCL 使用。
- `maps/slam_map.posegraph` 和 `maps/slam_map.data`：用于恢复 SLAM Toolbox 位姿图。

不要连续按多次 `Ctrl+C`，否则可能在写盘完成前中断进程。自定义保存位置：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  map_output_prefix:="${SLAM_WS}/maps/room_01"
```

运行中也可以手动保存检查点：

```bash
ros2 run slam_robot_slam save_slam_map \
  "${SLAM_WS}/maps/checkpoint"
```

临时关闭自动保存：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  auto_save_map:=false
```

### 5. 定位与导航

确认 `maps/slam_map.yaml` 和 `maps/slam_map.pgm` 存在，然后启动：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

机器人默认从建图原点出生，AMCL 自动使用 `(x, y, yaw) = (0, 0, 0)` 初始化。在 RViz 点击 `Nav2 Goal`，在空闲区域拖出目标朝向即可规划并行驶。

出生点改变时，可使用 RViz 的 `2D Pose Estimate`，或在启动时指定：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  initial_pose_x:=1.0 initial_pose_y:=-0.5 initial_pose_yaw:=1.57
```

无界面运行：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  gui:=false use_rviz:=false
```

导航前请关闭键盘遥控节点，避免多个节点同时向 `/cmd_vel` 发布速度。

### 6. 导航回归测试

完整导航仿真启动后，另开终端执行多目标测试：

```bash
ros2 run slam_robot_navigation navigation_regression.py
```

动态障碍物重规划测试：

```bash
ros2 run slam_robot_navigation navigation_regression.py \
  --scenario dynamic-obstacle
```

测试会检查导航结果、耗时、恢复次数、AMCL 终点误差和代价地图标记；动态场景结束后会自动移除测试箱体。

### 7. 自研 SLAM 开发入口

启动 Gazebo、C++ 激光预处理节点和专用 RViz：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

这个入口不会启动 SLAM Toolbox，也不会发布 `/map` 或 `map -> odom`。RViz 中红色点为原始 `/scan`，绿色点为预处理后的 `/custom_slam/scan_points`，青色点为局部子图匹配后的扫描，黄色线为自研匹配轨迹。

检查数据：

```bash
ros2 topic hz /custom_slam/scan_points
ros2 topic hz /custom_slam/laser_odom
ros2 topic echo /custom_slam/scan_points --once --field header
ros2 topic echo /ground_truth/odom --once
```

当前预处理节点会：

- 去除 `NaN`、`Inf` 和量程外数据。
- 按参数执行可选角度降采样。
- 将极坐标 LaserScan 转换为 `lidar_link` 下的二维笛卡尔点集。
- 发布标准 `sensor_msgs/PointCloud2`，供扫描匹配和 RViz 使用。

当前扫描匹配前端参考 slam_toolbox/Karto 与 Cartographer 的成熟结构：

- 使用轮式里程计作为搜索中心，不使用 Gazebo 真值。
- 机器人至少移动 `0.05 m` 或转动 `0.05 rad` 后才建立新关键帧。
- 将当前扫描与最近 20 个关键帧组成的局部相关栅格匹配。
- 先在平移和转角窗口内粗搜索，再做小范围精搜索。
- 根据相关分数和重合点数接受或拒绝匹配，失败时回退到里程计预测。
- 发布 `/custom_slam/laser_odom`、`/custom_slam/laser_path` 和
  `/custom_slam/aligned_scan_points`。

自动执行固定路线并对比真值、轮式里程计和匹配轨迹：

```bash
ros2 run slam_robot_slam scan_matcher_benchmark
```

点到线 ICP 作为独立的算法对照和单元测试保留，不是默认运行前端。
当前实现还没有发布占据栅格，也没有位姿图与回环，因此尚不是完整
SLAM。

录制可重复使用的 SLAM 数据集：

```bash
ros2 launch slam_robot_slam record_slam_data.launch.py
```

默认以 MCAP 格式保存到当前仓库的 `bags/slam_data_YYYYMMDD_HHMMSS/`，包括 `/clock`、`/scan`、`/odom`、`/ground_truth/odom`、`/tf`、`/tf_static` 和 `/robot_description`。录制结束时按一次 `Ctrl+C`，然后可用以下命令查看：

```bash
ros2 bag info bags/slam_data_YYYYMMDD_HHMMSS
```

指定输出目录：

```bash
ros2 launch slam_robot_slam record_slam_data.launch.py \
  output:="${SLAM_WS}/bags/first_run"
```

`/ground_truth/odom` 只能用于离线评估，禁止作为自研 SLAM 的输入，否则得到的轨迹误差没有意义。数据集默认被 Git 忽略。

不启动 Gazebo，直接回放数据并运行预处理节点：

```bash
ros2 launch slam_robot_slam play_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/first_run"
```

回放入口默认打开专用 RViz，并在发布数据前等待 2 秒，让节点和订阅关系完成初始化。可通过 `rate:=0.5` 慢速播放，或通过 `loop:=true` 循环播放。

## 系统结构

数据流：

```text
teleop / Nav2 -> /cmd_vel -> Gazebo diff drive -> /odom
Gazebo 2D LiDAR -> /scan
Gazebo world pose -> /ground_truth/odom（仅评估）
robot_state_publisher -> /tf、/tf_static
/scan + TF + /odom -> SLAM Toolbox -> /map、map -> odom
/scan -> custom C++ preprocessing -> /custom_slam/scan_points
/scan + /odom + TF -> local correlative matcher
  -> /custom_slam/laser_odom、/custom_slam/laser_path
saved map + /scan + TF -> AMCL / Nav2
```

TF 树：

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_link
            └── lidar_mount_link
                └── lidar_link
```

`base_footprint` 位于驱动轮轴线中点的地面投影，是差速运动学旋转中心。几何轮距为 `0.34 m`；Gazebo 中按原地旋转标定的有效运动学轮距为 `0.306 m`，仅用于驱动和里程计参数。

## 包结构

- `slam_robot_description`：Xacro、模型资源和模型显示。
- `slam_robot_gazebo`：Gazebo 世界、系统插件和 ROS-Gazebo 桥接。
- `slam_robot_bringup`：建图与导航的统一启动入口。
- `slam_robot_slam`：SLAM Toolbox、自研 C++ SLAM、数据录制和算法测试。
- `slam_robot_navigation`：Map Server、AMCL、Nav2 配置和自动回归工具。

## WSL2 与已知现象

- 在 WSL2 中，launch 默认使用 Mesa D3D12，并选择名称包含 `NVIDIA` 的适配器。AMD 或 Intel 显卡可传入 `wsl_gpu_adapter:=AMD` 或对应名称；非 WSL 环境默认不启用该设置。
- `Anti-aliasing level ... is not supported` 是 Ogre2 将不支持的 FSAA 级别回退到可用值的警告，不影响仿真、传感器或 SLAM。
- 10 Hz LiDAR 在机器人转弯时可能出现几厘米的短暂点云偏移；停稳后能够重新对齐且地图没有持续重影，通常属于扫描周期和 0.05 m 地图分辨率的正常表现。
- Nav2 和 Gazebo 的组合进程在 `Ctrl+C` 退出时可能打印生命周期或强制终止日志；如果运行过程正常且地图保存已完成，不代表导航失败。

## License

本项目采用 [Apache License 2.0](LICENSE)。
