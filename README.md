# slam_robot

基于 ROS 2 Jazzy 和 Gazebo Sim 的轮式机器人多传感器 SLAM 仿真项目。
当前开发阶段优先完成差速机器人模型和 2D 激光 SLAM。

## 构建

```bash
cd /home/jerry/robot_ws/slam
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 显示机器人模型

首次使用 GUI 关节调节工具前安装：

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

然后启动：

```bash
ros2 launch slam_robot_description display.launch.py
```

启动后，RViz 的固定坐标系应为 `base_footprint`，并能看到机器人模型和完整 TF。
无桌面环境或未安装 GUI 组件时可使用：

```bash
ros2 launch slam_robot_description display.launch.py use_gui:=false
```

## 模型验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/slam_robot_description/urdf/slam_robot.urdf.xacro \
  -o /tmp/slam_robot.urdf
check_urdf /tmp/slam_robot.urdf
```

校验输出应包含 `Successfully Parsed XML`，根坐标系应为 `base_footprint`。

## 启动 Gazebo 仿真

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_robot_gazebo simulation.launch.py
```

默认会同时打开 Gazebo 和 RViz。RViz 使用 `odom` 作为固定坐标系，并显示机器人、TF 和 `/scan` 激光点。

无图形界面时：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py gui:=false rviz:=false
```

在 WSL2 中，launch 默认设置 Mesa D3D12 渲染后端并选择 NVIDIA 适配器，避免 Gazebo 和 RViz 回退到 `llvmpipe` 软件渲染。AMD 或 Intel 显卡可改为：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py \
  wsl_gpu_adapter:=AMD
```

如果本机环境不需要该设置，可传入 `use_wsl_gpu:=false`。只需要 RViz 观察建图、不需要 Gazebo 三维窗口时，推荐关闭 Gazebo GUI：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  gui:=false use_rviz:=true
```

仿真启动后会生成 `slam_robot`，并建立以下 ROS 2 接口：

- `/clock`：Gazebo 仿真时间。
- `/cmd_vel`：差速底盘速度指令。
- `/odom`：轮式里程计，坐标关系为 `odom -> base_footprint`。
- `/joint_states`：左右驱动轮关节状态。
- `/scan`：2D 激光扫描。
- `/tf` 和 `/tf_static`：里程计及机器人内部坐标变换。

默认世界是一个 12 m × 10 m 的非对称室内场景，包含外围墙、分隔墙、箱体和圆柱路标，并保留了可绕行的回环通道。

机器人左右轮的几何中心距为 `0.34 m`。Gazebo ODE 中有限宽圆柱轮的有效接触轮距经原地旋转标定为 `0.306 m`，该数值仅用于差速运动学和里程计，机器人几何及 `base_footprint` 位置不变。

2D LiDAR 初始参数：

| 参数 | 数值 |
| --- | --- |
| 扫描范围 | -π～π（360°） |
| 样本数 | 720 |
| 更新频率 | 10 Hz |
| 最小距离 | 0.12 m |
| 最大距离 | 12.0 m |
| 高斯噪声标准差 | 0.005 m |
| 坐标系 | `lidar_link` |

检查激光数据：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

## 启动 2D SLAM 建图

一条命令启动 Gazebo、机器人、LiDAR、SLAM Toolbox 和建图 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

无界面验证：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  gui:=false use_rviz:=false
```

另开终端使用键盘控制机器人：

```bash
source /opt/ros/jazzy/setup.bash
source /home/jerry/robot_ws/slam/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

同一时间只运行一个 `simulation.launch.py` 或 `mapping_simulation.launch.py`。重复启动会产生多个 `/clock`、`/odom` 和 TF 数据源，导致时间戳错误和激光错位。

建图时应能观察到：

- `/map` 以约 0.5 Hz 更新。
- `slam_toolbox` 生命周期状态为 `active`。
- TF 树增加 `map -> odom`。
- 机器人移动后，地图的已知区域逐渐扩展。

检查命令：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

SLAM 参数位于：

```text
src/slam_robot_slam/config/mapper_params_online_async.yaml
```

当前性能配置使用 250 Hz 物理循环、10 Hz 激光、约 28 Hz 里程计和 2 s 地图更新周期。Gazebo 内不绘制激光射线，激光仍可在 RViz 中查看；RViz 的 TF 显示默认关闭，需要排查坐标系时可手动打开。

为减小起转和停转时激光相对地图的瞬时偏移，旋转关键帧阈值使用 `0.05 rad`。小角度转弯时仍可能看到几厘米的短暂偏差；如果停稳后能够重新对齐且地图没有持续重影，属于 10 Hz 激光和 0.05 m 栅格分辨率下的正常范围。

### 性能与旋转回归记录

2026-07-29 在 WSL2、RTX 5080 环境实测：

| 指标 | 优化前 | 优化后 |
| --- | ---: | ---: |
| Gazebo GUI CPU | 约 529% | 约 132% |
| Gazebo Server CPU | 约 58% | 约 31% |
| RViz CPU | 约 93% | 约 10% |
| 仿真实时率 | 约 1.0 | 稳定 1.0 |

Linux 中单个进程的 CPU 百分比可以超过 100%，表示占用了多个逻辑核心。以上数值只用于同机前后对比，不代表其他机器的固定性能。

优化后进行约 127° 原地旋转：轮式里程计和 SLAM 相对 Gazebo 真值的航向误差分别约为 0.31° 和 0.29°，SLAM 平移偏差约 1.5 mm。

短时发送直行指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}" -r 10
```

结束速度发布后，应再发送一次零速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## 保存地图

`mapping_simulation.launch.py` 默认启用自动保存。建图完成后，在运行 launch 的终端按一次 `Ctrl+C`，系统会先保存地图，再关闭 SLAM 和仿真。终端应显示：

```text
[auto_save_map] Saving map to: .../maps/slam_map
Map saved with prefix: .../maps/slam_map
[auto_save_map] Save completed; shutting down.
```

默认以启动命令所在目录下的 `maps/slam_map` 为文件前缀，生成：

- `slam_map.yaml` 和 `slam_map.pgm`：供 Nav2 Map Server 和 AMCL 使用。
- `slam_map.posegraph` 和 `slam_map.data`：供 SLAM Toolbox 恢复位姿图和继续建图。

等待看到 `Save completed` 后 launch 才会继续退出。不要连续多次按 `Ctrl+C`，否则可能中断正在进行的写盘。

使用自定义保存位置：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  map_output_prefix:=/home/jerry/robot_ws/slam/maps/room_01
```

确认默认地图文件：

```bash
ls -lh /home/jerry/robot_ws/slam/maps/slam_map.*
```

如果临时不希望退出时保存：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  auto_save_map:=false
```

原来的手动命令仍可在建图过程中用于保存检查点：

```bash
ros2 run slam_robot_slam save_slam_map \
  /home/jerry/robot_ws/slam/maps/checkpoint
```

## 使用保存地图进行定位与导航

确认 `maps/slam_map.yaml` 和 `maps/slam_map.pgm` 已存在，然后用一条命令启动 Gazebo、机器人、Map Server、AMCL、完整 Nav2 和官方 Nav2 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

机器人默认出生在建图原点，因此 AMCL 会自动使用 `(x, y, yaw) = (0, 0, 0)` 初始化。RViz 中激光应与保存地图重合；点击顶部的 `Nav2 Goal`（或 `2D Goal Pose`），再在地图空闲区域拖出目标朝向，机器人即可自主规划和行驶。

如果出生点不是地图原点，可以在 RViz 点击 `2D Pose Estimate` 重新给定位姿，也可以在启动时指定：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  initial_pose_x:=1.0 initial_pose_y:=-0.5 initial_pose_yaw:=1.57
```

无界面验证：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  gui:=false use_rviz:=false
```

导航基于 ROS 2 Jazzy 安装的 Nav2 官方参数，保留 MPPI Controller、Navfn Planner、Simple Smoother、Behavior Server、Velocity Smoother、Collision Monitor、Waypoint Follower、Route Server 和 Docking Server。项目只覆盖机器人相关的坐标系、真实多边形 footprint、激光量程、初始位姿和运动约束。

开始导航前请退出键盘遥控节点，并且不要同时运行建图 launch。多个节点直接向 `/cmd_vel` 发布速度会互相覆盖，造成导航轨迹异常。

检查定位与导航状态：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
```

以上生命周期节点应全部为 `active [3]`。官方默认的到点位置容差为 `0.25 m`，因此机器人进入目标附近后返回成功是正常行为。

完整仿真启动后，可以分别执行多目标和动态障碍物自动回归：

```bash
ros2 run slam_robot_navigation navigation_regression.py
ros2 run slam_robot_navigation navigation_regression.py \
  --scenario dynamic-obstacle
```

动态测试会在机器人行驶后生成地图中不存在的箱体，检查局部、全局代价
地图是否正确标记障碍，并在测试结束时自动移除箱体。

## 当前包

- `slam_robot_description`：机器人 Xacro、RViz 配置和模型显示。
- `slam_robot_gazebo`：Gazebo 世界、系统插件和消息桥接。
- `slam_robot_bringup`：统一启动入口。
- `slam_robot_slam`：2D SLAM 配置以及后续 C++ 算法节点。
- `slam_robot_navigation`：官方 Nav2 基线、Map Server、AMCL 定位和自主导航入口。

详细实施路线见 [plan.md](plan.md)。
