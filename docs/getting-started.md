# 安装与首次运行

本文是项目环境、依赖和构建方式的统一入口。项目面向 Ubuntu 24.04、
ROS 2 Jazzy 和 Gazebo Harmonic（Gazebo Sim 8），也支持安装了对应图形驱动的
WSL2 环境。

## 基础依赖

- ROS 2 Jazzy
- Gazebo Sim 8 与 `ros_gz`
- SLAM Toolbox
- Navigation2
- `robot_localization`
- Xacro、`robot_state_publisher` 和 RViz
- RTAB-Map ROS 2 包
- MOLA LiDAR odometry、ROS 2 bridge 和 metric maps（仅 MOLA 对照需要）

克隆仓库并安装能够由 rosdep 解析的依赖：

```bash
git clone https://github.com/chubbyk-uu/ros2-multisensor-slam-sim.git
cd ros2-multisensor-slam-sim

export SLAM_WS="$PWD"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

安装 3D SLAM 与里程计基线：

```bash
sudo apt install \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-mola-lidar-odometry \
  ros-jazzy-mola-bridge-ros2 \
  ros-jazzy-mola-metric-maps
```

首次使用模型关节 GUI 时，如系统尚未安装：

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

## 构建

```bash
cd "$SLAM_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

每个新终端都应加载 ROS 2 和当前工作区：

```bash
cd "$SLAM_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

构建目录 `build/`、`install/` 和 `log/` 都是生成内容，不应手工修改。

## 首次检查

查看机器人模型：

```bash
ros2 launch slam_robot_description display.launch.py
```

没有关节 GUI 时：

```bash
ros2 launch slam_robot_description display.launch.py use_gui:=false
```

检查 Xacro 与 URDF：

```bash
xacro src/slam_robot_description/urdf/slam_robot.urdf.xacro \
  -o /tmp/slam_robot.urdf
check_urdf /tmp/slam_robot.urdf
```

启动默认 2D 机器人基础仿真：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py
```

无图形界面时：

```bash
ros2 launch slam_robot_gazebo simulation.launch.py \
  gui:=false rviz:=false
```

检查基本话题与 TF：

```bash
ros2 topic hz /scan
ros2 topic echo /imu/data_raw --once
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

## 2D 与 3D 机器人变体

2D 与 3D LiDAR 是同一底盘的互斥配置，不会同时生成。默认基础仿真使用
2D LiDAR；单独查看 3D 点云：

```bash
ros2 launch slam_robot_gazebo lidar_3d_simulation.launch.py
ros2 topic hz /lidar_3d/points
ros2 run tf2_ros tf2_echo base_link lidar_3d_link
```

两种配置都包含固定在底盘内部的 100 Hz IMU。默认局部里程计模式为
`odometry_mode:=wheel_imu`，使用 `robot_localization` 融合轮速的平面线速度
与 IMU 偏航角速度；需要纯轮式对照时传入 `odometry_mode:=wheel`。

## 下一步

- [2D 建图、导航与自研 SLAM](2d-workflows.md)
- [3D 建图、在线导航与验收](3d-workflows.md)
- [系统架构与 TF](architecture.md)
- [常见问题](troubleshooting.md)
