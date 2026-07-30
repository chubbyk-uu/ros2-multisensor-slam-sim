# slam_robot_bringup

本包提供机器人仿真、SLAM 和导航的统一启动入口。

启动完整 2D 建图系统：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

该 launch 会启动 Gazebo、机器人、ROS-Gazebo bridge、SLAM Toolbox 和建图 RViz。

默认只打开一个使用 `map` 固定坐标系的建图 RViz。无界面运行时传入：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py \
  gui:=false use_rviz:=false
```

使用已保存地图启动 AMCL 和完整 Nav2：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

默认地图为启动目录下的 `maps/slam_map.yaml`。该入口会自动在地图原点初始化 AMCL，并打开官方 Nav2 RViz；发送导航目标前应退出键盘遥控节点。

启动自研 2D SLAM 开发环境：

```bash
ros2 launch slam_robot_bringup custom_slam_development.launch.py
```

当前入口启动 Gazebo、C++ 激光预处理节点和专用 RViz，不启动 SLAM Toolbox，也不发布 `/map` 或 `map -> odom`。
