# slam_robot_bringup

本包提供机器人仿真与 SLAM 的统一启动入口。

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
