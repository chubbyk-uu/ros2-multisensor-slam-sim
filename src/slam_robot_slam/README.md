# slam_robot_slam

本包保存 `slam_toolbox` 的在线异步建图配置和启动文件，之后逐步加入自研 C++ 2D SLAM 节点。

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
- 关键帧旋转阈值：0.10 rad。
- 启用扫描匹配和回环检测。
- 关闭交互式位姿图编辑，减少不必要的后台开销。

建图结束后，同时保存导航占据栅格和可恢复的 SLAM Toolbox 位姿图：

```bash
ros2 run slam_robot_slam save_slam_map \
  /home/jerry/robot_ws/slam/maps/slam_map
```

成功时会生成 `.yaml`、`.pgm`、`.posegraph` 和 `.data` 四个文件。确认这些文件存在后再退出建图进程；`Ctrl+C` 本身不会触发保存。
