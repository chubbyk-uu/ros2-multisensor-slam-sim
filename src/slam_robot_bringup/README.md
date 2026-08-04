# slam_robot_bringup

本包提供机器人仿真、SLAM 和导航的统一启动入口。

启动完整 2D 建图系统：

```bash
ros2 launch slam_robot_bringup mapping_simulation.launch.py
```

该 launch 会启动 Gazebo、机器人、ROS-Gazebo bridge、默认的轮速 + IMU
二维 EKF、SLAM Toolbox 和建图 RViz。EKF 唯一发布 `/odom` 和
`odom -> base_footprint`；需要纯轮式对照时显式传入 `odometry_mode:=wheel`。

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

当前入口启动 Gazebo、默认的轮速 + IMU 二维 EKF、自研 C++ 激光预处理、
扫描匹配/位姿图节点和专用
RViz，不启动 SLAM Toolbox。它发布 `/custom_slam/map` 和
`map -> odom`，但不占用标准 `/map`；因此不能与 SLAM Toolbox 建图入口
同时运行。

无界面启动自研 SLAM 退化长走廊回归环境：

```bash
ros2 launch slam_robot_bringup corridor_slam_regression.launch.py
```

需要人工观察时可增加 `gui:=true use_rviz:=true`。启动完成后，在另一个
终端运行 `ros2 run slam_robot_slam corridor_regression`。
研究模式下可传入 `reject_degenerate_loop_closures:=false`，再用
`corridor_regression --return-trip --expect-anisotropic-loop-closure`
验证 rank 1 回环确实以各向异性平移信息进入位姿图；默认仍保守拒绝。

其余自研 SLAM 专用入口：

| 场景 | 启动命令 | 配套判定器 |
| --- | --- | --- |
| 重复结构 | `repeated_structure_slam_regression.launch.py` | `repeated_structure_regression` |
| 155 m 大场景 | `large_scale_slam_regression.launch.py` | `large_scale_regression` |
| 固定数据集录制 | `large_scale_dataset_recording.launch.py` | `large_scale_regression` |

例如运行大场景回归：

```bash
# 终端 1
ros2 launch slam_robot_bringup large_scale_slam_regression.launch.py

# 终端 2
ros2 run slam_robot_slam large_scale_regression
```

固定数据集录制和 2× 离线回放的完整命令、话题白名单与文件指纹见
[docs/datasets.md](../../docs/datasets.md)。
