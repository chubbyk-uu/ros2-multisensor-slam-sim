# 性能与旋转标定记录

本文记录 2D SLAM 基线的性能优化和里程计标定结果。数值用于同一测试环境下的前后对比，不代表所有硬件上的固定性能。

## 性能优化

2026-07-29 在 WSL2 与 NVIDIA 独立显卡环境完成回归：

| 指标 | 优化前 | 优化后 |
| --- | ---: | ---: |
| Gazebo GUI CPU | 约 529% | 约 132% |
| Gazebo Server CPU | 约 58% | 约 31% |
| RViz CPU | 约 93% | 约 10% |
| 仿真实时率 | 约 1.0 | 稳定 1.0 |

Linux 中单个进程的 CPU 百分比可以超过 100%，表示它同时占用了多个逻辑核心。

当前配置的主要取值：

- Gazebo 物理循环：250 Hz。
- 2D LiDAR：10 Hz。
- 轮式里程计：约 28 Hz。
- SLAM 地图更新周期：2 s。
- Gazebo 不绘制 LiDAR 射线，激光仍通过 `/scan` 在 RViz 中显示。
- RViz 默认关闭 TF 可视化，需要排查坐标系时可手动启用。

Gazebo 的物理、传感器和场景管理仍主要运行在 CPU 上；GPU 负责 GUI 的三维渲染，因此启用 D3D12 不会让 Gazebo Server 的物理计算转移到 GPU。

## 旋转与里程计标定

机器人驱动轮几何中心距为 `0.34 m`。考虑 Gazebo ODE 中有限宽圆柱轮的有效接触位置，经过原地旋转标定，差速驱动和里程计使用的有效轮距为 `0.306 m`。该参数不会改变模型几何或 `base_footprint` 的位置。

优化后进行约 127° 原地旋转：

- 轮式里程计相对 Gazebo 真值的航向误差约 `0.31°`。
- SLAM 相对 Gazebo 真值的航向误差约 `0.29°`。
- SLAM 平移偏差约 `1.5 mm`。

SLAM 的旋转关键帧阈值为 `0.05 rad`，用于减小起转和停转时激光相对地图的瞬时偏移。10 Hz LiDAR 与 0.05 m 栅格分辨率下，小角度转弯仍可能出现几厘米的短暂误差，应以停稳后的重新对齐和整张地图是否持续重影作为判断标准。

## 复现基础运动测试

短时发送直行指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}" -r 10
```

结束速度发布后发送一次零速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

测试时应同时记录 Gazebo 真值、`/odom`、SLAM 位姿、实时率以及相关进程 CPU，占用变化需要在相同场景和显示配置下比较。
