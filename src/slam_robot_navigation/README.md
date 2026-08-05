# slam_robot_navigation

本包使用 ROS 2 Jazzy 安装的 Nav2 官方参数和官方 RViz 配置，提供保存地图上的 AMCL 定位与自动导航。

保留的官方主要组件包括 MPPI Controller、Navfn Planner、Simple Smoother、Behavior Server、Velocity Smoother、Collision Monitor、Waypoint Follower、Route Server 和 Docking Server。

项目只覆盖与当前机器人相关的参数：

- 定位和导航基座坐标系使用 `base_footprint`。
- 代价地图使用覆盖底盘和驱动轮的八点凸多边形 footprint。
- AMCL 默认初始位姿为地图原点 `(0, 0, 0)`。
- 激光最大距离为 `12 m`。
- MPPI 最大角速度限制为 `1.5 rad/s`，加速度限制与 Gazebo
  差速驱动模型保持一致。

一条命令启动仿真、定位、导航和 RViz：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py
```

仿真已经运行时，可独立启动：

```bash
ros2 launch slam_robot_navigation navigation.launch.py \
  map:="${SLAM_WS}/maps/slam_map.yaml"
```

其中 `SLAM_WS` 应指向仓库根目录。`maps/` 下的地图是每次建图的产物，不进入
版本控制；随仓库分发的演示地图在 `maps/reference/`，尚未自己建图时可直接
指向 `maps/reference/slam_map.yaml`。

如果机器人不是从保存地图时的原点出生，可通过 RViz 的 `2D Pose Estimate` 重新设置初始位姿。

导航前应关闭键盘遥控节点，避免它与 Nav2 同时发布 `/cmd_vel`。官方默认到点位置容差为 `0.25 m`。

完整仿真启动后，可自动执行项目的多目标导航回归路线：

```bash
ros2 run slam_robot_navigation navigation_regression.py
```

测试路线覆盖左右区域、连续转向和返回原点，输出导航结果、耗时、剩余
距离、恢复次数、Collision Monitor 触发次数和 AMCL 终点误差。

动态障碍物测试会在导航开始后生成一个地图中不存在的橙色箱体，并在
测试结束时自动移除：

```bash
ros2 run slam_robot_navigation navigation_regression.py \
  --scenario dynamic-obstacle
```

测试同时检查障碍物是否进入局部和全局代价地图，并记录机器人到障碍物
的最小距离和相对原直线路径的最大绕行距离。

2026-07-30 的无界面完整仿真回归结果为：9/9 个目标成功，总耗时
`98.5 s`，最终剩余路径 `0.10 m`，返回原点的 AMCL 位置误差
`0.107 m`，恢复次数和 Collision Monitor 触发次数均为 0。

2026-08-05 使用自研 2D SLAM 经 Ctrl+C 自动保存的
`242×202 @ 0.05 m` YAML/PGM 地图重复同一回归：9/9 个目标成功，总耗时
`100.7 s`，最终剩余路径 `0.12 m`，返回原点的 AMCL 位置误差
`0.122 m`，恢复次数和 Collision Monitor 触发次数均为 0。该结果同时验收了
自研地图保存、Map Server 加载、地图原点、AMCL 定位和 Nav2 导航的完整链路。

同日动态障碍物回归结果为：局部、全局代价地图均正确标记新箱体，路径
由 `4.19 m` 重规划为约 `5.00 m`，机器人在 `18.5 s` 内成功到达；
最近障碍距离为 `1.083 m`、最大绕行偏移为 `1.207 m`、终点 AMCL
位置误差为 `0.173 m`，箱体随后成功自动删除。恢复次数和 Collision
Monitor 触发次数均为 0，表示常规重规划已及时完成避障；完全封路后的
恢复行为需要单独测试。
