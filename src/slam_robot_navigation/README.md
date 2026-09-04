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

测试同时检查障碍物是否进入局部和全局代价地图，以消息时间戳记录从生成箱体到
首次致命格发布的仿真时延（默认上限分别为 `0.8 s`、`2.2 s`）。目标到达、两张
代价地图的感知证据、时延、车体到障碍物的真实净空、绕行偏移、终点误差、恢复预算和
Collision Monitor 均进入最终 PASS/FAIL，而不再只是打印测量值。
绕行偏移由实际起点和目标生成名义路线，只统计障碍站位附近的横向偏移；默认下限
`0.760 m` 是机器人与 `0.60 m` 方箱的保守外接圆之和。四个行为门限的完整来源见
[验证方法](../../docs/methodology.md#封路与动态障碍的判据)。

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

代价地图话题的发布节拍不必等于内部更新频率：当前全局 `1 Hz` 更新 / `1 Hz` 发布
组合在更新 tick 上量化后实测为 `0.5 Hz`，局部 `5 Hz` 更新 / `2 Hz` 发布实测为
`1.667 Hz`。这不降低进程内局部障碍更新；动态回归以消息时间戳而非 `ros2 topic hz`
的观测频率判断首次障碍物可见性。测量依据见[性能与标定](../../docs/performance.md#nav2-代价地图更新与话题发布节拍)。

## Frontier Exploration

`frontier_explorer_node` 订阅在线占据栅格，聚类自由区—未知区边界，并按信息
增益、机器人距离和障碍净距生成候选。候选先经 Nav2
`ComputePathToPose` 可达性检查和路径长度重评分，再通过 `NavigateToPose`
执行。节点不发布 `/cmd_vel`，因此规划、控制、避障和恢复仍全部由 Nav2
负责。

独立启动方式：

```bash
ros2 launch slam_robot_navigation frontier_exploration.launch.py \
  map_topic:=/map
```

主要输出为候选 `/frontier_explorer/markers`、持久完成提示
`/frontier_explorer/status_marker`、`/frontier_explorer/diagnostics` 和
transient-local `/frontier_explorer/complete`。`map_topic`、候选数、评分权重、
黑名单、超时与完成判据集中在 `config/frontier_exploration.yaml`。默认在连续
若干周期没有未拉黑的可达候选，或连续成功目标未带来最低自由区增长时结束；
900 秒空间黑名单确保永久不可达候选不会在导航超时后立即重新进入候选集。自研
3D SLAM 默认在完成时调用快照服务；接 RTAB-Map 时关闭该调用，由 RTAB-Map
数据库自身负责持久化。

节点支持按最终分数划出顶部区间并随机选择，`selection_random_seed:=0` 会生成新
种子；但项目正式配置的区间宽度为 `0.0`，因此当前总是选择评分最高的可达候选，
路线随机性由上层安全随机出生提供。完成时当前终端先显示醒目提示，快照保存成功后
再次提示可以安全按 `Ctrl+C`；同样的状态以持久 `TEXT_VIEW_FACING` Marker 显示在
RViz。Nav2 action server 在曾经就绪后失活或直接拒绝 goal 时属于运行故障：节点
进入 `fault`、发布 ERROR 诊断和红色 `EXPLORATION ABORTED`，不拉黑候选、不发布
完成消息，也不调用快照服务。
