# slam_robot_description

本包保存差速轮式机器人的 Xacro 模型、模型显示 launch 文件和 RViz 配置。

模型遵循 REP-103 坐标约定：`x` 向前、`y` 向左、`z` 向上。

`base_footprint` 位于左右驱动轮轴线中点的地面投影，`base_link` 位于其正上方。
底盘几何中心相对驱动轴后移 0.07 m，后万向轮与两驱动轮构成稳定的三点支撑。
雷达通过有实体、碰撞体和惯性的 `lidar_mount_link` 固定到底盘。

当前模型只包含 2D LiDAR，TF 固定链为
`base_link -> lidar_mount_link -> lidar_link`。3D 阶段会新增独立的
`lidar_3d_mount_link -> lidar_3d_link`，不会复用或替换现有 2D 雷达坐标系，
以保证已经冻结的 2D 回归接口保持兼容。
