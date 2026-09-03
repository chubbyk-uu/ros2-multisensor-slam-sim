# slam_robot_description

本包保存差速轮式机器人的 Xacro 模型、模型显示 launch 文件和 RViz 配置。

模型遵循 REP-103 坐标约定：`x` 向前、`y` 向左、`z` 向上。

`base_footprint` 位于左右驱动轮轴线中点的地面投影，`base_link` 位于其正上方。
底盘几何中心相对驱动轴后移 0.07 m，后万向轮与两驱动轮构成稳定的三点支撑。
雷达通过有实体和惯性的安装座固定到底盘；IMU 位于 `base_link` 原点，
即驱动轮轴线中点正上方的底盘内部。

`sensor_variant:=2d|3d` 选择互斥的 LiDAR 结构：

- 默认 `2d` 保留原有 `base_link -> lidar_mount_link -> lidar_link`。
- `3d` 使用 `base_link -> lidar_3d_mount_link -> lidar_3d_link`，传感器
  位于底盘几何中心上方，不生成 2D 雷达链接。

两种配置都包含 `base_link -> imu_link`。这样不需要在同一机器人上堆叠
两套雷达，也不会改变已经冻结的 2D 雷达安装位姿和回归几何。

`camera_variant:=none|rgbd` 与 LiDAR 选择相互独立，默认 `none` 保持已有回归
模型不变。`rgbd` 增加
`base_link -> camera_mount_link -> camera_link -> camera_optical_frame`：光心位于
`base_link` 的 `(0.155, 0, 0.09) m`，离地 `0.225 m`。相机外壳从底盘前沿向后
收进车体，不扩大 Nav2 footprint；顶部离地 `0.245 m`，低于 3D LiDAR 的
`-15°` 最低光束约 `10 mm`，也低于雷达本体下沿。`camera_link` 遵循 REP-103
机器人坐标，`camera_optical_frame` 为 `z` 向前、`x` 向右、`y` 向下的标准
光学坐标系。
