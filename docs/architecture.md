# 系统架构与接口

## 包职责

| 包 | 职责 |
| --- | --- |
| `slam_robot_description` | Xacro、机器人模型资源和模型显示 |
| `slam_robot_gazebo` | Gazebo 世界、系统插件和 ROS–Gazebo 桥接 |
| `slam_robot_bringup` | 2D 建图、定位、导航、自研 SLAM 和回归的组合启动入口 |
| `slam_robot_slam` | SLAM Toolbox 配置、自研 C++ 2D SLAM、数据录制和算法测试 |
| `slam_robot_slam_3d` | 自研 3D SLAM、点云契约、RTAB-Map/MOLA 对照、快照和结构化回归 |
| `slam_robot_navigation` | AMCL、Nav2、在线 SLAM 导航、Frontier Exploration 和导航回归 |

各包内部参数与实现细节见对应的 `src/<package>/README.md`。

## 传感器与里程计

```text
Gazebo diff drive
  ├─ wheel mode ───────────────────────> /odom + odom -> base_footprint
  └─ wheel_imu mode
       ├─ /wheel/odom_raw ─┐
       └─ /imu/data_raw ───┴─> covariance adapter
                              -> /wheel/odom + /imu/data
                              -> robot_localization EKF
                              -> /odom + odom -> base_footprint

Gazebo 2D LiDAR -> /scan
Gazebo 3D LiDAR -> /lidar_3d/points
Gazebo world pose -> /ground_truth/odom（仅测试与评分）
```

Gazebo 裸里程计和 IMU 消息不包含可用协方差。轻量适配节点补充有限、非零且
符合差速非完整约束的协方差，再交给 EKF。二维 EKF 只融合轮速平面速度和
IMU 偏航角速度，不融合 IMU 绝对姿态或线加速度。

## SLAM 与导航数据流

```text
2D 官方链路
  /scan + /odom + TF -> SLAM Toolbox -> /map + map -> odom
  saved map + /scan + TF -> Map Server + AMCL + Nav2

2D 自研链路
  /scan + /odom + TF -> 相关扫描匹配 + 关键帧局部子图
    -> 后台回环检测 + Ceres 位姿图
    -> 优化射线重放 -> /custom_slam/map
    -> map -> odom

3D RTAB-Map 成熟链路
  /lidar_3d/points + /odom -> RTAB-Map ICP + 回环 + 位姿图
    -> 3D MapData + /rtabmap/map + map -> odom
    -> Nav2 全局静态层

3D 自研链路
  /lidar_3d/points + /odom -> 预处理 + GICP 局部子图前端
    -> Scan Context + GICP 回环 + 后台 SE(2) 位姿图
    -> /custom_slam_3d/map_cloud + /map + map -> odom
    -> Nav2 全局静态层
  /lidar_3d/points -> Nav2 局部体素障碍层

在线自主探索
  /map 或 /rtabmap/map -> frontier 聚类 + ComputePathToPose
    -> NavigateToPose -> Nav2（探索器不发布 /cmd_vel）

3D 里程计对照
  /lidar_3d/points -> MOLA GICP
    -> /lidar_odometry/pose + /lidar_odometry/localmap_points
```

同一时间只能启用一条发布 `map -> odom` 的 SLAM 或定位链路。SLAM Toolbox、
AMCL、RTAB-Map、自研 SLAM 和 MOLA 不能以冲突组合同时运行。

## TF 树

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_link
            ├── imu_link
            └── LiDAR 二选一
                ├── lidar_mount_link -> lidar_link（2D）
                └── lidar_3d_mount_link -> lidar_3d_link（3D）
```

- SLAM 或定位节点唯一发布 `map -> odom`。
- 轮式模式或 EKF 唯一发布 `odom -> base_footprint`。
- `robot_state_publisher` 发布机器人固定结构和关节 TF。
- `base_footprint` 位于驱动轮轴线中点的地面投影，是差速运动学旋转中心。

几何轮距为 `0.34 m`；Gazebo 驱动和里程计使用原地旋转标定得到的有效轮距
`0.306 m`。完整测量记录见 [性能与标定](performance.md)。

## 设计边界

- 2D 与 3D LiDAR 使用互斥机器人模型，分别服务于独立 SLAM 基线。
- 当前 3D Gazebo 点云没有逐点时间字段，因此 RTAB-Map 和自研链路属于低速
  几何 ICP SLAM，MOLA 对照关闭 deskew；三者都不应称为完整 LIO。
- 真值 `/ground_truth/odom` 只能用于自动驾驶测试和误差评分，禁止进入
  SLAM、EKF 或导航估计链路。
- 后续多传感器路线是 3D LiDAR + 相机 + IMU，不机械融合 2D 与 3D LiDAR。
