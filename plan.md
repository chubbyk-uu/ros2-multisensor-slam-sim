# ROS 2 Gazebo 多传感器 SLAM 仿真项目计划

## 1. 项目目标

在 ROS 2 Jazzy 和 Gazebo Sim 中搭建一个可扩展的差速轮式机器人仿真平台，并按以下顺序实现 SLAM：

1. 2D 激光 SLAM。
2. 2D 地图定位与自主导航。
3. 自研 2D 激光 SLAM 核心模块。
4. 3D 激光 SLAM。
5. 视觉 SLAM。
6. 多传感器融合。

第一阶段的最终成果是：通过一条 launch 命令启动仿真，在 RViz 中遥控机器人，生成闭环一致的二维地图并保存地图。

## 2. 当前环境

已确认具备以下基础环境：

- ROS 2 Jazzy。
- Gazebo Sim（`gz`）。
- `ros_gz_sim`、`ros_gz_bridge`。
- `robot_state_publisher`、Xacro。
- `slam_toolbox`。
- Nav2。

已建立 description、gazebo、bringup 和 slam 四个基础 ROS 2 包；机器人、仿真、2D LiDAR、SLAM Toolbox 和地图保存链路已经接通，当前重点是完成稳定的闭环建图。

主要开发语言约定：

- SLAM 核心算法和正式 ROS 2 节点使用 C++17。
- Python 用于 launch、数据分析、测试工具和算法原型。
- Xacro 管理机器人模型，YAML 管理可调参数。

## 3. 建议目录结构

```text
slam/
├── AGENTS.md
├── plan.md
├── src/
│   ├── slam_robot_description/
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   ├── slam_robot_gazebo/
│   │   ├── worlds/
│   │   ├── models/
│   │   ├── config/
│   │   └── launch/
│   ├── slam_robot_bringup/
│   │   ├── config/
│   │   └── launch/
│   ├── slam_robot_slam/
│   │   ├── config/
│   │   ├── launch/
│   │   └── src/
│   └── slam_robot_navigation/
│       ├── config/
│       └── launch/
├── build/
├── install/
└── log/
```

各包职责：

- `slam_robot_description`：URDF/Xacro、模型资源和模型显示。
- `slam_robot_gazebo`：世界文件、Gazebo 系统和 ROS-Gazebo 桥接。
- `slam_robot_bringup`：统一组织机器人、仿真和算法启动。
- `slam_robot_slam`：`slam_toolbox` 配置及后续自研 SLAM 节点。
- `slam_robot_navigation`：地图加载、AMCL 和 Nav2 配置。

## 4. 系统架构

### 4.1 数据流

```text
遥控节点 / Nav2
       |
       v
   /cmd_vel
       |
       v
Gazebo 差速驱动 ----> /odom
       |
       +------------> /joint_states

Gazebo 2D LiDAR ----> /scan

robot_state_publisher ----> /tf、/tf_static

/scan + TF + /odom ----> slam_toolbox ----> /map、map -> odom
```

### 4.2 TF 结构

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_link
            └── lidar_mount_link
                └── lidar_link
```

TF 发布职责：

- `map -> odom`：由 `slam_toolbox` 发布。
- `odom -> base_footprint`：由差速驱动里程计发布。
- `base_footprint -> base_link` 及机器人结构：由 `robot_state_publisher` 发布。
- `base_footprint` 位于左右驱动轮轴线中点的地面投影，作为差速运动学旋转中心。
- 底盘几何中心相对驱动轴后移，质心投影保持在驱动轮与后万向轮形成的支撑三角形内。

## 5. 第一阶段：2D 激光 SLAM

### 5.1 建立 ROS 2 包

任务：

- 创建 `src/`。
- 创建 description、gazebo、bringup 和 slam 四个基础包。
- 配置依赖、安装目录和基础 launch 文件。
- 暂缓创建 navigation 包，等建图链路稳定后再加入。

验收：

- `colcon build` 成功。
- 所有包可通过 `ros2 pkg list` 查到。
- `source install/setup.bash` 后可以启动基础 launch 文件。

### 5.2 创建差速轮式机器人模型

第一版机器人建议参数：

| 项目 | 初始值 |
| --- | --- |
| 底盘尺寸 | 0.45 × 0.32 × 0.12 m |
| 轮子半径 | 0.075 m |
| 轮宽 | 0.035 m |
| 几何轮距 | 0.34 m |
| Gazebo 有效运动学轮距 | 0.306 m（原地旋转标定值） |
| 雷达高度 | 0.25～0.35 m |

任务：

- 使用 Xacro 建立底盘、左右驱动轮和万向轮。
- 添加 visual、collision 和 inertia。
- 建立 `base_footprint`、`base_link` 和轮子关节。
- 添加 `lidar_link`，暂时只建立安装位置。
- 创建模型显示 launch 和 RViz 配置。

验收：

- Xacro 可以正确展开为 URDF。
- RViz 中模型尺寸、方向和关节正确。
- TF 树连通且没有重复发布。
- 无惯性矩阵和关节定义错误。

### 5.3 接入 Gazebo Sim

任务：

- 创建基础测试世界。
- 将机器人生成到 Gazebo。
- 配置关节状态发布。
- 配置差速驱动系统。
- 桥接 `/clock`、`/cmd_vel`、`/odom` 和 `/joint_states`。

验收：

- 机器人可以稳定生成，不弹飞、不下沉。
- 发布 `/cmd_vel` 后能前进、后退和转向。
- `/odom` 连续变化，运动方向与机器人坐标系一致。
- 直线运动无严重侧滑，旋转中心合理。

### 5.4 添加 2D 激光雷达

建议初始参数：

| 项目 | 初始值 |
| --- | --- |
| 扫描范围 | -180°～180° |
| 角分辨率 | 约 0.5° |
| 最小距离 | 0.12 m |
| 最大距离 | 12～20 m |
| 更新频率 | 10 Hz |

任务：

- 在 `lidar_link` 上添加 Gazebo GPU LiDAR 或 LiDAR 传感器。
- 将 Gazebo 激光数据桥接到 ROS 2 `/scan`。
- 设置正确的消息类型、QoS 和 `frame_id`。
- 在 RViz 中添加 LaserScan 显示。

验收：

- `/scan` 持续发布且频率正确。
- `frame_id` 与机器人 TF 一致。
- RViz 中激光点与墙体重合。
- 机器人运动时激光数据不会相对模型漂移。

### 5.5 创建 SLAM 测试世界

世界应包含：

- 封闭房间。
- 内部隔墙。
- 若干箱体或立柱。
- 一条可形成回环的通道。
- 足够的非对称几何特征。

验收：

- 场景可以稳定加载。
- 碰撞模型与视觉模型基本一致。
- 激光能观察到足够的几何特征。
- 仿真实时率能够满足 10 Hz 激光和 SLAM 运算。

### 5.6 接入 `slam_toolbox`

第一版使用 `online_async` 模式。

主要配置项：

- `use_sim_time`。
- `scan_topic`。
- `map_frame`、`odom_frame` 和 `base_frame`。
- 地图分辨率，初始值为 `0.05 m/cell`。
- 激光最大有效距离。
- 扫描匹配阈值。
- 关键帧插入距离和角度。
- 回环检测参数。

验收：

- `/map` 正常发布。
- 地图随机器人运动逐步扩展。
- 墙体没有明显双线和重影。
- 机器人绕场景一圈后可以产生合理回环。
- SLAM 过程中没有 TF 超时或时间戳错误。

### 5.7 保存地图

任务：

- 保存 `map.yaml` 和地图图像。
- 保存必要的 `slam_toolbox` 序列化地图。
- 编写地图保存说明和启动命令。

验收：

- 地图文件可以重新加载。
- 地图分辨率、原点和占据阈值正确。
- 重启系统后地图不会损坏或发生比例变化。

## 6. 第二阶段：定位与导航

在 2D 建图稳定后进行：

- 使用 Nav2 Map Server 加载已保存地图。
- 使用 AMCL 完成二维定位。
- 配置全局代价地图和局部代价地图。
- 配置规划器、控制器和恢复行为。
- 在 RViz 中设置初始位姿并发送导航目标。

验收：

- 初始位姿设置后 AMCL 能够收敛。
- 激光与静态地图正确对齐。
- 机器人能够规划路径、动态避障并到达目标。

## 7. 第三阶段：自研 2D 激光 SLAM

成熟方案跑通后，记录 `/scan`、`/odom`、`/tf` 和 `/tf_static`，通过 rosbag 离线开发以下模块。

### 7.1 激光预处理

- 去除 NaN、Inf 和越界距离。
- 距离裁剪与可选降采样。
- 将极坐标激光点转换到机器人坐标系。

### 7.2 帧间扫描匹配

- 使用里程计作为位姿初值。
- 首先实现 ICP 或点到线 ICP。
- 计算相邻激光帧之间的相对位姿。
- 对比扫描匹配轨迹和 Gazebo 真值。

### 7.3 局部地图与关键帧

- 建立局部点云或局部栅格地图。
- 当前扫描与局部地图匹配。
- 按平移、旋转阈值插入关键帧。
- 控制局部地图大小和实时计算量。

### 7.4 占据栅格地图

- 使用射线模型更新空闲和占据区域。
- 使用 Bresenham 算法遍历栅格。
- 使用 log-odds 表示占据概率。
- 发布标准 `nav_msgs/OccupancyGrid`。

### 7.5 回环与位姿图

- 搜索历史回环候选。
- 使用激光匹配验证回环。
- 建立里程计、扫描匹配和回环约束。
- 使用 Ceres、G2O 或 GTSAM 优化位姿图。
- 优化后重建全局地图。

## 8. 后续扩展

### 8.1 3D 激光

- 在机器人模型中加入 3D LiDAR。
- 桥接 `PointCloud2`。
- 添加 IMU。
- 研究 LIO、点云去畸变、局部地图和回环检测。

### 8.2 视觉

- 添加 RGB、双目或 RGB-D 相机。
- 配置相机内参、深度和图像桥接。
- 实现特征提取、匹配、视觉里程计和回环。

### 8.3 多传感器融合

- 统一相机、激光、IMU 和轮速时间戳。
- 标定传感器外参。
- 使用 EKF、因子图或紧耦合方法进行融合。

## 9. 开发与调试策略

每个阶段遵循以下流程：

1. 启动最少数量的节点。
2. 检查节点、话题和服务是否存在。
3. 检查话题频率、消息内容和时间戳。
4. 检查 TF 连通性和发布者。
5. 在 RViz 中检查数据空间对齐。
6. 最后再调整算法参数。
7. 记录启动命令、测试结果和已知问题。

问题排查优先级：

```text
仿真时间
-> TF
-> 传感器 frame_id
-> 里程计方向和尺度
-> 激光数据质量
-> SLAM 参数
-> 算法实现
```

## 10. 近期任务清单

- [x] 确认 ROS 2、Gazebo、SLAM Toolbox 和 Nav2 环境。
- [x] 编写项目协作规范。
- [x] 编写总体项目计划。
- [x] 创建 ROS 2 基础包。
- [x] 创建差速轮式机器人 Xacro。
- [x] 在 RViz 中验证机器人模型与 TF。
- [x] 在 Gazebo 中生成机器人。
- [x] 配置差速驱动和 ROS-Gazebo bridge。
- [x] 添加 2D 激光雷达。
- [x] 创建 SLAM 测试世界。
- [x] 接入 `slam_toolbox`。
- [x] 标定 Gazebo 差速驱动有效轮距，验证原地旋转时真值、里程计和 SLAM 航向对齐。
- [x] 验证占据栅格与 SLAM Toolbox 位姿图保存流程。
- [x] 优化 WSL2 图形后端、Gazebo 物理与渲染负载、RViz 刷新率和 SLAM 关键帧频率。
- [ ] 完成第一张二维地图并保存。
- [ ] 接入 AMCL 和 Nav2。
- [ ] 开始自研 2D 激光 SLAM 模块。
