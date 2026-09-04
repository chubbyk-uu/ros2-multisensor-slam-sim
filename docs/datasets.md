# 固定 SLAM 数据集

## 大场景参考包

`large_scale_reference` 固化了 `large_warehouse.sdf` 中约 155 m 的两圈
路线。数据包本体约 79 MiB，位于工作区的 `bags/`，由 `.gitignore`
排除，不提交到 Git。算法输出也不进入参考包，避免回放新版本时读取旧
版本生成的位姿、地图或 TF。

允许的话题只有：

```text
/clock
/scan
/odom
/ground_truth/odom
/robot_description
/tf_static
```

专用录制入口显式关闭 `/tf`。录制仿真中运行的自研 SLAM 会发布
`map -> odom`，若把整个 `/tf` 录入参考包，回放时就会与待测节点形成
重复发布和结果泄漏。

### 生成参考包

```bash
# 终端 1：启动大场景、输入录制器和在线完整性检查所需的 SLAM
ros2 launch slam_robot_bringup large_scale_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/large_scale_reference"

# 终端 2：执行固定两圈路线
ros2 run slam_robot_slam large_scale_regression
```

路线通过后，在终端 1 按一次 `Ctrl+C`，等待 MCAP 写入完成。不要覆盖
已有参考包；需要重录时先将旧目录移动到备份位置。

当前本地参考包基线：

| 属性 | 值 |
| --- | ---: |
| 格式 | MCAP |
| 时长 | 543.352 s |
| 消息 | 195610 |
| `/scan` | 5434 |
| `/odom` / `/ground_truth/odom` | 27168 / 27168 |
| 文件大小 | 81874087 bytes |
| MCAP SHA-256 | `63d4ebc8c4007e15dcfdf1e2510be7a0f2cbe2023ca050a8b77af3e9cb5c396c` |

检查命令：

```bash
ros2 bag info "${SLAM_WS}/bags/large_scale_reference"
sha256sum "${SLAM_WS}/bags/large_scale_reference"/*.mcap
```

### 离线回归

先启动判定器，再启动带 2 秒延迟的回放：

```bash
# 终端 1
ros2 run slam_robot_slam large_scale_bag_regression

# 终端 2
ros2 launch slam_robot_slam play_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/large_scale_reference" \
  rate:=2.0 use_rviz:=false
```

判定器按消息时间戳插值真值，只把真值用于评估。它检查输入消息数与
持续时间、匹配误差、回环和地图重建、前端间隔、Path 上限、地图范围、
匹配拒绝与严重日志。播放完成并静默 5 秒后自动给出 PASS/FAIL。

2026-08-03 的 2× 基线结果为：543.3 s 数据在 271.7 s 墙钟内完成；
5434 帧扫描产生 5433 帧匹配，重新检测 42 条回环并完成 30 次地图重建。
最终误差 `0.120 m / 0.063°`，全程峰值 `0.230 m / 0.619°`，最大前端
间隔 `0.100 s`，地图范围 `26.15 × 20.20 m`，无拒配或严重日志。

退化边界缩放收口后的同日复核仍处理 5434/5434 帧扫描并通过全部判据：
最终误差 `0.109 m / 0.572°`，峰值 `0.203 m / 0.619°`，重新检测 41 条
回环并完成 30 次地图重建，最大前端间隔 `0.100 s`，无拒配或严重日志。
固定输入能排除 Gazebo 传感器随机噪声，但后台回环任务的完成和提交时序
仍可能使边界候选数量变化；因此该回归按误差上限、输入完整性和关键事件
验收，不要求回环数或最终误差逐位等于历史单次结果。

## 结构化 3D 参考包

`structured_3d_reference` 固化了 `structured_loop_3d.sdf` 中约 `143 m` 的
两圈路线，供 RTAB-Map 和后续自研 3D SLAM 使用完全相同的输入。数据包位于
工作区 `bags/`，由 `.gitignore` 排除，不提交 Git。

允许的话题只有：

```text
/clock
/lidar_3d/points
/wheel/odom
/imu/data
/odom
/ground_truth/odom
/tf_static
/robot_description
```

`/wheel/odom` 与 `/imu/data` 是补齐有限协方差后的传感器输入；`/odom` 是
项目统一的轮速 + IMU EKF 运动初值。它们都不是 SLAM 输出。动态 `/tf`、
RTAB-Map 地图/图和任何自研输出均不录制，防止回放时泄漏旧估计结果。
`/ground_truth/odom` 只允许用于路线控制和离线评分。

两条回放约束由此而来，都是消费方必须知道的：

- **`/tf_static` 全包只有一条，位于 `t≈0.008 s`。** 从头回放没问题；用
  `--start-offset` 从中途开始会直接跳过它，传感器外参永远不到，前端一帧扫描
  都不处理。需要中途回放时用 `recorded_static_tf_publisher` 从包里读出该消息
  并常驻广播——再起一个只放该话题的播放器不可靠，它发布后 1 s 内退出，
  transient_local 发布者在发现完成前消失，实测无人收到。
- **动态 `/tf` 没有录制**，`odom -> base_footprint` 需要用
  `recorded_odom_tf_publisher` 从 `/odom` 重建，否则 TF 树在 `odom` 以下断开。

生成固定参考包：

```bash
ros2 launch slam_robot_slam_3d structured_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_3d_reference"
```

入口以现有 RTAB-Map 两圈回归驱动确定性路线，但录制器只订阅上述八个输入。
路线结束后 launch 自动关闭，等待 rosbag 输出 `Recording stopped` 再使用数据。
输出目录已存在时会直接拒绝覆盖。

当前本地参考包基线：

| 属性 | 值 |
| --- | ---: |
| 格式 | MCAP，`zstd_fast` |
| 时长 | 361.632 s |
| 消息 | 184436 |
| `/lidar_3d/points` | 3617 |
| `/imu/data` | 36164 |
| `/wheel/odom` / `/odom` | 18082 / 18081 |
| 文件大小 | 492365324 bytes |
| MCAP SHA-256 | `e2b6bda6af7bcd8c50982258ccb61b59f78288471952ec46f479bb3f2074a34a` |

检查数据契约和回放：

```bash
ros2 run slam_robot_slam_3d dataset_contract_check \
  "${SLAM_WS}/bags/structured_3d_reference"

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=2.0
```

参考包保留 Gazebo 点云原始字段 `x/y/z/intensity/ring`，仍没有逐点 `time`；
因此它用于纯几何 3D SLAM、回环和地图回归，不用于声称完整 deskew/LIO 验证。

## 结构化 RGB-D 多传感器参考包

`structured_rgbd_reference` 固化同一结构化世界的一圈 `71.6 m` 路线，同时记录
RGB-D、3D LiDAR、轮速、IMU、统一 `/odom`、静态外参和仅供评分的真值。它用于
让 RGB-D 与纯 LiDAR 独立链路在需要时消费字节一致的运动与环境输入；任何一条
算法只能订阅其声明的传感器，不能因为包中同时存在 LiDAR 就把 RGB-D 链路称为
融合。

相机在固定包中配置为 `640 × 480 @ 10 Hz`，低于在线默认 `30 Hz`。原因是组合
原始 RGB + 32FC1 Depth 在 30 Hz 实测约 `65.6 MB/s`，一圈理论未压缩输入超过
`11 GiB`；RTAB-Map 图更新仅为 `2 Hz`，10 Hz 已保留足够的时间采样余量。MCAP
使用 `zstd_fast`：运动短测中五路 RGB/Depth/CameraInfo/LiDAR 均为 `439/439`
帧；`zstd_small` 则在完整一圈只留下约 `6.9–7.0 Hz`，因此被数据契约拒绝。

生成与检查：

```bash
ros2 launch slam_robot_slam_3d structured_rgbd_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_rgbd_reference"

ros2 run slam_robot_slam_3d rgbd_dataset_contract_check \
  "${SLAM_WS}/bags/structured_rgbd_reference"
```

录制器只保存下列输入，不包含 `/tf`、RTAB-Map、自研位姿或地图：

```text
/clock
/lidar_3d/points
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
/wheel/odom
/imu/data
/odom
/ground_truth/odom
/tf_static
/robot_description
```

当前本地基线：

| 属性 | 值 |
| --- | ---: |
| 格式 | MCAP，`zstd_fast` |
| 路线 | 一圈，`71.6 m`，4/4 航点 |
| 时长 | `185.288 s` |
| 消息总数 | `101910` |
| RGB / Depth / 两路 CameraInfo | 各 `1853` |
| `/lidar_3d/points` | `1853` |
| `/imu/data` | `18529` |
| `/wheel/odom` / `/odom` / 真值 | 各 `9264` |
| 文件大小 | `331935023 bytes` |
| MCAP SHA-256 | `59a5380d0a6059bb8a1bc2d3e83f25e96032e91a6a240979ef4e38877a034174` |

一圈路线在终点的朝向与起点不同，也没有重复走过同一路段；它只用于数据契约和短回放。
需要验证视觉回环时必须录制两圈。当前两圈正样本使用带方向性墙面纹理的同一物理世界：

```bash
ros2 launch slam_robot_slam_3d structured_rgbd_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_rgbd_textured_loop_reference" laps:=2

ros2 launch slam_robot_slam_3d rtabmap_rgbd_fixed_regression.launch.py \
  bag:="${SLAM_WS}/bags/structured_rgbd_textured_loop_reference"
```

| 两圈正样本属性 | 值 |
| --- | ---: |
| 路线 | 两圈，`143.136 m`，8/8 航点 |
| 时长 | `360.608 s` |
| RGB / Depth / 3D LiDAR | 各 `3607` |
| 文件大小 | 约 `672 MiB` |
| MCAP SHA-256 | `710fdb2cc7b5fb3baccdd524bba53ec9bbbf512d7f3deeada1bbbc414fb00211` |

在加入方向性墙面纹理前还录制过一份无纹理两圈包
`bags/structured_rgbd_loop_reference`。它不是正式视觉回环正样本，也不能与纹理包组成
精度 A/B：两次独立仿真的输入里程计漂移不同。保留它是为了复核“缺少稳定视觉结构时
RTAB-Map 得到了什么输入与结果”这一负向观察，而不是证明纹理提升了多少精度。

| 无纹理两圈历史包属性 | 值 |
| --- | ---: |
| 录制基线 | `b1d8f7f` 后的未提交工作区 |
| 路线 | 两圈，`route_laps=2` |
| 时长 | `360.596 s` |
| RGB / Depth / 3D LiDAR | `3606 / 3606 / 3606` |
| 两路 CameraInfo | 各 `3607` |
| `/wheel/odom` / `/odom` / 真值 | `18030 / 18029 / 18030` |
| 文件大小 | `653339364 bytes` |
| MCAP SHA-256 | `16207e2e196d00e316d8b4c1d862c45c4903106573ebd03962283a785b122cf8` |
| 当时世界 SHA-256 | `9fba4a68aba8e7a82a81d94805cba06c1d15a12451ab98e9c4a8074eb5e95173` |

录制所用入口和参数为：

```bash
ros2 launch slam_robot_slam_3d structured_rgbd_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_rgbd_loop_reference" laps:=2
```

当前 `structured_loop_3d.sdf` 已含纹理，同一命令不会重新生成字节相同的无纹理包；要
复核其世界内容必须结合上表的提交与世界哈希。可机器读取的清单见
[`2026-09-03-structured-rgbd-untextured-loop-dataset.json`](results/2026-09-03-structured-rgbd-untextured-loop-dataset.json)。

元数据记录 `route_laps`，契约检查会验证所有相机与 LiDAR 流的完整性。上述三份包均位于
`bags/` 且不提交 Git。

从头回放：

```bash
ros2 launch slam_robot_slam_3d play_rgbd_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_rgbd_reference"
```

播放器作为大消息 writer 使用与相机 bridge 相同的 Fast DDS profile。回放冒烟中
RGB 和 Depth 均约 `10.00 Hz`，CameraInfo frame 为 `camera_optical_frame`。
一圈包的可复核元数据见
[`2026-09-03-structured-rgbd-dataset.json`](results/2026-09-03-structured-rgbd-dataset.json)。
