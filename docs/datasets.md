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
