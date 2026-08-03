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
