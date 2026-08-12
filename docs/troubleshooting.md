# 常见问题与排查

## 同一时间只运行一个完整入口

多个 Gazebo、`/clock`、`/odom` 或 TF 发布源会造成时间戳异常、扫描错位和
不可解释的性能下降。开始新测试前先正常关闭旧 launch；不要只终止父 shell
却留下子进程。

检查可能的残留：

```bash
pgrep -af 'gz sim|rtabmap|slam_toolbox|ekf_filter_node|nav2'
```

不要在不确认目标的情况下批量杀进程；优先回到原启动终端按一次 `Ctrl+C`。

## 关闭 Gazebo 后机器人不动、规划线反复变化

Gazebo 提供 `/clock`、传感器和机器人动力学；关闭其窗口就等于终止本轮仿真，而
不是只隐藏三维视图。正式 launch 会在 Gazebo 退出时联动关闭其余节点。探索器若
先检测到 Nav2 action server 失活，会显示红色 `EXPLORATION ABORTED` 并发布 ERROR
诊断，不会把被拒绝的目标计入黑名单、发布 `complete=true` 或保存最终快照。

旧版本若出现 RViz 仍开着、红色路径不断变化但机器人不动，应结束整个 launch 后
重新启动，不要继续使用这次地图。诊断时检查：

```bash
ros2 topic hz /clock
ros2 lifecycle get /bt_navigator
ros2 topic echo /frontier_explorer/diagnostics --once
```

`/clock` 不再发布或 `bt_navigator` 为 inactive，说明是仿真/导航依赖失效，不是安全
随机出生点本身不可通行。

## Gazebo 窗口空白或无法移动视角

- 确认没有另一份 Gazebo server 或 GUI 使用相同世界。
- `structured_loop_3d.sdf` 已配置 `InteractiveViewControl` 和
  `CameraTracking`；修改源码后要重新构建并重新启动，运行中的实例不会更新。
- WSL2 默认使用 Mesa D3D12，并选择名称包含 `NVIDIA` 的适配器。AMD 或 Intel
  可在 launch 中传入对应的 `wsl_gpu_adapter` 名称。

`Anti-aliasing level ... is not supported` 表示 Ogre2 把不支持的 FSAA 级别
回退到可用值，不影响物理、传感器或 SLAM。

## RViz Message Filter 间歇红色

启动阶段，仿真时钟和 TF 缓存尚未就绪，少量 `Message Filter` 错误可以恢复。
若持续闪烁，应检查：

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map lidar_link
ros2 topic echo /scan --once --field header
```

持续错误通常来自 TF 缺失、时间戳不一致、多个 TF 发布者或旧进程残留，而不是
RViz 配色问题。

## 转弯时扫描短暂偏离墙体

10 Hz LiDAR 在转弯时可能出现几厘米的短暂点云偏移，停稳后重新对齐且地图不
持续重影时，一般属于扫描周期与 `0.05 m` 地图分辨率的正常表现。偏移持续
累积或直行后不能恢复时，应检查轮距标定、时间戳、TF 和实时率。

## RTAB-Map 是否会覆盖旧地图

3D launch 默认 `reset_database:=true`，因此不写也会删除旧数据库并从新地图
开始。只有显式传入下面参数才会复用：

```bash
reset_database:=false
```

数据库路径默认为 `~/.ros/rtabmap_3d.db`。

## 物理上能走，但 Nav2 判断无路

先区分真实障碍膨胀和地面误标。项目曾发现 RTAB-Map 六自由度 ICP 令
`map -> odom` 产生不足一度的倾斜，远处地面因此越过障碍高度门限，在代价
地图形成同心弧假障碍，最终返回 `NO_VALID_PATH`。

正式配置已启用 `Reg/Force3DoF=true` 和 `RGBD/ForceOdom3DoF=true`。修改后
必须重新启动；如果旧数据库由未约束版本建立，还必须使用默认重置行为重新
建图。不要简单提高离地过滤阈值，这会漏掉真实低矮障碍。

检查当前约束与 TF：

```bash
ros2 param get /rtabmap/rtabmap Reg/Force3DoF
ros2 run tf2_ros tf2_echo map odom
```

平面机器人运行时，`map -> odom` 的 Z、roll、pitch 应保持接近零。

## 从固定包中途回放时前端不处理任何扫描

症状很像 SLAM 失效，其实是回放问题：节点正常启动（有快照时还会报告恢复成功
并发布一次地图），随后只反复打印

```text
Waiting for 3D LiDAR static transform: "lidar_3d_link" ... does not exist
```

固定包里 `/tf_static` 只有开头那一条，`ros2 bag play --start-offset` 会跳过它。
前端拿不到雷达外参就不会处理扫描，而地图、TF、诊断看起来都"有输出"，因此容易
误判成配准或恢复出了问题。确认方式：

```bash
ros2 run tf2_ros tf2_echo base_link lidar_3d_link
```

中途回放时用 `recorded_static_tf_publisher` 从包里读出该消息并常驻广播；
`snapshot_resume_phase.launch.py` 已经这样做。另起一个只放该话题的播放器不能
解决——它发布后 1 s 内退出，transient_local 发布者在发现完成前消失。

## 3D RViz 看起来像二维地图

Nav2 的路径规划仍基于二维栅格，但局部避障可以使用 3D 点云的安全高度投影。
在专用 RViz 中启用：

- `RTAB-Map Cloud`：累计三维点云地图。
- `Current 3D Scan`：实时 `/lidar_3d/points`。
- `Map`、costmap 和 plan：二维导航层。

若只看到二维灰色栅格，先确认点云显示项已启用及其 Topic、QoS 和 Fixed Frame。

## 退出时出现生命周期或强制终止日志

项目直接启动 Gazebo 而不经过额外 shell，并默认以非组合模式启动 2D Nav2，
以保证一次 `Ctrl+C` 可以回收完整进程树。退出时 Gazebo 仍可能打印非零退出码；
只要 launch 已结束且下面的残留检查为空，就不表示运行过程失败。需要节省进程
和内存时仍可显式传入 `use_composition:=True`，但 Jazzy 的组合容器关机较慢，
可能等待超时后才由 launch 强制回收。2D 自动保存期间只按一次 `Ctrl+C`，并
等待 `Save completed`。
