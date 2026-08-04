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

## 3D RViz 看起来像二维地图

Nav2 的路径规划仍基于二维栅格，但局部避障可以使用 3D 点云的安全高度投影。
在专用 RViz 中启用：

- `RTAB-Map Cloud`：累计三维点云地图。
- `Current 3D Scan`：实时 `/lidar_3d/points`。
- `Map`、costmap 和 plan：二维导航层。

若只看到二维灰色栅格，先确认点云显示项已启用及其 Topic、QoS 和 Fixed Frame。

## 退出时出现生命周期或强制终止日志

Nav2 与 Gazebo 组合进程在 `Ctrl+C` 后可能打印生命周期切换或超时终止日志。
如果运行过程正常、地图或数据库已经写盘，通常不代表任务失败。2D 自动保存
期间只按一次 `Ctrl+C`，并等待 `Save completed`。
