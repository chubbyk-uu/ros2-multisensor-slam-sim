# 3D 建图、导航与验收

当前保留两条在线 3D SLAM 链路：RTAB-Map 是成熟验收基线，自研链路已完成
GICP 前端、回环后端、全局地图、Nav2 和 Frontier Exploration。MOLA GICP
作为不伪造逐点时间字段的纯激光里程计对照。各算法使用相同的 3D LiDAR
机器人变体，但不能同时发布 `map -> odom`。

## 固定 3D 数据集

自研算法与 RTAB-Map 的统一输入已经固化为结构化世界两圈 MCAP：

```bash
ros2 launch slam_robot_slam_3d structured_dataset_recording.launch.py \
  output:="${SLAM_WS}/bags/structured_3d_reference"
```

数据包不记录动态 `/tf` 或任何 SLAM 输出。使用前运行：

```bash
ros2 run slam_robot_slam_3d dataset_contract_check \
  "${SLAM_WS}/bags/structured_3d_reference"
```

完整话题契约、哈希和回放方式见[固定数据集](datasets.md)。

## 自研点云预处理

先启动预处理节点，再回放同一固定输入：

```bash
ros2 launch slam_robot_slam_3d custom_3d_preprocessing.launch.py

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=10.0
```

节点输出 `/custom_slam_3d/points_filtered`，其 `frame_id` 和时间戳与原始扫描
一致。过滤顺序为非有限值、量程、本体包围盒、`0.05 m` 建图体素；前端再从
同一帧内部生成 `0.10 m` 注册点云，不增加话题同步。参数全部在
`custom_3d_slam.yaml`。该阶段不发布任何 TF。用下面的工具收集多帧诊断并
检查平均处理预算：

```bash
ros2 run slam_robot_slam_3d preprocessing_regression \
  --ros-args -p minimum_samples:=1000
```

默认链路不启用地面或离群点过滤。scan-to-local-map 前端使用内部 `0.10 m` 点云和
轮速 + IMU 运动初值；二维射线投影使用 `0.05 m` 点云。实验性过滤的对照与结论见下一节；
后续若需要通用地面分类，必须先建立不依赖当前水平地面的输入契约与验收场景。

### 预处理固定包对照

默认配置不启用地面或离群点过滤。为避免在缺少对照时改变建图输入，包内只提供四个
可重放实验 profile：`baseline.yaml`、`low_return.yaml`、`statistical_outlier.yaml` 和
`low_return_statistical_outlier.yaml`，位于 `config/preprocessing_experiments/`。前者的
低位回波项是针对本机器人水平地面、`lidar_3d_link` 坐标系的实验代理，不能称作
通用地面分割；后者使用 PCL 统计离群点过滤。

固定包前端 A/B 使用同一个组合回归入口，只替换预处理 override：

```bash
ros2 launch slam_robot_slam_3d custom_3d_loop_regression.launch.py \
  preprocessor_overrides_file:="$(ros2 pkg prefix slam_robot_slam_3d)/share/slam_robot_slam_3d/config/preprocessing_experiments/baseline.yaml"
```

将最后的文件名替换为其余三个 profile 可复现实验。每次必须从干净的 ROS 图串行启动，
并同时运行 `preprocessing_regression` 收集 1000 帧的点数与耗时；不得用 RViz 地图
观感替代前端误差、退化比例和处理预算。2026-09-02 的四组结果拒绝了三种过滤组合，
默认输入保持不变；数值与判定见[性能与标定](performance.md#固定包地面与离群点-ab)。

## 自研 scan-to-local-map 前端

固定包验证分三个终端运行：

```bash
ros2 launch slam_robot_slam_3d custom_3d_front_end.launch.py

ros2 launch slam_robot_slam_3d play_3d_slam_data.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=1.0

ros2 run slam_robot_slam_3d front_end_regression
```

前端采用标准 PCL GICP，并以 `/odom` 的帧间增量作为初值；它不是把 EKF 位姿
直接当作激光结果。关键帧按平移/旋转阈值插入，局部子图只保留最近 12 帧并
再次体素化，因此运行时间和内存不会随全局路程无限增长。子图版本不变的
普通扫描复用 GICP 目标点云和目标协方差，只有新增关键帧或重初始化后才重新
计算。诊断包含匹配状态、对应点数、RMSE、原始/实际运动初值修正量、关键帧
数、目标缓存命中和完整回调耗时。最终对应点
还利用 GICP 目标邻域协方差的最小特征向量作为表面法向，构造平面运动的
`x/y/yaw` 点到平面信息矩阵。平移弱特征方向采用双阈值连续处理：严重退化
时完全保留轮速 + IMU 预测，过渡区逐步恢复 GICP 修正，强方向不受影响。
诊断同时给出特征值、弱方向、实际缩放量，以及平移、平面、偏航三个分项
退化标志。激光里程计协方差会沿实际受抑制的平移弱方向连续放大；完全不
可观测时使用各向同性大协方差，偏航退化时单独放大偏航方差。

固定包的正式时序验收使用 `rate:=1.0`。每帧点云必须由时间戳前后的两条
里程计包夹，节点在两者之间插值；后样本尚未到达时用最多 5 帧的 FIFO
暂存，而不是立即使用滞后的最近邻。过期或队列溢出会明确告警。连续 5 次
真实配准失败时，节点按当前轮速 + IMU 预测位姿清空并重播种局部子图，避免
旧子图让失败自我延续；正常回归要求该恢复计数严格为零。`rate:=2.0` 只作
计算压力测试，不能替代在线时间契约。回环后端成功提交后，节点默认发布唯一的
标准 `map -> odom`；EKF 保持唯一的 `odom -> base_footprint` 发布者。自研链路
已输出全局关键帧重放点云 `/custom_slam_3d/map_cloud` 和高度感知的标准
`/map` `OccupancyGrid`；`custom_3d_navigation_simulation.launch.py` 已用这套
输出接入 Nav2，RTAB-Map 成熟基线入口继续独立保留；
`pose_graph.publish_map_to_odom_tf:=false` 可在只评估局部输出时
显式关闭该 TF。

这个节点的四个 latched 大话题（`local_map`、`map_cloud`、`/map`、
`occupancy_probability`）都是 reliable + transient_local，其中 `local_map` 在
`localization` 模式下装的是整张恢复地图（实测 110,544 点，约 `3.4 MiB`），
`map_cloud` 约 `1.2 MiB`，都超过 Fast DDS 给共享内存段的默认 `512 KiB`。段被写满
丢掉一个分片后，可靠读端要等满默认 `3 s` 心跳周期才发现缺口，表现是某一次地图更新
卡住几秒而所有小话题照常——看起来像算力问题，其实是传输配置。
`custom_3d_front_end.launch.py` 因此给这两个节点注入
`config/fastdds_large_cloud.xml`（`64 MiB` 段，同时显式保留 UDPv4，因为声明用户
transport 会关掉内建的那套）。已经自己导出 `FASTRTPS_DEFAULT_PROFILES_FILE` 的
操作者保留自己的 profile；`dds_profiles_file:=` 可显式指定。
`test_large_cloud_transport` 静态守住这条：仓库里每一个 durable writer 要么在被
注入 profile 的 launch 之内，要么在清单里附上它为什么装得下默认段的实测依据，
新增一个 writer 在做出这个判断之前不属于任何一类。

`map_cloud` 不参与实时匹配：节点按 `global_map.rebuild_keyframe_interval` 取全局
关键帧快照，在独立的有界定时批次中按优化位姿重放，并以
`global_map.voxel_leaf_size` 体素去重。回环或新关键帧在重建期间到来时只保留最新
请求，当前重建正常完成后再开始，确保前端回调不被地图重建占用。

### 自研与 RTAB-Map 的固定包对照

要在同一份录制数据上比较两条链路，各回放一趟再比对。两趟消费同一份数据，因此
轨迹、探索策略和实时率都不构成变量：

```bash
ros2 launch slam_robot_slam_3d map_projection_comparison.launch.py \
  stack:=custom  map_topic:=/map \
  output:=/tmp/census_custom.json  trajectory_output:=/tmp/traj_custom.json
ros2 launch slam_robot_slam_3d map_projection_comparison.launch.py \
  stack:=rtabmap map_topic:=/rtabmap/map \
  output:=/tmp/census_rtabmap.json trajectory_output:=/tmp/traj_rtabmap.json \
  database_path:=/tmp/rtabmap_comparison.db
ros2 run slam_robot_slam_3d stack_comparison_report \
  /tmp/traj_custom.json /tmp/traj_rtabmap.json \
  --reference-grid /tmp/census_custom.json \
  --candidate-grid /tmp/census_rtabmap.json
```

每趟同时产出两份 JSON。`map_projection_census` 记录二维投影：自由/占据格数与
面积、`free/(free+occupied)`、已知包围盒和 `/map` 发布间隔。未知格数只作记录不作
判据，因为栅格范围随观测边界增长，建图越多的一侧包住的未知格子也越多。
`stack_trajectory_census` 记录另外三维：相对真值的轨迹漂移、回环事件和资源开销。
只看投影时仍可直接用 `map_projection_compare` 比两份普查。

资源采样按 `/proc/<pid>/cmdline` 的可执行文件字段定位进程，不扫描参数文本；自研链路聚合
`point_cloud_preprocessor_node` 与 `scan_to_map_odometry_node`，RTAB-Map 则采样其单一
`rtabmap` 进程。报告若找不到唯一目标或运行中丢失任一进程，会把资源行标为缺失/部分，
不会静默换成另一个旧实例。

完整进程采样的三趟复测、逐次资源数据和限制条件见
[固定包资源复测结果](results/2026-09-03-stack-resource-validation.json)；轨迹与前一批合并后的
六趟观察见[性能与标定](performance.md#完整进程资源复测与六趟轨迹观察)。

三处设计决定了两条链路是否真的可比：

- **各自锚定自身首样本。** map 系与真值原点无关，不锚定得到的是原点偏移而不是
  漂移。两侧用同一种锚定，误差才能互相对读。
- **按真值时间戳查 TF，而不是取最新变换。** `map -> odom` 只在后端提交时更新，
  最新变换可能比要配对的真值样本还旧。
- **RSS 不是一条链路的全部开销。** RTAB-Map 把地图放在 sqlite 里，这部分从不出现
  在 RSS 中，而自研链路把等价内容留在进程内存。报告因此把数据库大小列在 RSS
  旁边；只比两个 RSS 会把 RTAB-Map 挪到磁盘上的内存算成它的优势。

回环事件不给比值列：自研报告的是已提交的位姿图约束，RTAB-Map 报告的是
proximity 检测，二者计的不是同一种东西。

要单独衡量前端漂移，两侧都要先关掉回环，否则误差比的是回环质量而不是前端。两个
开关都是单行配置改动，不需要改代码：自研把
`loop_closure.scan_context.minimum_travel_distance` 抬到长于整条轨迹，检索候选恒
为空；RTAB-Map 把 `RGBD/ProximityBySpace` 置 `false`——本项目不订阅图像，proximity
是它唯一的回环来源，而 `RGBD/NeighborLinkRefining` 保持开启以保留它的前端 ICP。

单趟结果不能作为结论。固定输入不固定异步回环校验与位姿图提交的顺序，因此一次运行
的回环计数和误差都只是一次抽样。重复若干趟后用分布汇总：

```bash
ros2 run slam_robot_slam_3d stack_comparison_distribution \
  --profile custom=/tmp/traj_custom_1.json,/tmp/traj_custom_2.json,/tmp/traj_custom_3.json \
  --profile rtabmap=/tmp/traj_rtabmap_1.json,/tmp/traj_rtabmap_2.json,/tmp/traj_rtabmap_3.json
```

汇总取中位数并在旁边给出实测范围：三到五趟里一次主机抖动就足以把均值推到能凭空
造出或抹掉一个差异，而重复运行的目的正是不让这种事悄悄发生。没有采到轨迹样本的
运行会被排除并显式标注，不会被静默平均掉。两条链路的 RMSE 比值按实测运行的最好与
最坏配对给出区间，而不是一个数字。

该入口需要先按本文开头录制固定包。

自研状态使用版本化单文件快照保存，写入同目录临时文件，`fsync` 后原子替换目标，
再 `fsync` 一次父目录。`rename` 替换的是目录项，它本身并不保证 44 MB 数据已经落到
存储上，重命名这个动作也一样。恢复
`mapping` 模式时会重建最近局部子图、Scan Context 索引、已提交回环约束和两类
地图输出，再继续追加关键帧；`localization` 模式使用完整全局点云作只读匹配
目标，不新增关键帧或更新地图。当前只定位入口的初始位姿契约是保存时末端位姿，
尚不提供 kidnapped-robot 式全局重定位。

当前格式为版本 5，每个关键帧分别保存 `0.10 m` 注册点云和 `0.05 m` 占据点云。
版本 1/2/3/4 加载时明确报错并要求重新建图。对 v1/v2，降级读取是做得到的——旧文件
里那份 `0.10 m` 点云可以同时充当两份，恢复出的占据栅格与双分辨率改造之前逐格
相同——但项目当前没有外部使用者、存量快照只有个位数，为此长期维护一条兼容分支
不划算。这是一个明确的取舍，不是"数据不足以恢复"：代价是升级即作废全部已有
快照，节点会 FATAL 退出而非降级。v3 被拒的原因不同：它先后以两种不兼容的字节
布局发布过（占据投影契约后来被插在版本字段与关键帧数之间），旧 v3 在新布局下
会把关键帧数当成契约读，报出的又是"契约无效"，指向完全错误的方向。版本号存在
的意义就是让布局变更自己声明出来，因此这次改为递增而不是复用。恢复时全部历史关键帧的前端位姿会被刚体变换到地图系，否则跨越
接缝的第一条位姿图边会把整个累积修正当成一次测量。

版本 5 在负载之后追加 8 字节 FNV-1a 校验和，覆盖它前面的每一个字节；校验和是
把文件写完、`fsync` 之后**重新读回来**算的，不是对内存缓冲算的——对缓冲算只能
证明缓冲，而要防的恰恰是写出去的字节没有完整到达。加载时先看 magic 和版本，
再验校验和：v4 文件没有校验和尾部，它的最后 8 字节是负载，先验校验和会把一个
只是旧了的文件报成损坏，重复 v3 那次"布局变更表现为契约无效"的误导。这个顺序
由 `ReportsAVersionFourFileAsAVersionRatherThanAsCorruption` 锁定。

占据点云在平面模式下只保存最大投影距离内的地面自由射线和高度带内障碍点；
高于障碍带的回波在二维投影中既不清自由空间也不标障碍，因此不会进入关键帧和
快照。距离和高度分类使用同一个投影函数，不再由节点和栅格各自实现。v4 文件还
保存输入体素、高度上下限、最大射线距离和 `force_planar_motion`；加载时必须与
当前参数一致。非平面模式无法提前证明某个点在未来优化位姿下仍然带外，因此保守
保留完整占据输入。

这条恢复路径此前只有单元覆盖：文件格式、恢复位姿的重基准各自成立，但没有任何
入口跑过“存档 → 重启 → 继续建图”整条链路——所有 launch 的 `load_snapshot`
默认都是 false。`snapshot_resume_regression` 用同一个固定包把它分两段跑完：

```bash
ros2 run slam_robot_slam_3d snapshot_resume_regression \
  --bag "${SLAM_WS}/bags/structured_3d_reference" --split 240.0
```

前段建图到第 `240 s`，通过 `~/save_snapshot` 保存——与探索器完成时调用的是同一个
服务，因此覆盖的是生产写入路径，也不依赖关闭时恰好留了多少时间。后段用全新前端
以 `load_snapshot:=true` 启动，从 `--start-offset 240` 放完余下的包。快照和两段
报告都写在 `--report-directory` 下，不碰用户主目录里的那一份。

默认 `--split` 取 `240.0` 而不是折半，因为固定包是两圈：第一圈内没有回环，切在
`180 s` 会让前段零提交。两段回放还需要 `recorded_static_tf_publisher`——包里
`/tf_static` 只有开头那一条，`--start-offset` 会跳过它，缺了它前端会一直等雷达外参
而不处理任何扫描。

关键在锚点。本仓库其它轨迹判据都把估计锚到它自己的第一帧，因为地图系与真值系的
原点本就无关；这里照做恰好会让判据失效——恢复到错误位姿的一段会把跳变吸收进自己
的新原点，测出零误差。所以前段把锚点写进报告，后段用**前段的**锚点度量，接缝上的
任何不连续都直接显示为误差。

判据里有两条问的不是“通过没有”，而是“这次运行有没有测到东西”：保存前若没有位姿图
提交，存下来的坐标修正就是单位阵，重基准等于什么都没做；重启后若没有提交，跨接缝
的那条边就从未进过优化器。两种情况都判 FAIL 并提示调整 `--split`，而不是报一个
没挣到的 PASS。

`localization` 模式与 `mapping` 共用同一份快照文件，除此之外几乎没有共同点：恢复
时走另一条分支——整份全局点云直接替换局部地图成为匹配目标，而不是滚动的 12 帧
子图——随后禁止一切写入（不插关键帧、不进位姿图、不做子图重初始化、关闭时不存
快照）。单元测试覆盖的文件格式与位姿重基准是两种模式共有的部分，**使这条分支与众
不同的地方此前没有任何入口跑过**。补上的入口是：

```bash
ros2 run slam_robot_slam_3d snapshot_localization_regression \
  --bag "${SLAM_WS}/bags/structured_3d_reference"
```

固定包是两圈同一回路，这正是它可测的原因：前段建图第一圈，先验地图因此已经覆盖
第二圈要走的地面；后段以 `mode:=localization` 重放第二圈。

**只测精度证明不了什么。**一个每帧匹配都失败、退回轮速推算的前端，在一圈固定数据
上看起来也会足够准。所以匹配接受率是独立判据：估计必须来自与恢复点云的配准。只读
这条同样不是"最后看地图有没有变"，而是"整轮里出现过几种不同的局部地图尺寸"——
恢复安装了一份点云，之后再没动过就是 1，动过就大于 1。快照文件本身则比对
`(大小, SHA-256)`：这个守卫写错会覆盖掉机器人正在赖以定位的那张图。用内容哈希
而不是 `(大小, mtime)`，是因为后者答的是"这个文件被碰过吗"，而要断言的是"机器人
定位时用的那张先验图还是保存下来的那张"——一次长度不变、落在同一个文件系统时间戳
刻度内的重写，前者看不出，后者看得出。

前端每次插入关键帧时还会写入独立的长期全局关键帧库：其中保存注册/占据两份
传感器坐标系点云、时间戳、`x/y/yaw` 前端位姿、插值 `/odom` 预测、传感器
外参与协方差/退化诊断。它不参与实时 GICP 的目标构建，也不会因本地 12 帧
子图淘汰而丢失数据；`front_end_regression` 会检查诊断中的
`global_keyframes`、注册点数与占据点数。Scan Context、GICP 回环复核、局部子图
和三维全局地图只消费注册点云；高度感知二维栅格只消费占据点云。

当前已接入 Scan Context 风格的极坐标高度描述子检索：ring key 先筛选历史
候选，再按扇区循环位移比较完整描述子并输出预测偏航。候选必须跨过关键帧
间隔和累计行程门限；不能按当前估计位置排除历史帧，因为真正的闭环恰好会
回到历史位置附近。诊断会给出候选数量、最佳候选 ID、描述子/ring-key 距离和
预测偏航。描述子使用无量纲的归一化余弦距离；检索还使用
`loop_closure.scan_context.maximum_descriptor_distance` 过滤明显不相似的地点提议。
默认 `0.05` 由固定包真回环和重复走廊假回环 A/B 共同确定，这只是 GICP 前的廉价
质量门槛，不替代后续几何复核。

每个新关键帧的前 N 个地点候选会构建候选帧附近的全局关键帧子图，并以 PCL
GICP 复核。复核必须满足 GICP 的收敛、对应点、RMSE 与最大修正量门限，同时
满足最小扫描重叠率且不处于平移、平面或偏航弱几何状态；诊断保留验证数量、
接受数量、最佳验证状态与重叠率。检索与复核在单一后台任务中顺序执行；通过
复核的约束会排队给另一后台位姿图任务；每个当前关键帧最多保留最佳一条，两次
接受还必须满足 `minimum_constraint_keyframe_interval`。成功提交后才以严格 SE(2) 修正更新
`map -> odom`。检索没有按当前前端位置的硬距离门限；复核后才检查与前端相对
运动的一致性，从而既允许数米漂移后的真实回环，也拒绝重复走廊中的巨大假修正。

专用 70 m 平行墙世界把端墙和少量锚点移出雷达量程，用下面的自动回归验证
严格弱几何段及进入/离开退化区的状态切换：

```bash
ros2 launch slam_robot_slam_3d corridor_3d_regression.launch.py
```

这与 2D 使用的 `degenerate_corridor.sdf` 是两套场景；3D 版本针对 20 m
量程和 16 线扫描单独设计，不用结构丰富的 `structured_loop_3d.sdf` 冒充
秩亏环境。

正常结构中的快速旋转与单侧轮胎打滑使用：

```bash
ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=rotation

ros2 launch slam_robot_slam_3d front_end_motion_regression.launch.py \
  profile:=slip
```

两种 profile 都检查匹配接受率、真值误差、相对 `/odom` 劣化、退化误报、
意外子图重初始化、回调间隔和 P95。打滑 profile 还要求低摩擦实际造成可测
的里程计误差。

### 自研回环正反验收

正向验收回放结构化两圈固定包，并要求至少一次成功的后台图优化且零丢弃、零
后台异常：

```bash
ros2 launch slam_robot_slam_3d custom_3d_loop_regression.launch.py \
  bag:="${SLAM_WS}/bags/structured_3d_reference" rate:=1.0
```

负向验收仍运行 `corridor_3d_regression.launch.py`，要求弱几何段没有任何位姿图
提交。两者分别验证“真实闭环能进入后端”和“重复结构不会污染后端”。

### 复合启动冒烟回归

任何修改复合 launch、作用域、延迟事件处理器或启动参数后，先运行：

```bash
ros2 run slam_robot_slam_3d launch_smoke_check
```

它以无界面模式串行启动走廊前端、运动前端、自研固定包闭环、结构化 RTAB-Map、RTAB-Map +
Nav2、结构化数据录制六个入口。每个入口必须在关键节点连续在线 `60 s` 后
才通过；脚本检查启动日志中的作用域和子进程错误，并以独立进程组回收全部
子进程。数据录制的短暂 bag 输出位于临时目录，结束后自动清理。单独复测某个
入口时可用 `--profile structured_navigation`；默认超时和保持时长都可通过
`--startup-timeout`、`--hold-time` 调整。
首版 GICP、关键帧和局部子图参数由固定包、长走廊、快速旋转和打滑四类测试
共同约束；进入回环后端阶段后，修改这些参数必须复跑该集合。

## RTAB-Map 在线 3D SLAM

启动 3D 机器人、轮速 + IMU EKF、点云检查、RTAB-Map 和 RViz：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py
```

另开终端驾驶：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

RViz 的 `RTAB-Map Cloud` 显示累计三维地图；`Current 3D Scan` 显示实时
`/lidar_3d/points`。RTAB-Map 同时发布 `/rtabmap/map` 二维占据栅格，供
Nav2 使用。

默认 `reset_database:=true`，每次启动删除旧数据库并从新地图开始。正常退出
后数据库保存在 `~/.ros/rtabmap_3d.db`。继续已有地图时才使用：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  reset_database:=false
```

无界面运行：

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_simulation.launch.py \
  gui:=false rviz:=false
```

这是一台平面差速机器人，因此 RTAB-Map 强制轨迹为 `x/y/yaw` 三自由度。
该约束只作用于轨迹和 `map -> odom`，累计点云仍为三维；不能关闭后继续将
同一地图用于 Nav2，否则微小横滚/俯仰误差会把远处地面误标成障碍。

当前 Gazebo 点云没有逐点时间字段，不能做严格 deskew。因此该链路是低速
几何 ICP + 回环 + 位姿图基线，不是完整 LIO。详细参数与 QoS 约束见
[3D SLAM 包说明](../src/slam_robot_slam_3d/README.md)。

## RTAB-Map + Nav2 在线导航

一条命令启动在线建图和导航：

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py
```

该入口不启动 Map Server 或 AMCL。RTAB-Map 发布 `map -> odom` 和实时二维
投影地图；Nav2 使用该地图做全局规划，并直接把 3D 点云送入局部体素障碍层。
RViz 同时显示累计 3D 地图、实时点云、二维栅格、代价地图和路径。

障碍物安全扫掠高度为 `0.05–0.45 m`：机器人含雷达总高 `0.35 m`，另留
`0.10 m` 余量。这个二维导航模型会绕开安全高度内的最大障碍外轮廓，同时
允许通过净空高于 `0.45 m` 的门洞。

需要单独调试 Nav2 时，先确保 RTAB-Map 已运行，再启动：

```bash
ros2 launch slam_robot_navigation online_slam_navigation.launch.py
```

地图形成后可检查二维栅格契约：

```bash
ros2 run slam_robot_slam_3d grid_contract_check
```

## 结构化世界与自动验收

手动在 3D 结构化世界中建图和导航：

```bash
ros2 launch slam_robot_slam_3d rtabmap_navigation_simulation.launch.py \
  world:=$(ros2 pkg prefix slam_robot_gazebo)/share/slam_robot_gazebo/worlds/structured_loop_3d.sdf
```

世界包含立体墙面、斜柱、高门洞和下细上粗柱。Gazebo 中可以拖动视角；RViz
使用可旋转 Orbit 视角。

两圈闭环建图回归：

```bash
ros2 launch slam_robot_slam_3d structured_loop_regression.launch.py
```

建图后继续执行 Nav2 高度语义验收：

```bash
ros2 launch slam_robot_slam_3d structured_navigation_regression.launch.py
```

完整验收要求：

- 形成真实 proximity 回环和非零全局修正。
- `map -> odom` 高度峰值不超过 `0.02 m`，横滚/俯仰不超过 `0.5°`。
- 机器人通过净空 `0.55 m` 的门洞。
- `0.07 m` 细底座上方的 `0.40 m` 柱帽按最大外轮廓进入代价地图。
- 真实轨迹与柱心保持至少 `0.55 m`，并产生至少 `0.25 m` 绕行。

自动回归默认无 Gazebo 和 RViz 窗口，这是为了减少资源占用；人工观察时使用
上面的手动启动命令。测量结果见 [性能与标定](performance.md)。

## MOLA 纯激光里程计对照

```bash
ros2 launch slam_robot_slam_3d mola_lo_simulation.launch.py
```

该入口关闭 deskew，不生成伪造的逐点时间，也不使用真值、轮式里程计或 IMU
作为位姿输入。`use_imu_gravity:=true` 仅提供重力方向先验，不等同于完整
LIO。MOLA 输出局部里程计和局部点云，不提供与 RTAB-Map 等价的完整在线全局
SLAM 能力。

## 自主 Frontier Exploration

RTAB-Map 是成熟算法验收基线；自研链路已经完成点云预处理、GICP 前端、
弱几何退化处理、Scan Context + GICP 回环、后台位姿图、全局点云地图、二维
导航投影和版本化快照恢复。现有在线入口由自研 SLAM 独占 `map -> odom` 并
持续发布 `/map`，Nav2 继续使用 3D 点云局部避障；RTAB-Map 基线及其回归场景
继续保留用于同场景对照。

探索器从实时占据栅格提取自由区与未知区边界，按连通簇、障碍净距、信息增益
和机器人距离生成候选，再调用 Nav2 `ComputePathToPose` 过滤不可达候选并用
实际路径长度重评分。最终目标只通过 `NavigateToPose` 发送；探索器不发布
`/cmd_vel`。地图使在途目标不可通行时会取消并重规划，失败目标进入带过期时间
的空间黑名单。

自研 3D SLAM 完整入口：

```bash
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py
```

默认沿用显式原点，便于复现历史基准。开启通用 SDF 安全随机出生：

```bash
ros2 launch slam_robot_slam_3d custom_3d_exploration_simulation.launch.py \
  world:=$(ros2 pkg prefix slam_robot_gazebo)/share/slam_robot_gazebo/worlds/large_warehouse.sdf \
  random_spawn:=true spawn_seed:=42
```

`safe_spawn_sampler` 解析 SDF 的 collision，而不是 visual 或手写坐标清单。它先按
机器人外接圆 `0.336 m`（Nav2 footprint 多边形的外接圆，由契约测试锁定）加
`0.15 m` 平面余量膨胀障碍，再检查从地面到 `0.48 m` 的垂直扫掠区间（`0.03 m`
生成抬升 + `0.35 m` 机器人 + `0.10 m` 余量），因此低横梁会阻挡出生，高于扫掠体
的门洞不会被误杀。最后只保留参考原点所在的四连通自由区，批量采样还要求点间至少
相距 `2 m`（`spawn_minimum_separation` 可调）。

**能力边界写在契约里**，不是笼统的“任意 SDF”：

- **单层共面支撑面。** 以参考点所在平面为唯一可行驶层；与它高差超过可通过台阶
  （`0.025 m`）的薄平面几何一律降级为障碍并完整膨胀。因此齐平的分块地面连通，
  `0.08 m` 门槛是障碍，而 `0.70 m` 的门楣仍可从底下开过去——降级本身不阻挡，
  垂直区间才决定。参考点下方存在多个不同高度层时**明确拒绝**，不静默取最高层。
- **非静态模型按 SDF 初始位姿当障碍**，并额外加 `0.25 m` 余量。Gazebo 以 `-r`
  启动，采样完成到机器人生成之间动态物体可能已经移动，所以这只保证初始布局的
  保守避让，不预测运行后轨迹。需要更严时用 `--reject-non-static`（当前三个验收
  世界全静态，开启无代价）。
- **栅格范围由墙体决定**，取地面 ∩ 障碍包围盒外扩，并吸附到以世界原点为锚的
  格网，因此小于一格的墙体编辑不会平移全局格心。

采样器把整条记录以一行 JSON 输出（当前 `schema_version: 4`），除种子和位姿外还
包含 `parameters`（分辨率、外接圆半径、平面余量、整机高度、垂直余量、最小间距、
生成抬升、参考点、非静态额外余量、可通过台阶、栅格上限）、`world_parsing_policy`、
`world_geometry`、`support_resolution` 与 `sampling_bounds`。**复现位姿需要种子与
这整组参数**：其中任何一项变化都会改变安全格集合，进而改变同一种子给出的位姿。
`spawn_z` 是**相对参考地面的生成抬升**，不是绝对高度。

交互式启动把同一条记录以 `SAFE SPAWN RECORD ...` 打进日志，campaign 则把它整体
写入 `campaign_summary.json` 的 `spawn_sampling`；campaign 另外记录世界文件的
SHA256 与 `source_revision`（commit 与 `dirty` 标志），并**拒绝低于 schema 4 的
采样器**——接受旧记录等于继续放行"不完整却看起来可复现"的正式证据。

两条关于记录含义的限制：

- **SHA256 才是世界的内容标识，路径只是定位线索。**记录里的路径归一化为
  `package://slam_robot_gazebo/worlds/xxx.sdf`，无法识别为包内资源时退化为裸
  文件名；绝对路径不写入，因为它指向个人目录或 `install/` 这种构建产物位置。
- **`source_revision` 是定位源码的线索，不是复现保证**：C++ 节点从 `install/`
  运行，构建时间早于运行时间，因此工作区干净并不证明当时跑的二进制就由该提交
  编出。

连续若干周期没有未拉黑的可达 frontier，或连续到达目标但已知自由区不再增长，
且已知自由单元达到下限后，节点发布 `/frontier_explorer/complete=true`，并调用
自研 SLAM 快照服务保存 `~/.ros/custom_slam_3d.snapshot`。不可达候选会进入长于
导航超时的空间黑名单，避免多个永久不可达的残余边界循环轮换。成熟基线使用同一
探索器，仅把地图输入切换到 `/rtabmap/map`；RTAB-Map 自行持续保存数据库：

探索器仍保留 `exploration_seed` 与近优候选随机选择机制，但当前正式配置把
`selection_top_score_band_fraction` 设为 `0.0`，所以总是选择评分最高的可达候选，
种子不会改变目标序列。路线多样性来自 `random_spawn:=true` 的安全随机出生；需要
复现整次运行时应固定 `spawn_seed`。历史随机候选 A/B 及停用原因见性能记录。
探索完成时终端和 RViz 会立即提示，快照保存成功后再提示可以按 `Ctrl+C`，仿真
不会自动退出，以便继续检查最终地图。若 Gazebo 在探索完成前被关闭或异常退出，
公共仿真 launch 会终止其余节点；若 Nav2 action server 在运行中失活，探索器会
发布 ERROR 诊断并显示 `EXPLORATION ABORTED`，不累计假失败、不发布
`complete=true`，也不保存快照。

二维导航投影只将 `0.05–0.45 m` 高度带内的回波写成障碍；低于带下限的地面回波
只提供射线自由空间证据，高于带上限的回波不投影，避免二维地图错误清除低处障碍。
常规建图仅增量整合新关键帧；仅在位姿图回环提交或快照恢复后，以分批全量重建
修正过的地图。

自研投影内部保留概率栅格，并把它单独发布到
`/custom_slam_3d/occupancy_probability` 供诊断；`/map` 则始终是 Nav2 可消费的
三态 `-1/0/100` 栅格。默认自由阈值为 25：三次独立自由射线后可通行；介于
自由和占据阈值之间的部分证据保持未知，绝不会被探索器猜成自由空间或障碍物。
每个新全局关键帧都会请求一次增量投影更新，避免把探索决策建立在旧地图上。

探索回归必须先收到本次运行的 `complete=false` 和地图，才接受 `complete=true`，
从而隔离 DDS transient-local 的历史完成消息。固定 `structured_loop_3d.sdf` 场景
以 RTAB-Map 的 76,012 个自由单元和 70.778 m 真值行程为参考，自研链路至少需达到
38,000 个自由单元与 35 m 行程；快照服务不可用或保存失败也会直接判失败。路径
规划、导航和取消请求分别有墙钟看门狗，因此 TF 暂时不可用不会阻塞超时恢复。

上述门槛是防退化闸门，不是两条链路的精度对比。固定包对照表明同轨迹下自研投影
产出基线 98% 的自由空间；在线回归还验证了概率三态投影与及时地图交付后，自研链路
可在 900 s 预算内自行完成探索。完整的历史问题、因果和测量结果见
`docs/performance.md` 的“探索覆盖与栅格投影整改”。

```bash
ros2 launch slam_robot_slam_3d rtabmap_3d_exploration_simulation.launch.py
```

无界面自动回归分别为：

```bash
ros2 launch slam_robot_slam_3d frontier_exploration_regression.launch.py
ros2 launch slam_robot_slam_3d rtabmap_frontier_exploration_regression.launch.py
```

回归默认从固定出生点开始，也可由 campaign 为每轮注入预生成的随机安全位姿；
它检查至少一个导航目标成功、自由区增长、
真值行程、导航恢复预算、失败/成功目标预算、Collision Monitor 触发和进程级错误；
自研链路还限制地图已知区的宽、高、包围盒面积和子图重启次数，并检查探索完成后
快照服务能够成功保存。单圈探索不强制出现回环；正向召回由
`custom_3d_loop_regression.launch.py` 的固定包双圈回归负责。

三个世界各跑 5 次随机出生的活动验收命令示例：

```bash
ros2 run slam_robot_slam_3d exploration_campaign \
  --runs 5 --minimum-evaluable-runs 4 --random-spawn \
  --world /absolute/path/to/structured_loop_3d.sdf \
  --world-profile structured_loop_3d --campaign-seed 2026081201 \
  --log-directory campaign_logs/structured
```

`slam_world` 和 `large_warehouse` 分别改用同名 profile。摘要记录 SDF SHA-256、
实际种子、每轮出生位姿和最小点间距。Gazebo 若在评分前因 WSL D3D12/GLX 初始化
失败，campaign 会立即清理并记为 `INFRA_UNSTABLE`，不会让 Nav2 空等总超时。

## 后续路线

相机随后用于增强地点识别和全局重定位，IMU 用于支持具备逐点时间数据的
LIO；不计划融合 2D 与 3D LiDAR。详细分阶段任务和验收标准见项目根目录
`plan.md` 的 8.3、8.4 节。
