# 验收记录

本页回答“每条链路凭什么算通过”。启动命令与操作流程见各工作流；性能数值见
[性能与标定](performance.md)，故障归因见[工程事件](incidents.md)。原始逐日表格和
所有历史上下文仍保留在[性能历史档案](archive/performance-chronicle.md)。

## 验收原则

- 验收输入、评分与真值独立于待测估计链路；真值仅用于离线评分。
- 同一条 `map -> odom` 同时只能有一个发布者。
- “能启动”不是验收：需要检查话题、TF、运行期事件、地图契约和结果退出码。
- 可重复的活动验收使用版本库内 JSON 摘要；历史临时日志会明确标注不可复核。

## 2D 官方与自研链路

2D 官方基线完成 SLAM Toolbox 建图、地图自动保存、Map Server、AMCL 与 Nav2。
自研链路完成扫描匹配、关键帧局部子图、后台回环、Ceres 位姿图和优化射线重放。

自研 2D 的验收覆盖 12.57 m 两圈闭环、三档原地旋转、25 m 退化走廊、50 m
走廊往返、重复结构、155.2 m 大场景和固定 MCAP 重放。每类均检查误差、前端
间隔、回环/重建事件以及严重日志；走廊还要求物理弱几何中段的退化检出率，正常
旋转与闭环则要求零退化误报。详见[历史档案](archive/performance-chronicle.md#性能优化)。

## 成熟 3D 基线与结构化导航

MOLA 用作纯 LiDAR 里程计对照；RTAB-Map 作为 ICP、proximity 回环、位姿图和
二维导航投影的成熟基线。RTAB-Map + Nav2 的结构化世界回归要求：

- 3D 点云地图、二维投影和 `map -> odom` 均可用；
- 平面机器人约束下 `map -> odom` 的高度、横滚、俯仰受限；
- 下细上粗柱按上部轮廓绕行；净空足够的高门洞允许通过；
- 目标到达、碰撞与恢复预算、进程收尾均被记录。

首次接口烟雾、两圈回归和高度语义导航的完整结果见
[历史档案](archive/performance-chronicle.md#rtab-map-在线-3d-slam-集成烟雾验收)。

## 自研 3D 前端、回环与全局地图

自研 3D 前端的受控回归覆盖固定输入、快速旋转、打滑和弱几何长走廊。弱方向
处理要求降低纵向漂移且不冻结正常二维几何；回环后端要求 Scan Context 检索、
GICP 复核、后台 SE(2) 图优化和地图重建均完成，而不阻塞 10 Hz 前端。

完整固定包验收确认：输入、真值与静态 TF 可独立重放；回环、地图、路径均由当前
代码重新生成；验收同时检查回环提交、地图重建、误差、前端间隔与严重日志。
详情见[历史档案](archive/performance-chronicle.md#自研-3d-闭环完整固定包验收)。

## 快照恢复与只读 localization

快照恢复验收分开验证两种语义：

- `mapping`：存档、重启、恢复后继续追加关键帧和地图；
- `localization`：恢复整张全局点云作为匹配目标，禁止关键帧、局部地图、位姿图
  和快照文件写入。

`mapping` 恢复以固定包 240 s 处切分录制与恢复两段，结果 JSON 为
[`2026-08-11-snapshot-resume.json`](results/2026-08-11-snapshot-resume.json)。

`localization` 固定包两圈验收 v5 为 `11/11`：匹配接受率 `1213/1213`，接缝
峰值 `0.013128980774705309 m / 0.07628954263957714°`，恢复地图的已知格与保存时
一致，快照 `(大小, SHA-256)` 不变。结果 JSON 为
[`2026-09-02-snapshot-localization-v5.json`](results/2026-09-02-snapshot-localization-v5.json)；
校验和尾部引入前的 v4 基准为
[`2026-08-12-snapshot-localization.json`](results/2026-08-12-snapshot-localization.json)。
两次的位置误差、峰值误差和栅格计数逐位一致（`peak_position_error_m` 同为
`0.013128980774705309`，`restored_known_cells` 与 `final_known_cells` 同为
`79367`）；只有接缝偏航峰值因接缝采样点数不同（`678` / `715`）在第七位小数上有
差异。据此可以认为 v5 只增加了持久化，没有改变估计本身。

## 自研与成熟基线的可比对照

对照不是验收判据，但它决定“自研是否够好”能否被回答。固定包上开回环累计各六趟，
以录制里程计为对照；首批还各自关闭回环三趟隔离前端。结论是：开回环时 RTAB-Map 的
位置 RMSE 为自研的 `2.25x`–`4.43x`，隔离前端后为 `13.42x`–`17.47x`；自研的精度
不来自回环，RTAB-Map 的
几乎全部来自 proximity 检测。结果 JSON 为
[`2026-09-03-stack-comparison-distribution.json`](results/2026-09-03-stack-comparison-distribution.json)，
完整进程资源复测为
[`2026-09-03-stack-resource-validation.json`](results/2026-09-03-stack-resource-validation.json)，
被它取代的单次运行版本为
[`2026-09-03-stack-comparison.json`](results/2026-09-03-stack-comparison.json)。

结论限于 `structured_loop_3d` 这一条闭合两圈轨迹：没有重访的路线不会触发回环，因此
不能据此断言任意场景下的排序。

## Frontier Exploration

探索器只通过 ComputePathToPose 与 NavigateToPose 调度 Nav2，不直接发布速度。
验收检查覆盖、地图边界、目标/恢复预算、碰撞、子图重初始化、快照保存、位姿图和
地图系位姿误差。Nav2 依赖失活必须进入可分类 fault，而不能伪装成探索完成。

三世界随机出生的两批结论必须分开读：

| 批次 | 作用 | 结论 |
| --- | --- | --- |
| 2026-08-12 标定 | 用于设定 profile 门限 | 15/15 自洽；不是独立证明 |
| 2026-08-12 独立验证 | 新 seed、完成几何整改后验证 | 三世界各 5/5 `ACCEPTED`，共 15/15 |

标定批次结果分别见
[structured](results/2026-08-12-random-spawn-structured-campaign.json)、
[slam_world](results/2026-08-12-random-spawn-slam-world-campaign.json) 与
[large_warehouse](results/2026-08-12-random-spawn-large-warehouse-campaign.json)；
独立验证结果分别见
[structured](results/2026-08-12-verification-structured_loop_3d-campaign.json)、
[slam_world](results/2026-08-12-verification-slam_world-campaign.json) 与
[large_warehouse](results/2026-08-12-verification-large_warehouse-campaign.json)。
活动验收的设计、标定限制和门限解释见[验证方法](methodology.md)。

## 动态障碍与完全封路

两个场景的判据相反，必须分开读。可绕行的动态障碍要求到达目标且无碰撞；完全封路要求
目标**不**被到达、机器人安全停下。后者通过不需要新的 verdict 类别：
`PASS`/`FAIL`/`INFRA_UNSTABLE` 已分区完备，“正确地失败”是封路场景的通过判据。

封路验收在 `blocked_road_world` 中进行，其死胡同只有一个 `1.1 m` 门洞。九项检查全部
通过：规划器给出 `NO_VALID_PATH`（`error_code=208`）、`30.1` 秒内结束（预算 `180` 秒）、
末速 `0.000 m/s`、中心到封口最小距离 `1.080 m`，扣除机器人 `0.336 m` 外接圆后净空
`0.744 m`、门洞缺口 `0.00 m`、恢复 15 次（下限 1、上限 18）、碰撞监视器介入 0 次。
结果 JSON 为
[`2026-09-03-blocked-road.json`](results/2026-09-03-blocked-road.json)。

同一份结果记录了可执行负向对照：以 `seal_offset_x:=1.5` 把封口移开门洞、其余不变，
launch 返回非零，且 `goal_not_reached`、`blockage_perceived`、
`planner_reported_no_path`、`robot_at_rest` 和 `recovery_floor` 五项核心检查同时失败。
没有它，通过只能说明这次跑成了，不能说明判据在区分什么。

尚未覆盖：3D 链路既无封路场景也无动态障碍场景；本结论也不适用于超出机器人单次视野的
封路，原因见[工程事件](incidents.md#代价地图会忘记看不见的封路)。

## 启动与故障注入验收

五个复合 3D launch 入口均有无界面冒烟检查，验证作用域、关键节点持续在线、
异常栈和子进程死亡。Nav2 启动期 configured 未 activate、以及运行期先接受目标
再失活的两种故障注入，均要求提前退出并归类为 `INFRA_UNSTABLE`；未知故障代码
不得被当作基础设施噪声放过。

两条注入的结果 JSON 为
[启动期](results/2026-08-12-nav2-fault-injection-startup.json)（`nav2_startup_timeout`，
墙钟 `46.3 s` / 预算 `300 s`）与
[运行期](results/2026-08-12-nav2-fault-injection-runtime.json)（`nav2_runtime_lost`，
墙钟 `68.8 s` / 预算 `300 s`），两者的 `fault_class` 均为 `dependency_lost`。

这两条注入验证了“探索器诊断 → 回归 verdict → campaign 汇总”整条归因链，而非
只验证各层单元测试。完整步骤和结果见[工程事件](incidents.md#nav2-依赖失活不能被解释为地图完成)。
