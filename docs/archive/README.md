# 文档历史档案

本目录保留不再作为当前入口维护、但仍需追溯的完整历史材料。归档不是失效：其中的
数字、A/B 对照和整改细节仍可作为证据引用。

| 文件 | 内容 |
| --- | --- |
| [performance-chronicle.md](performance-chronicle.md) | 原逐日性能、验收、方法与故障记录，已由主题文档重新索引 |
| [project-plan-history.md](project-plan-history.md) | 已完成阶段的详细任务清单与历史计划 |
| [2026-07-30-review-remediation.md](2026-07-30-review-remediation.md) | 2D 工程审查整改主表，以及随后补充的审查历史 |

## 迁移对照

`performance-chronicle.md` 是重构前 `docs/performance.md` 的完整逐行历史记录，
仅修正了一条因移动目录产生的相对链接。当前主题页不删除其中任何数字或结论，而是
把日常阅读入口按用途分开：

| 历史材料类型 | 当前入口 |
| --- | --- |
| CPU、内存、回调耗时、QoS、轮距与 IMU 标定 | [性能与标定](../performance.md) |
| 成熟基线、自研链路、快照、导航与探索是否通过 | [验收记录](../acceptance.md) |
| 三态投影、地图拉伸、launch、宿主污染与快照故障 | [故障归因与工程事件](../incidents.md) |
| 固定包、A/B、重复运行、随机化与故障注入的设计依据 | [验证方法](../methodology.md) |

当当前主题页只引用结论而需要完整表格、原始对照或逐日上下文时，应回到这份
chronicle，而不是重新转录数字。

当前入口请从[文档索引](../README.md)开始。
