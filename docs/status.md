# 项目状态

最后更新：2026-09-02。

## 当前可用能力

- 2D：SLAM Toolbox 与自研相关匹配 SLAM 均可建图、保存地图并用 Nav2 导航；默认局部
  预测为轮速 + IMU EKF。
- 3D 成熟基线：RTAB-Map 可在线建图、生成二维导航投影，并以 3D 点云参与局部避障。
- 3D 自研：GICP 前端、回环后端、全局地图、二维投影、在线 Nav2、快照继续建图和
  只读 localization 均已接通。
- 自主探索：自研与 RTAB-Map 共用 Frontier 调度；自研链路支持自动快照和从 SDF
  几何采样的安全随机出生点。

## 最近已验证的结果

- 自研 3D `localization` 恢复固定包 v5：`11/11` 判据通过。
- 三世界随机出生独立验证：`structured_loop_3d.sdf`、`slam_world.sdf`、
  `large_warehouse.sdf` 各 `5/5 ACCEPTED`。
- Nav2 启动期与运行期故障注入均可提前归类为 `INFRA_UNSTABLE`，不会被误记为
  地图完成。

完整结果、门限和证据链接见[验收记录](acceptance.md)。

## 下一步

1. 固定输入下评估地面/离群点预处理，并与 RTAB-Map 作可比对照。
2. 验证动态障碍与完全封路时 Nav2 的安全恢复。
3. 进入相机、视觉 SLAM 与 LiDAR—视觉—IMU 融合阶段。

详细工作排序见[项目路线图](../plan.md)。
