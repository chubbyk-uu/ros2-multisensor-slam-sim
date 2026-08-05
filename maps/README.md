# 地图文件

本目录用于保存建图结果。建议同一次建图使用相同的文件名前缀保存：

- `*.yaml` 和 `*.pgm`：Nav2 / AMCL 使用的二维占据栅格地图。
- `*.posegraph` 和 `*.data`：SLAM Toolbox 可继续建图的位姿图与传感器数据。

统一建图 launch 默认在收到一次 `Ctrl+C` 后、关闭 SLAM 前自动保存为 `slam_map.*`。等待终端显示 `Save completed` 后即可退出；也可以通过 `map_output_prefix` 参数指定其他文件名前缀。

自研 2D SLAM 默认保存为 `custom_slam_map.yaml/.pgm`，不会生成
`.posegraph/.data`，也不会覆盖 SLAM Toolbox 的默认文件。

## 本目录与 `reference/` 的区别

本目录根下的所有地图都是**每次运行的产物**，不进入版本控制。这样按一次
`Ctrl+C` 触发自动保存不会弄脏工作区，也不会把某次随手跑出来的地图变成
仓库资产。

`reference/` 下是**策展过的演示地图**，随仓库分发，让 clone 之后不必先
建图就能直接跑导航看效果：

- `reference/slam_map.yaml/.pgm`：SLAM Toolbox 基线建出的地图。
- `reference/custom_slam_map.yaml/.pgm`：自研 2D SLAM 建出的地图。

导航入口的默认地图仍然是本目录根下的 `maps/slam_map.yaml`，即"自己建的
图"，所以正常的**建图 → 导航**流程不需要任何额外参数。只有在还没建图、
想直接使用演示地图时才需要显式指定：

```bash
ros2 launch slam_robot_bringup navigation_simulation.launch.py \
  map:="${SLAM_WS}/maps/reference/slam_map.yaml"
```

`reference/` 只保留 `.yaml/.pgm`，不保留 `.posegraph/.data`：位姿图和传感器
数据合计约 `40 MB`，且只对产生它的那台机器的"继续建图"有意义，别人 clone
下来无法接着用。要更新演示地图，把满意的一次建图结果显式复制过去：

```bash
cp maps/slam_map.yaml maps/slam_map.pgm maps/reference/
```
