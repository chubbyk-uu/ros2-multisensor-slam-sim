# 地图文件

本目录用于保存建图结果。建议同一次建图使用相同的文件名前缀保存：

- `*.yaml` 和 `*.pgm`：Nav2 / AMCL 使用的二维占据栅格地图。
- `*.posegraph` 和 `*.data`：SLAM Toolbox 可继续建图的位姿图与传感器数据。

统一建图 launch 默认在收到一次 `Ctrl+C` 后、关闭 SLAM 前自动保存为 `slam_map.*`。等待终端显示 `Save completed` 后即可退出；也可以通过 `map_output_prefix` 参数指定其他文件名前缀。

自研 2D SLAM 默认保存为 `custom_slam_map.yaml/.pgm`，不会生成
`.posegraph/.data`，也不会覆盖 SLAM Toolbox 的默认文件。
