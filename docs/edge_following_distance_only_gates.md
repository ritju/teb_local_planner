# 贴边门控：只距离、不平行

## 结论（已确认）

`transformed_plan` 与 **mission**、与 **reference_path** 的进入/选用判定均改为：

- **只使用距离**（路径上各点到折线的距离）
- **不再使用平行度 / 夹角**

选贴边线段仍采用：参考折线上相对机器人的**最近邻段**，弧长不足则向两端扩到约 `L`（`min_wall_line_length_`）。

## 分层职责

| 环节 | 对象 | 判定 |
|------|------|------|
| `refreshActiveReferencePair` | removed_plan front ↔ mission | 配对选哪条 reference（本方案不改） |
| `shouldRunEdgeFollowingForTransformedPlan` | `transformed_plan` ↔ **active mission 折线** | 重叠 ≥ `min_path_edge_overlap_length` 且窗内 max 垂距 ≤ `paths_near_edge_match_distance_threshold`；滞后进出 |
| `trySelectBestReferencePathFromList` | `input_path` ↔ **reference 折线** | 同上，距阈为 `distance_tolerance`；再取最近邻段写入 `wall_line_points_` |
| `trySelectSegmentFromTwoPointPath` | `input_path` ↔ 两点边线（路沿等） | 同上，距阈为 `distance_tolerance`；去掉与边线的平行判定 |

## 距离判据

对路径上每个点 \(p_i\)，仅当其正交投影落在折线某段上（未裁剪 \(t\in[0,1]\)）时计入重叠窗：

- **重叠弧长**：最长连续重叠段的路径弧长 \(\ge\) `min_path_edge_overlap_length`（默认 0.5 m）
- **窗内垂距**：重叠点 \(\max_i d_i^\perp \le\) 对应距离阈值  
- 投影落在折线端点之外的点不参与 max，避免纵向未对齐把斜距抬过阈值  
- 日志同时输出 max / mean / overlap

不再计算 path 弦与 mission/reference 弦（或段）的夹角。

## 去掉的逻辑

- `shouldRunEdgeFollowingForTransformedPlan`：mission 首尾大弦的 `segmentToSegmentDistance` + `segmentDirectionAngleDifferenceDeg`，以及「距离近但夹角大立刻退出」
- `trySelectSegmentFromTwoPointPath` / reference 选用：path 与边线方向点积平行门槛  
- 误导性日志「找到贴边参考线，继续贴边模式」（改为按 hit/miss/active 真实状态打印）

## 保留

- `transform_path_line_length`：在 `runEdgeFollowingPathUpdate` 按**弧长**截断；不足则退出贴边（不再在 `shouldRunEdgeFollowingForTransformedPlan` 用弦长复查）
- `paths_near_edge_enter_hit_count` / `exit_miss_count` 滞后  
- 机器人相对路径航向差（`trySelectSegmentFromTwoPointPath` 内 ±45°，属车头与 path，不是边线平行度）  
- 最近邻段扩窗写 `wall_line_points_`  

## 参数

| 参数 | 用途 |
|------|------|
| `transform_path_line_length` | 局部 path 截取/贴边所需最小**弧长** [m] |
| `paths_near_edge_match_distance_threshold` | mission 走廊：重叠窗内 path 点到 mission 折线最大允许垂距 |
| `distance_tolerance` | reference/边线：重叠窗内 path 点到参考折线/线段最大允许垂距 |
| `min_path_edge_overlap_length` | 三处门控共用：路径与边线/折线最小重叠弧长 [m]，默认 0.5 |
| `paths_near_edge_match_angle_threshold_deg` | 门控中不再使用（可保留参数以免破坏配置加载） |
| `parallel_tolerance` | reference/两点边线选用中不再使用（墙线激光候选 `trySelectWallSegmentFromCandidates` 仍可能使用） |
