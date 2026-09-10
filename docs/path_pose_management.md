# TEB 路径点管理

覆盖弓字路径上，局部规划器并不直接吃整条 `global_plan_`。每拍按固定顺序裁点、锁窗、再交给 TEB。本文记录当前整套规则，以及到达裁剪与几何锁必须共用同一个急弯的原因。

## 每拍顺序

`computeVelocityCommands` 里与路径点有关的步骤：

```text
1. pruneGlobalPlan 粗切（车后明显已走）
2. pruneGlobalPlan 精切（车后 1 m 量级）
3. pruneArrivedLockedCorner          ← 沿路到达当前急弯，含角点 erase
4. 同一套函数再找当前急弯，供射线占用检查
5. transformGlobalPlan               ← 最近点（禁止跳过未裁急弯）+ 几何锁 / 占用锁
6. 占用锁必要时 erase 本行
7. 射线 254 且角点已在车后时，含角点 erase（独立通道，先保留）
8. 贴边再改 transformed_plan
9. via-point 从 transformed_plan 采样
```

`global_plan_` 是剩余全局路径；`transformed_plan` 是这一拍交给 TEB 的局部窗。几何锁只改窗，不删 `global_plan_`；到达裁剪和占用锁才删全局点。

## 急弯定义

折线顶点夹角（弦 `idx-1, idx, idx+1`）小于 `theta_threshold`（默认 135°）视为急弯。下标 0 无法计算夹角（返回 180°）。下标 1 可以算。

弓字换行是两个急弯夹一条短边：

```text
长行 → ①急弯 → 短边 → ②急弯 → 下一长行
```

一次只处理当前锁住的那一个顶点。裁掉 ① 后，下一拍再锁 ②。不要用平面距离把 ② 当成 ①。

## 车后裁剪（pruneGlobalPlan）

- 粗切：`rough_global_plan_prune_distance`（默认 3 m）
- 精切：`global_plan_prune_distance`（默认 1 m），搜索弧长 `global_plan_prune_max_accum_dist`
- 语义：丢掉机器人后方已走路点，**不保证**把换行角点裁掉（角点还在车侧/车前时会留下）

## 最近点（窗起点）

欧氏最近 + 0.5 m（平方距离 0.25）局部极小滞后。弓字换行时下一长行在平面上往往比眼前角点更近，纯欧氏会把窗起点跳到邻行。

因此在欧氏结果 \(i\) 上再扫 \([1, i)\)：若存在未裁急弯 \(k\)，则在 \([0, k]\) 上重选最近点。邻行再近，窗起点也不会越过还在路上的 ①。

实现：`findEuclideanClosestPlanIdx`、`clampClosestIdxBeforeUnprunedCorner`。到达裁剪和 `transformGlobalPlan` 共用。

## 当前锁点 \(C\)

```text
若最近点本身已是急弯：从该点前一个下标起搜（避免把 C 当成窗头后锁到 C2）
否则：从最近点起搜第一个急弯
C = findFirstSharpCornerIdx(...)
```

搜索含下标 1，短边终点贴在路径头时也能锁住。不再从下标 2 起搜，也不再与上一角点做 0.2 m 平面去重。

前瞻弧长：`max_global_plan_lookahead_dist + 1`（到达裁剪）；几何锁还受 `transform_global_plan_costmap_span_scale * costmap` 限制。

## 到达裁剪（pruneArrivedLockedCorner）

在 `transformGlobalPlan` **之前**执行，先裁再锁。

到达条件（必须同时）：

- \(C \ge 0\) 且 \(C\) 不是全局最后一个点（后面还有路）
- 沿路径弧长 \(i \rightarrow C \le\) `last_corner_record_distance`（默认 0.3 m）

到达后：

```text
global_plan_.erase(begin, begin + C + 1)  // 含角点，一次只裁 C
```

不用车到任意急弯的欧氏圆：换行时 ① 和 ② 常常同时落在 0.3 m 圆内，欧氏到达会连裁两刀、跳过短边。

不用残差 0.5 m：短边本身可能短于 0.5 m，会把该裁的 ① 卡住。

已删除：

- `last_corner_pose_` / `last_corner_pruned_`
- 搜索时 0.2 m 去重（`last_corner_distinct_distance` 仅兼容 yaml，代码不再读取语义）
- 4 m 内 5 cm 匹配后再 erase

日志：

- `[lock_corner] closest_raw=.. closest=.. C=.. angle=.. arc_to_c=.. arrived=0/1 reason=...`
- 未裁原因：`no_corner` / `no_residual` / `arc_to_c` / `arrive_dist_disabled`
- 裁成功：`prune_arrived_corner idx=C erase=[0,C] remain=N`

## 几何锁与占用锁（transformGlobalPlan）

| lock | 条件 | 窗尾 | 是否删 global_plan_ |
|------|------|------|---------------------|
| 1 几何锁 | 前瞻内有急弯，路上到弯裸足迹空闲 | `end = C` | 否 |
| 2 占用锁 | 占用贴窗尾 | 加长 / 安全点 / 删本行 | 本行不可达或到达行尾安全点时 erase |
| 0 无锁 | 前瞻内无急弯 | 前瞻弧长（默认数米） | 否 |

几何锁的意义：行尾达得到，局部窗仍停在第一个急弯，避免 TEB 把换行拉成斜线。它**不是**到达判定，不会裁点。裁点由上一节到达裁剪负责。

占用锁尾点用左右扩足迹检查；本行找不到合法尾点会 `erasing global_plan_[from,to]`。短边（3 个点的换行段）被当成「一行」整段删掉仍是已知风险，本次未改。

## 射线占用删角点

贴边/角点接近时，车到角点的直线上采样 footprint。命中 254/253、角点已在车后、且 `|vx|` 超过阈值时，含角点 erase。与到达裁剪独立，先保留。

## 局部窗之后

- 占用锁 erase 后的 `goal_idx` 会下移
- `runEdgeFollowingPathUpdate` 只改 `transformed_plan`
- via-point 按 `global_plan_viapoint_sep` 从局部窗采样

## 参数

| 参数 | 用途 |
|------|------|
| `theta_threshold` | 急弯夹角阈值 [deg] |
| `last_corner_record_distance` | 沿路到达弧长 [m]，到达裁剪 |
| `max_global_plan_lookahead_dist` | 局部窗前瞻、角点搜索弧长 |
| `transform_global_plan_costmap_span_scale` | 行尾/角点搜索相对代价图跨度 |
| `global_plan_prune_distance` | 车后精切 |
| `transformed_plan_row_terminal_arrive_distance` | 占用锁行尾到达后删行 |
| `last_corner_distinct_distance` | 已停用 |
| `prune_before_corner_distance` / `prune_corner_residual_distance` | 到达裁剪已停用 |

## 验收

1. 长行接近 ①：`lock=1`，`arc_to_c` 下降，到阈值后出现 `erase=[0,C]`，下一拍窗在短边上（角点是 ②）。
2. 短边再短，不会同一拍把 ② 裁掉。
3. 不应再出现「无到达裁剪日志、窗从 `end=7 lock=1` 下一拍变成 `corner=68 lock=0 end=27`」。
4. ② 可以出现在下标 1，日志仍能打出该角点。
