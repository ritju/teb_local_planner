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

## 车后裁剪的方向门控与全覆盖路径优化

### 结论

这段方向门控不建议直接删除。它的设计目的仍然成立：机器人为了绕障可能暂时靠近相邻长行，不能因为相邻行的平面距离更近，就把当前行和换行角点一起裁掉。

但是，当前实现既没有真正执行方向门控，也不能单独依靠朝向解决弓字形/回字形路径的误裁剪问题。因此应当保留“方向是裁剪条件之一”的原则，重写候选点选择和裁剪提交逻辑，而不是保留当前代码或简单删除。

### 当前代码为什么没有生效

`pruneGlobalPlan()` 中先执行：

```cpp
if (dist_sq < min_dist_threshold)
{
  min_dist_threshold = dist_sq;
  erase_end = it;
  if (dist_sq < dist_thresh_sq)
  {
    break;
  }
}
```

后面又判断：

```cpp
if (dist_sq < min_dist_threshold)
```

但此时 `min_dist_threshold` 已经被赋值为当前的 `dist_sq`，所以这个条件不会成立。机器人和路径点距离小于裁剪阈值时还会在方向检查之前直接 `break`。因此注释中描述的“朝向不一致时继续向前搜索”实际上没有发生。

### 只使用朝向也不够

朝向门控需要保留，但它只能作为辅助条件。弓字形或回字形的相邻长行通常近似平行，可能出现以下情况：

```text
当前行方向：  →
相邻行方向：  →
机器人绕障后靠近相邻行
```

这时当前行和相邻行的 yaw 都可能满足阈值，单纯比较机器人 yaw 与路径点 yaw 仍会误选相邻行。真正需要保护的是路径顺序和未完成换行角点，而不是单个路径点的朝向。

### 建议的裁剪判定

建议把裁剪分成“候选点搜索”和“裁剪提交”两个阶段。

#### 候选点搜索

1. 先按路径顺序找到机器人当前所在的候选区间，不能从整条剩余路径中自由选择最近点。
2. 使用点到折线段的投影距离，而不是只比较路径顶点距离。投影结果应同时返回路径段下标和沿路径弧长。
3. 候选点只能位于当前未完成角点之前。`clampClosestIdxBeforeUnprunedCorner()` 应继续作为硬限制。
4. 在当前点附近建立有限搜索窗：允许少量向后的点和有限距离向前的点，禁止一次跳过一个完整换行段。
5. 对每个候选点计算以下条件：

   | 条件 | 作用 |
   |---|---|
   | 横向距离 | 判断机器人是否真的靠近该行 |
   | 路径切线与机器人 yaw | 排除明显反向的点 |
   | 候选点与当前锚点的路径弧长 | 防止跨行跳跃 |
   | 候选点之前是否存在未完成急弯 | 防止跳过当前换行 |
   | 当前点到候选点之间是否连续 | 防止从当前行直接跳到相邻行 |

6. 方向差只作为门控条件之一。方向不一致时，只有候选点明显比当前候选更近并且没有跨过未完成角点，才允许继续搜索。
7. 当前候选与新候选的代价差必须超过一个滞后阈值，或者连续多个周期都满足，才能切换。这样可以避免机器人在两条相邻行之间来回切换。

#### 裁剪提交

只有候选点同时满足以下条件，才删除其之前的路径点：

```text
候选点在当前行顺序之后
候选点未跨过未完成角点
机器人已在路径弧长意义上经过候选点
候选点方向满足阈值，或已连续确认多个周期
```

粗裁剪和精裁剪不应各自重新寻找一次最近点。建议统一使用同一个候选结果：

```text
先确定当前路径锚点
→ 粗裁剪只删除确定在车后的点
→ 精裁剪在同一锚点附近细化
→ 最后再执行角点到达裁剪
```

粗裁剪不能跨过未处理急弯，也不能因为相邻行距离更近而改变当前路径锚点。如果暂时没有可靠的路径顺序状态，建议先关闭粗裁剪，保留精裁剪和角点到达裁剪，避免粗裁剪抢先删除一整段路径。

### 针对弓字形和回字形路径的推荐策略

建议显式维护一个“当前行/当前角点”状态，而不是每次只依靠欧氏最近点推断：

```text
FOLLOW_ROW
  当前行可通行，沿当前行前进

APPROACH_CORNER
  已进入当前角点前视范围，局部窗锁到该角点

WAIT_CORNER_BLOCKED
  角点被占用，保持在角点前最近安全位置，等待障碍消失

ADVANCE_NEXT_ROW
  机器人沿路径弧长到达角点后，才删除 [0, corner_idx]

GOAL_OCCUPIED_HOLD
  原始终点被占用且没有安全退让点，立即进入制动/零速保持
```

状态转换必须沿路径顺序发生。相邻行的平面距离不能触发 `ADVANCE_NEXT_ROW`，只有当前角点的沿路到达条件才能触发。

### 角点被占用时的行为

设计目标是尽可能接近角点并清扫更多区域，因此角点被占用时不应立即删除整行。建议采用以下顺序：

1. 在角点前搜索 footprint 空闲的最近路径位置。
2. 将该位置作为临时终点，局部窗锁定到该点。
3. 机器人到达该点后发布零速度，等待角点释放。
4. 只有确认当前行和角点在持续时间内都不可行，或者收到上层重新规划路径，才允许删除当前行。

这样可以避免“当前行尾点被占用 → 找不到离散安全点 → 直接 erase 当前行 → 少清扫一段区域”的情况。

### 终点被障碍物占用时的行为

终点处理应优先于普通局部窗裁剪，并且只保留一个最终停车目标：

```text
原始终点空闲       → 使用原始终点
找到沿路径退让点   → 使用退让点并锁定
没有退让点         → 依据制动距离立即进入 GOAL_OCCUPIED_HOLD
```

`last_occupied_goal_dist` 不应成为唯一触发条件。触发距离至少应为：

```text
制动距离 + occupied_goal_braking_margin + footprint 安全裕度
```

如果当前局部窗已经找到安全终点，`occupied_goal_no_standoff_` 和停车距离必须针对这个安全终点更新，不能继续引用原始占用终点的距离。

### 需要同步修正的现有冲突

1. `global_plan_` 删除应在最终终点和局部窗确定后统一提交，避免先生成 `transformed_plan`、再删除 `global_plan_` 导致两个路径版本不一致。
2. `goal_idx`、`path_key_points`、退让状态和 `occupied_goal_dist_m_` 必须随着最终终点一起更新。
3. 角点分支修改 `global_plan_` 后，必须重新生成或重新截取 `transformed_plan`，不能继续使用旧快照。
4. 角点无障碍时如果要停在角点前，应明确删除角点本身还是只删除角点之后的点，避免 `erase(check_it, end)` 带来的语义歧义。
5. `setPlan()` 重新收到相同全局路径时会清除退让锁，但 TEB 仍可能 warm start。建议按路径版本判断：路径实质未变化时保留退让状态，路径发生变化时再清除。
6. `transform_global_plan_goal_occupied_tolerance` 和 `transform_global_plan_goal_search_resolution` 当前没有实现。如果不再做终点周围网格搜索，应删除或明确标记为废弃参数。

### 推荐的最小改动顺序

```text
第一步：修正 pruneGlobalPlan 的候选点逻辑，使方向门控真正执行。
第二步：增加未完成角点和路径顺序限制，防止相邻行跳跃。
第三步：把粗裁剪、精裁剪统一到同一个路径锚点。
第四步：角点被占用时优先等待，不立即删除整行。
第五步：统一最终 terminal pose，再生成 transformed_plan。
第六步：将终点占用停车改为制动距离触发，而不是仅依赖固定距离。
```

### 验收场景

至少需要验证以下场景：

1. 机器人沿长行直行，最近点方向一致，正常裁剪车后点。
2. 机器人绕障后靠近相邻长行，但当前角点未到达，不能裁到相邻行。
3. 相邻行与当前行方向相同，单纯 yaw 相同也不能触发跳行。
4. 机器人沿短边接近第一个角点，局部窗锁到当前角点，不提前锁定第二个角点。
5. 当前角点被占用，机器人能到达角点前安全位置并保持等待，不删除整行。
6. 原始终点被占用且存在退让点，机器人停车在退让点。
7. 原始终点被占用且不存在退让点，机器人在制动距离内进入零速保持，不先让 TEB 规划到占用终点。
8. 上层重复下发相同路径时，退让锁和 TEB warm start 不发生冲突。

## 按职责重新梳理现有路径点逻辑

当前混乱的根源不是某一个参数，而是同一拍内有多个函数同时决定“当前路径走到哪里”和“局部路径终点在哪里”。建议先把现有功能分成四类，再按固定顺序执行。

### 四类对象

#### 1. 全局路径进度

只负责回答：机器人已经走过哪里，哪些历史点可以从 `global_plan_` 删除。

涉及函数：

```text
setPlan
pruneGlobalPlan
pruneArrivedLockedCorner
```

这一类可以修改 `global_plan_`，但不应该直接修改 `transformed_plan`。

#### 2. 当前行和局部窗口

只负责回答：从全局路径的哪个点开始，当前周期把哪一段交给 TEB。

涉及函数：

```text
findEuclideanClosestPlanIdx
clampClosestIdxBeforeUnprunedCorner
findFirstSharpCornerIdx
lastIdxWithinArc
transformGlobalPlan
```

这一类应该读取已经确定的 `global_plan_`，生成一个局部窗口结果，不应该在窗口生成过程中继续修改全局路径。

#### 3. 障碍物和停车决策

只负责回答：局部窗口的终点是正常目标、角点前目标、安全退让点，还是零速度保持。

涉及功能：

```text
selectGoalStandoffAlongPlan
candidate_is_safe
tryGoalStandoffHold
tryOccupiedGoalNearHold
tryPathWaitHold
```

这一类应该只产生一个最终终点和一个停车状态。当前实现中原始终点退让、occupancy lock、安全距离终点和无退让点停车分别保存状态，导致同一拍可能有多个“终点”。

#### 4. TEB 输入和优化

只负责消费最终确定的 `transformed_plan`：

```text
updateViaPointsContainer
configureBackupModes
runEdgeFollowingPathUpdate
planner_->plan
planner_->isTrajectoryFeasible
```

TEB 不应再承担全局路径裁剪、当前行切换或终点占用判定。

### 当前代码的实际执行顺序

当前一拍大致是：

```text
setPlan（异步发生）
  ↓
粗裁剪 global_plan_
  ↓
精裁剪 global_plan_
  ↓
到达角点裁剪 global_plan_
  ↓
寻找倒车段和角点
  ↓
transformGlobalPlan
  ├─ 再找最近点
  ├─ 再找角点
  ├─ 选择终点退让点
  ├─ 检查当前行 footprint
  ├─ 选择 occupancy lock 终点
  └─ 返回 transformed_plan 和 global_plan_ 删除区间
  ↓
根据返回区间再次删除 global_plan_
  ↓
角点碰撞逻辑
  ├─ 可能再次删除 global_plan_
  └─ 可能截短 transformed_plan
  ↓
贴边逻辑继续修改 transformed_plan
  ↓
via point 和 backup horizon 再处理 transformed_plan
  ↓
等待/退让/占用终点停车判断
  ↓
原地转向判断
  ↓
TEB 规划
```

这里同时存在三种路径版本：

```text
裁剪前的 global_plan_
transformGlobalPlan 使用的 global_plan 快照
角点和贴边修改后的 transformed_plan
```

`goal_idx`、退让状态、占用终点距离和可视化下标又分别来自不同阶段，因此后续函数看到的终点可能不一致。

### 按现有功能整理后的单一流程

在不立即删除现有功能的前提下，建议把每拍流程整理成下面九步。

#### 第一步：接收路径并建立路径版本

`setPlan()` 只做以下事情：

```text
清洗无效点
删除重复点
保存路径版本号
重置与旧路径绑定的状态
```

路径内容没有实质变化时，不应反复清除终点退让锁和 TEB warm start。

#### 第二步：确定路径进度锚点

只执行一次最近点搜索，输出：

```text
anchor_idx
anchor_arc
active_corner_idx
active_row_end_idx
```

这个结果在本周期内固定使用。粗裁剪、精裁剪、角点检测和局部窗口都使用同一个锚点，不能每个函数重新从整条路径搜索最近点。

#### 第三步：决定是否可以删除历史点

只允许以下两类删除：

```text
机器人已经沿路径通过的历史点
机器人已经沿路径到达的当前角点及其之前点
```

“相邻行平面距离更近”不能触发删除。角点没有沿路径到达时，不能因为避障偏移就删除角点之前的路径。

#### 第四步：确定当前行窗口

使用已经确定的 `anchor_idx` 和 `active_corner_idx`，得到：

```text
window_start_idx
window_end_idx
lookahead_idx
```

此时只截取路径，不修改 `global_plan_`。局部窗口的最大长度由以下顺序共同限制：

```text
当前角点
代价地图跨度
max_global_plan_lookahead_dist
```

#### 第五步：统一做占用检查

对当前窗口中的路径点做一次 footprint 检查，并记录：

```text
first_occupied_idx
last_occupied_idx
last_free_idx
```

原始终点占用、当前行尾占用和角点占用都使用这一份结果，避免 `selectGoalStandoffAlongPlan()` 和 `candidate_is_safe()` 各自重新判断。

#### 第六步：统一选择一个 terminal decision

建议用一个结果表示终点：

```text
terminal_type:
  NORMAL_GOAL
  CORNER_APPROACH
  SAFE_BEFORE_OBSTACLE
  GOAL_STANDOFF
  WAIT_FOR_CLEARANCE

terminal_idx
terminal_pose
hold_required
hold_reason
```

选择优先级固定为：

```text
路径等待条件
→ 终点被占用停车
→ 角点被占用等待
→ 当前行障碍安全终点
→ 可达角点
→ 普通 lookahead 终点
```

一旦得到 `terminal_decision`，所有后续逻辑都使用它。`occupied_goal_dist_m_`、`goal_standoff_force_hold_`、`current_goal_idx` 和 marker 都从这个结果生成。

#### 第七步：提交 global_plan_ 删除

只有在终点决策完成后，才提交本周期的删除区间。提交后需要重新确认：

```text
global_plan_ 仍然非空
terminal_idx 仍然有效
terminal_pose 仍然属于当前路径版本
```

如果删除改变了终点索引，应重新生成局部窗口，不能继续使用删除前的索引和快照。

#### 第八步：生成唯一的 transformed_plan

此时才把最终路径窗口转换到控制器坐标系，并执行：

```text
角点前截断
贴边路径调整
via point 提取
backup horizon 截断
```

这些操作只修改 `transformed_plan`，不再修改 `global_plan_`。

#### 第九步：停车或交给 TEB

执行顺序固定为：

```text
transformed_plan 为空检查
→ terminal_decision 要求保持则发布停车命令
→ 原地转向
→ TEB 规划和可行性检查
```

这样 TEB 只处理“从机器人当前位置到最终局部终点”的优化问题，不再参与路径点生命周期管理。

### 现有函数对应的整理结果

| 现有功能 | 当前职责 | 整理后的职责 |
|---|---|---|
| `pruneGlobalPlan` | 搜索最近点并删除历史点 | 只按已确认的路径锚点删除历史点 |
| `pruneArrivedLockedCorner` | 再次搜索角点并删除角点 | 使用本周期唯一的 `active_corner_idx` |
| `transformGlobalPlan` | 变换、找最近点、找角点、选终点、请求删除 | 只生成局部窗口和占用信息 |
| `selectGoalStandoffAlongPlan` | 单独维护终点退让锁 | 参与统一的 `terminal_decision` |
| `candidate_is_safe` | 在当前行内二次选择安全点 | 使用统一占用结果选择安全终点 |
| 角点碰撞分支 | 可能修改 global_plan_ | 只报告角点状态，由路径进度管理器决定是否删除 |
| `runEdgeFollowingPathUpdate` | 修改局部窗口 | 保持只修改 transformed_plan |
| `tryGoalStandoffHold` | 依据独立锁定状态停车 | 读取统一 terminal decision |
| `tryOccupiedGoalNearHold` | 依据原始终点距离停车 | 读取统一 terminal decision 和制动距离 |
| `configureBackupModes` | 继续截断局部路径 | 只能在 terminal decision 之后缩短窗口 |

### 最需要先解决的三个冲突

#### 冲突一：同一拍重复寻找最近点

`pruneGlobalPlan()`、角点检测、`transformGlobalPlan()` 和 `pruneArrivedLockedCorner()` 都可能重新寻找最近点。它们使用的搜索范围和限制不同，容易在同一拍得到不同的当前点。

应改为一次搜索，生成 `PathProgress`，本周期内只读。

#### 冲突二：global_plan_ 和 transformed_plan 交叉修改

当前先生成 `transformed_plan`，后删除 `global_plan_`，然后又修改 `transformed_plan`。这会让索引和终点失去对应关系。

应改为：

```text
先确定路径进度和删除区间
→ 提交 global_plan_ 修改
→ 再生成 transformed_plan
→ 后续只修改 transformed_plan
```

#### 冲突三：多个停车终点并存

原始终点退让点、当前行安全点、角点前点和无退让点停车使用不同状态。机器人实际要去的点和停车判断使用的点可能不同。

应改为只有一个 `terminal_decision`，其他状态都由它派生。

### 简化后的判断原则

后续修改代码时，每个路径点操作只回答一个问题：

```text
删除 global_plan_：机器人是否沿路径已经通过？
截断 transformed_plan：局部窗口是否应该在这里结束？
进入停车保持：机器人是否已经不能安全继续接近下一个目标？
交给 TEB：当前终点是否已经明确且 footprint 可行？
```

如果一个函数同时回答两个以上问题，就应拆分为“计算结果”和“提交修改”两个步骤。
