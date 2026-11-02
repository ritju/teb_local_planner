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

## 四类路径处理功能的设计目标

下面的目标以当前程序中的 `global_plan_`、`transformed_plan`、角点锁、占用锁、终点退让和 TEB 优化功能为基础，明确每一类功能应该承担的责任。

### 1. 全局路径进度管理

全局路径进度管理的对象是 `global_plan_`。它只负责记录任务路径中尚未完成的部分，并删除已经确认完成的历史路径点。

#### 1.1 剔除已经走过的路径点

目标：

```text
删除机器人已经沿路径通过的点，减少后续搜索范围和 TEB 输入长度。
```

必须满足以下条件：

- 删除依据使用路径顺序和沿路径弧长，不使用单独的欧氏最近距离决定跨行跳转。
- 不能因为机器人避障后靠近相邻行，就删除当前行之后的路径点。
- 不能跨过尚未处理的换行角点。
- 粗裁剪、精裁剪和角点到达裁剪必须共享同一个路径进度锚点。
- 删除后必须保留当前有效行、当前未完成角点和后续任务路径。

弓字形和回字形路径中，相邻行可能在空间上很近，甚至方向相同。因此最近点只能作为候选，最终是否裁剪必须同时考虑：

```text
路径索引顺序
当前行索引
未完成角点
机器人到路径的横向距离
机器人与路径切线的方向关系
```

#### 1.2 角点被障碍物占用时保留路径状态

目标：

```text
角点被占用时，保留当前行和角点信息，使机器人可以到达角点前的最近安全位置并等待。
```

处理要求：

- 角点 footprint 碰撞不能直接等同于“当前行已经完成”。
- 角点未被机器人沿路径到达前，不得执行 `pruneArrivedLockedCorner()`。
- 不能因为局部窗口尾部没有可用点，就立即删除整行。
- 应优先寻找角点前的安全路径点作为临时终点。
- 机器人到达临时终点后，应进入零速度保持，等待障碍物清除。
- 只有在持续确认当前行不可行，或上层路径已经改变时，才允许放弃当前行。

这样可以避免以下错误链路：

```text
角点被占用
→ 局部窗口尾部无可用点
→ 删除当前行
→ transformed_plan 失去有效终点
→ TEB 规划失败
→ 上层重新规划
```

#### 1.3 180° 回头路的角点裁剪

目标：

```text
对于 180° 回头路，在机器人真正通过回头角点后及时裁剪角点，防止局部路径长期锁在旧行和回头段上。
```

处理要求：

- 180° 回头点必须作为路径顺序中的当前角点处理，不能只依赖欧氏距离判断是否到达。
- 到达条件应使用机器人到角点的沿路径弧长，并允许角点前后存在很短的回头段。
- 角点到达后，一次只删除当前角点及其之前的历史点，不能继续删除下一个回头角点。
- 如果角点被障碍物占用，应进入“角点前等待”状态；障碍物释放后再完成角点裁剪。
- 裁剪完成后，下一周期重新确定新的当前行和新的前视角点。

回头路的核心约束是：

```text
角点到达触发路径推进，平面距离接近不能触发路径推进。
```

#### 1.4 清扫机器人尽可能跟踪原始路径

目标：

```text
在满足 footprint 安全和 TEB 优化约束的前提下，尽可能沿上层下发的原始路径运行。
```

表现为：

- 优先使用原始路径点和原始路径方向生成局部窗口。
- 避障只改变当前局部轨迹，不随意删除未完成的全局路径。
- 经过临时避障后，应回到原始路径，而不是将避障后的偏移位置当成新的永久路径进度。
- 只有机器人沿路径确认通过后，才删除全局路径点。
- 通过角点后继续使用后续原始路径，不用局部 TEB 轨迹反向推断全局进度。

### 2. 当前行和局部窗口

当前行和局部窗口的对象是 `transformed_plan`。它负责从已经确定的 `global_plan_` 中截取当前周期可交给 TEB 的路径段。

#### 2.1 截取局部路径

目标：

```text
从当前路径进度开始，截取一段连续、可解释、能够被 TEB 使用的局部路径。
```

窗口边界应按以下优先级确定：

```text
当前未完成角点
→ 局部代价地图可覆盖范围
→ max_global_plan_lookahead_dist
→ 当前行尾
```

窗口必须满足：

- 起点来自唯一的路径进度锚点。
- 路径点索引连续，不能从当前行跳到相邻行。
- 窗口终点具有明确原因，例如 `NORMAL_GOAL`、`CORNER_APPROACH` 或 `SAFE_BEFORE_OBSTACLE`。
- 生成窗口后，后续贴边、via point 和 TEB 逻辑只能修改 `transformed_plan`，不能再反向修改 `global_plan_`。

#### 2.2 避免固定窗口导致障碍点成为终点

目标：

```text
局部窗口不能因为固定长度截取，把被障碍物占用的路径点直接交给 TEB 作为终点。
```

处理顺序应为：

1. 先按固定范围生成候选窗口。
2. 对窗口内路径点进行 footprint 碰撞检查。
3. 如果窗口末端空闲，使用末端作为候选终点。
4. 如果窗口末端或其附近被占用，向路径起点方向搜索最近安全点。
5. 如果存在安全点，将其作为临时终点并记录终点原因。
6. 如果不存在安全点，进入停车等待或角点等待，不把占用点交给 TEB。

安全点搜索应优先检查路径连续性和 footprint 安全，随后再检查前方、后方安全距离。安全点可以是路径点，也可以是路径段上的插值点，避免路径采样稀疏导致误判。

#### 2.3 当前行障碍的处理边界

当前行检测到障碍物时，应区分三种情况：

```text
障碍物在窗口中间：缩短窗口，但保留后续全局路径。
障碍物位于角点：锁定角点前安全终点并等待。
障碍物位于最终任务终点：进入终点占用停车策略。
```

不能把三种情况都归类为“当前行不可达并删除当前行”。

### 3. 障碍物和停车决策

障碍物和停车决策负责确定机器人是否还能安全接近局部终点，以及是否必须停止等待。

#### 3.1 沿路径距离达到阈值时立即停车

目标：

```text
如果沿路径搜索到的可用安全距离已经小于设定阈值，机器人立即停车等待，不再继续向占用区域前进。
```

“沿路径距离”应优先于单纯欧氏距离，至少包含：

```text
机器人当前点到障碍物的路径弧长
机器人当前速度对应的制动距离
footprint 安全裕度
occupied_goal_braking_margin
```

停车触发条件应类似于：

```text
剩余安全弧长
≤ 制动距离 + 制动缓冲 + footprint 安全裕度
```

达到条件后：

- 不再调用会继续逼近障碍物的 TEB 目标规划。
- 发布逐步减速直到零速的控制命令。
- 持续等待障碍物清除或路径更新。
- 停车等待期间不删除未完成的全局路径。
- 等待超时后再把控制失败交给上层恢复机制。

#### 3.2 终点被占用

终点占用时，停车决策按以下顺序执行：

```text
原始终点空闲
  → 正常规划

原始终点占用且存在沿路径安全退让点
  → 使用退让点作为临时终点
  → 到达后零速保持

原始终点占用且不存在安全退让点
  → 根据制动距离提前停车
  → 保留原始全局路径并等待
```

停车决策必须和最终 `terminal_pose` 绑定，不能一边把局部终点设为安全点，一边仍然使用原始占用终点计算停车距离。

#### 3.3 角点占用和一般障碍占用的区别

角点占用的处理目标是“尽量接近角点并等待”；一般路径中间障碍的处理目标是“在安全点停车或寻找绕行轨迹”。两者不能共用同一个立即删行分支。

建议终点类型至少区分：

```text
NORMAL_GOAL
CORNER_APPROACH
SAFE_BEFORE_OBSTACLE
GOAL_STANDOFF
WAIT_FOR_CLEARANCE
```

只有 `WAIT_FOR_CLEARANCE` 和安全停车状态可以阻止 TEB 继续向前规划；普通障碍中间点仍可以交给 TEB 做局部绕行，但不能改变全局路径进度。

### 4. TEB 输入和优化

TEB 的职责是：

```text
在最终确定的局部路径、终点姿态、障碍物集合和运动学约束下，计算当前控制周期的可行轨迹和速度。
```

TEB 输入必须已经确定：

- 起点是当前机器人位姿；
- 局部路径点连续；
- 局部终点未被 footprint 占用；
- 终点类型和停车状态已经明确；
- via point 来源于最终 `transformed_plan`；
- 角点、贴边和倒车模式已经完成配置。

TEB 不负责：

- 删除 `global_plan_` 历史点；
- 判断机器人是否已经通过全局路径角点；
- 决定是否跳转到相邻清扫行；
- 把占用终点改成安全终点；
- 在已经要求停车等待时继续向前优化。

TEB 优化失败时，应区分失败原因：

```text
局部窗口终点被占用
  → 返回停车等待结果

局部窗口无有效安全终点
  → 保留全局路径，等待或请求恢复

局部轨迹在可行终点之间优化失败
  → 执行优弧、倒车或其他 TEB 恢复策略

路径本身已经完成或被上层更新
  → 重新建立路径进度
```

只有最后一种“可行终点已经确定但 TEB 优化失败”才适合直接进入 TEB 的 infeasible recovery。终点占用和局部窗口截取失败不应直接伪装成 TEB 优化失败。

### 四类功能之间的固定接口

建议每个控制周期形成三个明确结果：

```text
PathProgress
  anchor_idx
  active_corner_idx
  row_start_idx
  row_end_idx
  erase_from_idx
  erase_to_idx

LocalWindow
  transformed_plan
  first_occupied_idx
  last_free_idx
  window_end_idx

TerminalDecision
  terminal_type
  terminal_idx
  terminal_pose
  hold_required
  hold_reason
```

调用关系固定为：

```text
PathProgress
  → LocalWindow
  → TerminalDecision
  → transformed_plan
  → TEB
```

`global_plan_` 只由 `PathProgress` 的删除结果修改；`transformed_plan` 只由 `LocalWindow` 和局部模式修改；停车函数只读取 `TerminalDecision`。

### 最终验收标准

满足以下行为才算达到设计目标：

1. 机器人避障靠近相邻行时，不会因为欧氏最近点而跨行裁剪。
2. 当前角点被占用时，机器人可以到达角点前安全位置并停车等待。
3. 180° 回头路机器人通过当前角点后，下一周期可以进入新的路径段，不会长期锁在旧角点附近。
4. 机器人始终优先沿原始全局路径运行，避障结束后能够回到原路径。
5. 固定局部窗口末端被占用时，会先搜索安全终点，不会把占用点直接交给 TEB。
6. 沿路径剩余距离小于制动安全阈值时，立即进入减速和零速保持。
7. 停车等待期间不删除未完成全局路径，也不触发无意义的全局重规划。
8. TEB 只接收已经确定终点和安全状态的局部路径，规划失败日志能够区分“终点占用”“无安全局部窗口”和“优化失败”。

## 按现有逻辑补充的四类设计目标

本节只描述当前程序已有功能应达到的行为，不改变现有函数分工和调用顺序。

### 1. 全局路径进度管理

全局路径进度管理由 `setPlan()`、`pruneGlobalPlan()`、`pruneArrivedLockedCorner()`、角点检测和 `transformGlobalPlan()` 共同完成，目标是维护 `global_plan_` 中仍然有效的任务路径。

#### 1.1 删除已经走过的点

粗裁剪和精裁剪用于删除机器人后方已经经过的路径点，但删除必须沿全局路径顺序进行。弓字形和回字形路径中，相邻行虽然空间距离很近，仍然属于未完成路径，因此不能仅根据欧氏最近点把路径直接裁到相邻行。

当前逻辑应保证：

- `rough_global_plan_prune_distance` 只删除明确位于机器人后方的历史点；
- `global_plan_prune_distance` 负责进一步清理已经经过的点；
- `clampClosestIdxBeforeUnprunedCorner()` 防止当前点跨过尚未处理的角点；
- `pruneArrivedLockedCorner()` 只有在机器人沿路径到达当前角点后，才删除角点及其之前的历史点；
- 当前行、换行段和后续清扫行不能因为相邻行更近而被提前删除。

#### 1.2 角点被障碍物占用时及时处理

角点占用由角点 footprint 检查、射线检查和 `transformGlobalPlan()` 中的占用检查共同识别。目标是避免被占用的角点继续作为局部路径终点，导致 TEB 在角点附近反复尝试同一条不可行路径。

现有逻辑下应达到以下行为：

- 角点 footprint 被判定为占用后，不能继续把该角点作为可达的普通局部终点；
- 根据当前角点碰撞分支和占用锁逻辑，及时截断局部路径或提交当前行裁剪；
- 现有程序没有独立的“角点前安全点搜索”函数。只有在 `occ_near_tail` 成立、尾部空闲距离不足时，`candidate_is_safe()` 才会从 `row_end_idx` 向 `segment_start_idx` 搜索安全离散点，并把找到的 `safe_idx` 设置为 `end_idx`；该点可能位于角点前，也可能只是当前行中的其他安全点；
- 如果当前行没有可行终点，现有代码主要进入 `need_prune_remaining_row` 和 `end_idx` 回退分支；只有满足动态路径等待、终点退让或无退让终点近距条件时，才会真正进入等待保持，不能把这两者混写成“当前行无可行终点就会等待”；
- 角点处理完成后，后续窗口应继续使用角点之后的有效路径，不能一直锁在旧角点。

这里的“及时裁剪”指及时从当前可规划局部路径中移除不可用角点，是否删除 `global_plan_` 中更长的当前行，仍由现有占用锁和到达裁剪条件决定。

#### 1.3 180° 回头路及时完成角点裁剪

180° 回头路依赖角点识别、几何锁、到达角点裁剪和倒车段判断共同推进。目标是避免机器人已经完成回头动作，但旧角点仍然保留在局部路径中，使 TEB 继续尝试旧方向而产生震荡。

现有逻辑下应保证：

- 180° 回头点被识别为当前急弯或当前行终点；
- 几何锁只将局部窗口锁到当前角点，不永久锁住全局路径；
- 当机器人满足 `last_corner_record_distance` 的沿路径到达条件时，`pruneArrivedLockedCorner()` 删除该角点及其之前的历史点；
- 删除后下一周期重新确定新的最近点、当前行和下一个角点；
- 倒车段判定只影响 TEB 运动学和倒车偏好，不代替全局路径角点裁剪。

#### 1.4 尽可能跟踪清扫原路径

清扫机器人应优先执行上层下发的原始路径。避障时可以让 TEB 在局部范围内偏离路径，但全局路径进度不能由临时避障轨迹直接推进。

现有逻辑下应保证：

- `global_plan_` 只有在机器人确认经过路径点或完成角点后才裁剪；
- 局部避障不会永久改变未完成的清扫路径；
- 障碍物清除后，局部窗口可以重新回到原始路径；
- 不因为一次局部规划失败就直接丢弃尚未完成的大段清扫路径；
- via point 和 TEB 初始化仍然优先来自当前有效的原始路径窗口。

### 2. 当前行和局部窗口

当前行和局部窗口由 `transformGlobalPlan()` 完成，目标是从 `global_plan_` 中截取当前周期可用的连续路径，并避免固定窗口末端落在障碍物上。

#### 2.1 当前行范围

当前行范围由最近点、当前角点、代价地图跨度和 `max_global_plan_lookahead_dist` 共同限制。固定前视距离只能提供候选范围，不能直接决定最终终点。

现有逻辑下应保证：

- 局部窗口从当前路径进度开始；
- 有未处理角点时，窗口优先在当前角点处结束；
- 没有角点时，窗口受前视距离和代价地图跨度限制；
- 窗口路径点保持全局路径顺序连续；
- 不跨行截取，也不跳过当前未完成角点。

#### 2.2 避免固定窗口末端被占用

`globalPlanPoseLateralPaddedFootprintFreeInControllerFrame()` 用于检查路径点 footprint 是否可通行，`candidate_is_safe()` 用于检查占用点前后的安全距离，`transformed_plan_collision_pose_to_end_distance`、`safe_distance_front` 和 `safe_distance_back` 用于决定局部窗口的安全终点。

现有逻辑下应保证：

- 固定窗口末端先经过 footprint 检查；
- 末端被占用时，优先在当前行向前搜索可行安全点；
- 安全点必须同时满足自身空闲以及前后安全距离约束；
- 找到安全点时，`transformed_plan` 终点应使用安全点；
- 找不到安全点时，不能把被占用路径点直接作为 TEB 终点；
- 当前行裁剪分支只能在已有条件满足时触发，避免因一次窗口不足就过度删除整行。

#### 2.3 局部窗口与全局路径的关系

`transformed_plan` 是当前周期的局部路径快照，`global_plan_` 是后续任务路径。局部窗口截短、贴边调整和 TEB 优化不应被解释为全局路径已经完成。

现有逻辑下应保证：

- 局部窗口因障碍物缩短时，后续全局路径仍然保留；
- 只有已有行裁剪或角点到达条件满足时，才删除 `global_plan_`；
- `goal_idx` 与最终窗口终点保持一致；
- 角点、占用锁和贴边逻辑不会让局部窗口使用已经从全局路径删除的失效点。

### 3. 障碍物和停车决策

障碍物和停车决策由 footprint 检查、占用锁、终点退让、`tryGoalStandoffHold()`、`tryOccupiedGoalNearHold()` 和 `tryPathWaitHold()` 共同完成，目标是让机器人在剩余安全距离不足时停止接近障碍物。

#### 3.1 沿路径距离进入阈值后立即停车

当沿路径搜索得到的剩余安全距离已经小于设定阈值时，机器人应立即进入零速保持，不应继续让 TEB 以占用点为目标向前推进。

现有参数和逻辑中的安全距离包括：

```text
transformed_plan_collision_pose_to_end_distance
transformed_plan_collision_pose_safe_distance_front
transformed_plan_collision_pose_safe_distance_back
occupied_goal_braking_margin
last_occupied_goal_dist
```

现有逻辑下应保证：

- 先计算沿路径的剩余距离，再判断是否允许继续规划；
- 进入制动距离和停车缓冲范围后，`tryOccupiedGoalNearHold()` 或 `tryGoalStandoffHold()` 优先返回停车命令；
- 终点停车命令直接将线速度和角速度置零，不经过控制器内部的减速度曲线；动态 PATH 等待仍可使用原有减速逻辑；
- 等待期间不继续向占用终点发送前进目标；
- 等待期间不因为局部 TEB 失败而立即丢弃未完成的全局路径；
- 障碍物清除后，已有路径能够恢复局部规划；
- 超过已有保持超时时间后，再交由上层恢复机制处理。

#### 3.2 终点占用和一般路径占用

终点占用、角点占用和普通路径中间占用使用不同的现有分支：

```text
普通路径中间占用：优先生成局部安全终点或等待
角点占用：避免把角点继续作为局部目标
原始终点占用：使用沿路径退让点或近距零速保持
动态障碍等待：由 path wait 逻辑决定是否暂时停止
```

这些分支的共同目标是：在障碍物占用区域前停止，保留后续全局路径，并防止 TEB 在不可达终点附近重复优化。

### 4. TEB 输入和优化

TEB 使用最终生成的 `transformed_plan`、当前机器人位姿、当前速度、障碍物容器、via points 和配置中的运动学/优化参数执行轨迹优化。

现有逻辑下，TEB 的目标是：

- 使用局部窗口内的连续路径作为初始轨迹；
- 使用当前机器人位姿覆盖局部路径起点；
- 使用局部窗口最终安全终点作为优化目标；
- 按配置执行速度、加速度、转向、倒车、障碍物距离和时间代价优化；
- 对优化结果执行可行性检查；
- 在允许的情况下使用优弧、倒车或窄道参数进行已有的重试；
- 生成当前周期的速度命令。

TEB 不应把被占用终点当作正常优化目标。终点占用、角点等待和局部窗口无安全终点时，应先由已有停车或占用锁逻辑处理；只有局部终点已经明确且可用于规划时，才进入 TEB 优化。

### 四类目标之间的现有逻辑关系

当前功能的目标关系可以概括为：

```text
全局路径进度管理
  决定哪些路径点已经完成，避免跨行裁剪

当前行和局部窗口
  决定本周期截取哪些连续路径点，并过滤占用终点

障碍物和停车决策
  决定是否可以继续接近局部终点，必要时停车等待

TEB 输入和优化
  在局部终点安全且明确后，按照现有配置完成轨迹优化
```

这四类功能的验收重点是：全局路径不误删，局部终点不落在障碍物上，安全距离不足时及时停车，TEB 只优化已经通过路径和障碍物处理的局部路径。

### 当前程序中“等待”实际会在哪里触发

当前程序并不存在一个“当前行没有可行终点就自动等待”的通用分支。实际触发范围如下：

| 等待函数 | 实际触发条件 |
|---|---|
| `tryPathWaitHold()` | `path_wait_enable` 开启，并且独立的动态路径等待模块判定存在需要等待的障碍物 |
| `tryGoalStandoffHold()` | 原始终点被占用，已经找到并锁定沿路径退让点，机器人接近该退让点或被 `goal_standoff_force_hold_` 强制保持 |
| `tryOccupiedGoalNearHold()` | 原始终点被占用、没有退让点，并且机器人到原始终点的距离进入 `last_occupied_goal_dist` 或制动距离阈值 |

当前行中间点或角点被 footprint 判定为占用时，`transformGlobalPlan()` 的实际处理是：

```text
找到 safe_idx
  → end_idx = safe_idx

找不到 safe_idx
  → need_prune_remaining_row = true
  → end_idx 回退到当前行起点或最后一个空闲点
  → 请求删除当前行范围
```

这条路径不会直接设置等待状态。之后仍可能把回退后的 `transformed_plan` 交给 TEB；如果 TEB 不可行，才会抛出 `trajectory is not feasible`。因此“当前行无可行终点时进入已有等待”不是现有事实，而是尚未实现的行为目标。文档中此前的相关表述已改为上述实际逻辑。

### 终点占用处理的最新行为

终点占用时已取消沿全局路径搜索退让点的逻辑。`selectGoalStandoffAlongPlan()` 现在只检查原始终点 footprint：

```text
终点空闲
  → 正常路径处理

终点占用
  → occupied_goal_no_standoff_ = true
  → 不搜索 standoff_idx
  → 不提交当前行裁剪
  → tryOccupiedGoalNearHold() 立即输出零速度停车等待
```

因此 `last_occupied_goal_dist` 不再决定终点占用停车是否触发；只要本周期检测到原始终点占用，就进入停车保持。终点恢复空闲后，保持状态经过 0.4 秒释放延迟后解除。
