可以，而且建议优先增加“转向边”。但不要一次性把转角、转角速度、转角加速度全部加入，否则很难判断是哪一条边导致了优化不稳定。

建议按以下顺序实施：

1. 先验证转角/曲率边；
2. 再加入转角速度边；
3. 最后加入转角加速度边；
4. 每增加一类边，都做单元测试和轨迹级验证。

---

## 一、为什么可以优先增加转向边

当前 TEB 顶点仍然是：

\[
[x_i,y_i,\theta_i]
\]

但是每一段轨迹可以根据相邻位姿计算：

\[
v_i,\quad \omega_i,\quad \kappa_i,\quad \delta_i
\]

因此不需要先扩展顶点类型，就可以增加转向相关的 g2o 边。

当前程序已经有：

- `EdgeKinematicsDiffDrive`
- `EdgeKinematicsCarlike`
- `EdgeAcceleration`

其中 carlike 边位于 `edge_kinematics.h:163`，已经具备最小转弯半径约束。

因此第一阶段不建议直接修改顶点结构，而是复用现有位姿和时间节点，增加：

```text
EdgeSteeringAngle
EdgeSteeringRate
EdgeSteeringAcceleration
```

---

# 二、统一段变量计算

所有转向边必须使用同一套段变量计算函数，否则不同边之间会出现模型不一致。

建议抽出一个公共函数，例如：

```cpp
struct SegmentKinematics
{
  double dt;
  double longitudinal_displacement;
  double lateral_displacement;
  double velocity;
  double omega;
  double curvature;
  double steering_angle;
};
```

输入：

```text
Pose_i
Pose_i+1
TimeDiff_i
```

输出：

```text
v_i
omega_i
kappa_i
delta_i
```

## 1. 位移计算

令：

\[
\Delta x=x_{i+1}-x_i
\]

\[
\Delta y=y_{i+1}-y_i
\]

\[
\Delta\theta=
\operatorname{normalize}(\theta_{i+1}-\theta_i)
\]

建议使用平均航向：

\[
\bar{\theta}_i=
\theta_i+\frac{\Delta\theta_i}{2}
\]

得到：

\[
s_i=
\cos(\bar{\theta}_i)\Delta x+
\sin(\bar{\theta}_i)\Delta y
\]

\[
e_i=
-\sin(\bar{\theta}_i)\Delta x+
\cos(\bar{\theta}_i)\Delta y
\]

其中：

- \(s_i\)：沿车辆纵向的有符号位移；
- \(e_i\)：横向位移；
- \(s_i<0\)：倒车。

## 2. 速度和角速度

\[
v_i=\frac{s_i}{\Delta t_i}
\]

\[
\omega_i=\frac{\Delta\theta_i}{\Delta t_i}
\]

这里的 \(v_i\) 就是后轴中心线速度，\(\omega_i\) 就是后轴中心角速度。

## 3. 曲率

当 \(|v_i|>v_\epsilon\) 时：

\[
\kappa_i=\frac{\omega_i}{v_i}
\]

更稳定的几何形式是：

\[
\kappa_i\approx\frac{\Delta\theta_i}{s_i}
\]

前轮转角：

\[
\delta_i=\arctan(L\kappa_i)
\]

其中：

\[
L=0.854\text{ m}
\]

当：

\[
|v_i|\leq v_\epsilon
\]

时不能直接计算 \(\omega_i/v_i\)。此时应：

- 如果 \(|\omega_i|\) 也很小，则认为该段为静止段；
- 如果 \(|\omega_i|\) 较大，则直接判为原地旋转，不可行；
- 不应该用一个很小的速度去除角速度。

这一点非常重要，否则优化器容易产生数值异常。

---

# 三、第一条边：转角/曲率边

## 1. 是否需要新增转角边

从数学上说，当前 `EdgeKinematicsCarlike` 的最小转弯半径约束已经等价于最大曲率约束：

\[
R_{\min}=\frac{L}{\tan\delta_{\max}}
\]

你的参数：

\[
\delta_{\max}=60^\circ
\]

所以：

\[
R_{\min}
=
\frac{0.854}{\tan60^\circ}
\approx0.493\text{ m}
\]

对应：

\[
\kappa_{\max}
=\frac{1}{R_{\min}}
\approx2.028\text{ m}^{-1}
\]

因此：

```yaml
robot:
  wheelbase: 0.854
  min_turning_radius: 0.493
```

理论上已经可以限制最大前轮转角。

当前 carlike 边在 `optimal_planner.cpp:1183` 中添加。

## 2. 为什么仍然建议显式增加转角边

显式转角边的价值在于：

- 可以直接输出和调试 \(\delta\)；
- 可以与转角速度、转角加速度边共用同一变量；
- 可以处理速度相关角速度限制；
- 可以明确区分“曲率限制”和“转角执行器限制”。

但是建议第一版不要同时保留两个完全重复的最大曲率约束，否则会产生重复加权。

推荐选择：

### 方案 A：保留现有最小转弯半径边

暂时不增加独立 `EdgeSteeringAngle`，只增加：

- `EdgeSteeringRate`
- `EdgeSteeringAcceleration`

优点是改动更小。

### 方案 B：新增统一 `EdgeSteeringLimits`

将现有最小转弯半径约束替换成：

\[
|\delta_i|\leq\delta_{\max}
\]

并同时增加：

\[
|L\omega_i|\leq |v_i|\tan\delta_{\max}
\]

这样逻辑更清晰，但需要确认不要和旧的 `EdgeKinematicsCarlike` 重复计权。

第一阶段建议使用方案 A，先验证转向动态边。

---

# 四、第二条边：转角速度边

转角速度限制为：

\[
|\dot{\delta}|\leq0.5\text{ rad/s}
\]

对于连续两个轨迹段：

\[
\dot{\delta}_i=
\frac{\delta_{i+1}-\delta_i}
{(\Delta t_i+\Delta t_{i+1})/2}
\]

约束：

\[
|\dot{\delta}_i|\leq0.5
\]

## 1. 顶点连接关系

\(\delta_i\) 由 `Pose_i` 和 `Pose_i+1` 得到。

因此 \(\dot{\delta}_i\) 需要：

```text
Pose_i
Pose_i+1
Pose_i+2
TimeDiff_i
TimeDiff_i+1
```

也就是一个五顶点边：

```cpp
class EdgeSteeringRate
  : public BaseTebMultiEdge<1, double>
```

构造函数：

```cpp
EdgeSteeringRate()
{
  resize(5);
}
```

顶点顺序建议：

```text
0: Pose_i
1: Pose_i+1
2: Pose_i+2
3: TimeDiff_i
4: TimeDiff_i+1
```

这与现有 `edge_acceleration.h:64` 的组织方式一致。

## 2. 边误差

建议使用区间惩罚：

\[
e_{\dot{\delta}}
=
\operatorname{penaltyBoundToInterval}
\left(
\dot{\delta},
-0.5,
0.5,
\epsilon
\right)
\]

这样：

- 在允许范围内误差为零；
- 超出范围才产生代价；
- 对正常轨迹不会额外施加不必要的阻力。

不要直接使用：

\[
e=\dot{\delta}
\]

否则优化器会强行把所有转角速度压到零，车辆会变得过于迟钝。

---

# 五、第三条边：转角加速度边

转角加速度限制为：

\[
|\ddot{\delta}|\leq1.0\text{ rad/s}^2
\]

先定义：

\[
\dot{\delta}_i=
\frac{\delta_{i+1}-\delta_i}
{\bar{\Delta t}_i}
\]

其中：

\[
\bar{\Delta t}_i=
\frac{\Delta t_i+\Delta t_{i+1}}{2}
\]

然后：

\[
\ddot{\delta}_i=
\frac{\dot{\delta}_{i+1}-\dot{\delta}_i}
{
(\bar{\Delta t}_i+\bar{\Delta t}_{i+1})/2
}
\]

## 1. 顶点连接关系

需要三个连续的段转角：

```text
delta_i
delta_i+1
delta_i+2
```

因此需要四个位姿和三个时间节点：

```text
Pose_i
Pose_i+1
Pose_i+2
Pose_i+3
TimeDiff_i
TimeDiff_i+1
TimeDiff_i+2
```

这是一个七顶点边：

```cpp
class EdgeSteeringAcceleration
  : public BaseTebMultiEdge<1, double>
```

## 2. 边误差

\[
e_{\ddot{\delta}}
=
\operatorname{penaltyBoundToInterval}
\left(
\ddot{\delta},
-1.0,
1.0,
\epsilon
\right)
\]

该边必须使用实际的优化时间差，而不能用固定的 `dt_ref`。

否则 TEB 通过缩短或拉长时间段时，转向加速度约束不会随之变化，物理意义不正确。

---

# 六、速度相关角速度边

前轮最大转角会产生速度相关的角速度约束：

\[
|\omega_i|
\leq
|v_i|\frac{\tan60^\circ}{0.854}
\]

即：

\[
|\omega_i|\leq2.028|v_i|
\]

这个约束最好单独做成一个边：

```cpp
EdgeSteeringKinematics
```

或者合并进现有 `EdgeKinematicsCarlike`。

推荐误差：

\[
e_{\omega}
=
\max
\left(
0,\,
|\omega_i|-\kappa_{\max}|v_i|
\right)
\]

为了保持双侧对称，可以写成：

\[
e_\omega=
\operatorname{penaltyBoundFromBelow}
\left(
\kappa_{\max}|v_i|-|\omega_i|,
0
\right)
\]

但要小心 \(|v|\) 和 \(|\omega|\) 在零点不可导。

更稳定的实现是分段处理：

```cpp
if (std::abs(v) < v_eps)
{
  error = penaltyBoundToInterval(omega, 0.0, omega_eps, epsilon);
}
else
{
  const double omega_limit = std::abs(v) * kappa_max;
  error = penaltyBoundToInterval(omega, -omega_limit, omega_limit, epsilon);
}
```

这个边是整个前轮转向模型的关键，因为它同时保证：

- 最大转角；
- 速度相关最大角速度；
- 低速时不能原地旋转。

---

# 七、建议的加入顺序

不要一次性全部开启，建议按以下顺序验证。

## 第一步：只验证转角上限

保留：

```cpp
EdgeKinematicsCarlike
```

配置：

```yaml
wheelbase: 0.854
min_turning_radius: 0.493
```

验证：

\[
|\delta|\leq60^\circ
\]

这一步理论上现有程序已经支持。

## 第二步：加入 `EdgeSteeringRate`

只增加：

\[
|\dot{\delta}|\leq0.5
\]

暂时不加入转角加速度。

观察：

- 轨迹是否仍然可收敛；
- 是否出现大量不可行；
- 是否产生明显的时间拉长；
- 前轮转角是否连续。

## 第三步：加入 `EdgeSteeringAcceleration`

增加：

\[
|\ddot{\delta}|\leq1.0
\]

如果轨迹突然变得很难收敛，优先检查：

- 时间差是否过小；
- 转角速度边是否已经过于严格；
- 当前轨迹初始化是否存在大幅转角跳变；
- 起始转角是否有边界值。

## 第四步：加入命令级保护

即使优化器内满足约束，重规划后第一条命令仍可能和上一周期命令不同。

因此需要保存上一周期的：

```text
delta_previous
delta_rate_previous
```

并对最终输出进行验证：

\[
|\delta_{\text{cmd}}-\delta_{\text{prev}}|
\leq0.5\Delta t_{\text{control}}
\]

\[
|\dot{\delta}_{\text{cmd}}-\dot{\delta}_{\text{prev}}|
\leq1.0\Delta t_{\text{control}}
\]

---

# 八、如何验证这些边的数学正确性

## 测试 1：直线运动

输入：

```text
x: 0 -> 1
y: 0 -> 0
theta: 0 -> 0
```

应得到：

\[
v>0,\quad
\omega=0,\quad
\kappa=0,\quad
\delta=0
\]

所有转向边误差应为零。

---

## 测试 2：恒定圆弧

假设：

\[
R=1.0\text{ m}
\]

则：

\[
\kappa=1.0\text{ m}^{-1}
\]

\[
\delta=\arctan(0.854)
\approx40.5^\circ
\]

应满足：

- 转角小于 60°；
- 如果每段转角相同，转角速度为零；
- 转角加速度为零。

---

## 测试 3：超过最大转角

选择：

\[
R=0.4\text{ m}
\]

则：

\[
\delta=\arctan(0.854/0.4)
\approx64.9^\circ
\]

应产生最大转角或最小半径约束误差。

---

## 测试 4：转角速度边界

设置：

```text
delta_i   = 0.0 rad
delta_i+1 = 0.1 rad
dt        = 0.2 s
```

则：

\[
\dot{\delta}=0.5\text{ rad/s}
\]

应恰好位于约束边界，误差接近零。

如果：

```text
delta_i+1 = 0.11 rad
```

则：

\[
\dot{\delta}=0.55\text{ rad/s}
\]

应产生正的约束误差。

---

## 测试 5：时间缩放一致性

同样的转角变化：

```text
delta: 0 -> 0.1 rad
```

分别使用：

```text
dt = 0.1 s
dt = 0.2 s
```

应得到：

\[
\dot{\delta}_{0.1}=1.0\text{ rad/s}
\]

\[
\dot{\delta}_{0.2}=0.5\text{ rad/s}
\]

也就是说，延长时间后约束应该变得更容易满足。

---

## 测试 6：倒车

倒车时：

\[
v<0
\]

运动学关系仍然是：

\[
\omega=v\frac{\tan\delta}{L}
\]

因此倒车时：

- \(v\) 符号改变；
- 相同前轮转角会导致相反方向的 \(\omega\)；
- \(\delta=\arctan(L\omega/v)\) 必须保留 \(v\) 的符号。

不能使用：

\[
\delta=\arctan\left(\frac{L|\omega|}{|v|}\right)
\]

否则倒车时会丢失转向方向。

---

## 测试 7：原地旋转

输入：

```text
x 不变
y 不变
theta 发生变化
```

此时：

\[
v=0,\quad\omega\neq0
\]

必须被判定为不可行。

---

## 测试 8：角度跳变

输入：

```text
theta_i     = 179 deg
theta_i+1   = -179 deg
```

应使用归一化后的：

\[
\Delta\theta=2^\circ
\]

不能错误地认为发生了 \(-358^\circ\) 的转向。

现有代码已经使用 `g2o::normalize_theta()`，新边必须保持一致。

---

# 九、Jacobian 的验证

新边最容易出问题的地方不是公式，而是 Jacobian。

建议分三步：

## 1. 先关闭解析 Jacobian

如果工程允许，先让 g2o 使用数值差分，验证：

- 误差函数本身正确；
- 优化趋势正确；
- 轨迹不会发散。

## 2. 再实现解析 Jacobian

转角速度边的变量依赖包括：

```text
Pose_i
Pose_i+1
Pose_i+2
TimeDiff_i
TimeDiff_i+1
```

转角加速度边的变量依赖包括：

```text
Pose_i ... Pose_i+3
TimeDiff_i ... TimeDiff_i+2
```

## 3. 进行有限差分对比

对每个顶点变量扰动：

\[
\frac{\partial e}{\partial x}
\approx
\frac{e(x+\epsilon)-e(x-\epsilon)}{2\epsilon}
\]

与解析 Jacobian 比较，建议相对误差控制在：

\[
10^{-4}\sim10^{-3}
\]

特别检查：

- \(v\approx0\)；
- \(\Delta\theta\approx0\)；
- \(\theta\) 穿越 \(\pm\pi\)；
- 转角约束边界；
- 时间差接近最小值。

---

# 十、一个重要的实现建议

不要在三个边中分别重复计算：

```cpp
v
omega
kappa
delta
```

建议统一放到一个公共函数中，例如：

```cpp
bool computeSteeringSegment(
    const VertexPose& pose1,
    const VertexPose& pose2,
    const VertexTimeDiff& dt,
    SegmentKinematics& segment);
```

所有边都调用它：

```text
EdgeSteeringKinematics
EdgeSteeringRate
EdgeSteeringAcceleration
```

这样可以避免以下问题：

- 角度边用弦长；
- 转角速度边用纵向投影；
- 加速度边又用欧氏距离；
- 三条边最终约束的不是同一个 \(\delta\)。

当前 `edge_acceleration.h:98` 已经采用多个 `VertexTimeDiff`，新边可以沿用这种结构。

---

## 最终建议

可以优先增加转向边，而且推荐顺序是：

```text
现有最小转弯半径边
        ↓
速度相关角速度/转角边
        ↓
转角速度边
        ↓
转角加速度边
        ↓
命令级转向限幅
```

第一步最建议实现的是：

```cpp
EdgeSteeringRate
```

但在它之前必须先建立并验证统一的：

```cpp
computeSteeringSegment()
```

否则后续无法判断转角速度边的误差到底是公式问题，还是段速度、角速度定义不一致。

从正确性角度看：

- 最大转角约束对应最大曲率；
- 转角速度约束对应曲率连续性；
- 转角加速度约束对应曲率变化率连续；
- 零速角速度约束保证不能原地旋转；
- 使用实际 `TimeDiff` 保证约束具有正确的时间尺度。

这套实现可以在不改变 `[x,y,\theta]` TEB 顶点结构的前提下，较完整地表达你的前轮转向车辆约束。