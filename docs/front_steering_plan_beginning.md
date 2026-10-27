我会把第一阶段具体化为“段级控制量 + 位姿顶点”的方案：不扩展 TEB 顶点维度，但从相邻位姿和时间差计算 \(v,\omega,\kappa,\delta\)，再增加转角、转角速度、转角加速度和零速不可旋转约束。重点会说明边的连接方式、离散公式、优化顺序以及当前代码中应改的入口。

下面给出一个“保持现有 TEB 顶点结构不变”的完整第一阶段方案。

核心思想是：

\[
\text{优化顶点仍为 }[x_i,y_i,\theta_i]
\]

但把每一段轨迹隐式解释为：

\[
(v_i,\omega_i,\kappa_i,\delta_i)
\]

其中 \(v,\omega,\kappa,\delta\) 都由相邻位姿和时间差计算得到，不增加新的顶点维度。

---

## 1. 车辆模型和参考点

必须明确 TEB 的位姿参考点是后轴中心。

车辆模型为：

\[
\dot{x}=v\cos\theta
\]

\[
\dot{y}=v\sin\theta
\]

\[
\dot{\theta}=v\kappa
\]

\[
\kappa=\frac{\tan\delta}{L}
\]

其中：

```text
L = 0.854 m
δ  = 前轮转角
κ  = 曲率
v  = 后轴中心纵向速度
ω  = 后轴中心角速度
```

因此：

\[
\omega=v\kappa
\]

\[
\delta=\arctan\left(\frac{L\omega}{v}\right)
\]

如果当前 `base_link` 不在后轴中心，需要先修改 TF 或在 TEB 内部增加参考点偏移。否则即使运动学公式正确，输出的 \(v,\omega\) 也不是后轴中心的速度。

---

## 2. 由位姿节点计算段级变量

对相邻位姿 \(i,i+1\)：

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

\[
\Delta t_i=t_{i+1}-t_i
\]

将位移转换到当前车体坐标系：

\[
s_i=
\cos\theta_i\Delta x+
\sin\theta_i\Delta y
\]

\[
e_i=
-\sin\theta_i\Delta x+
\cos\theta_i\Delta y
\]

其中：

- \(s_i\)：沿车体纵向位移；
- \(e_i\)：横向位移。

段速度定义为：

\[
v_i=\frac{s_i}{\Delta t_i}
\]

角速度定义为：

\[
\omega_i=\frac{\Delta\theta_i}{\Delta t_i}
\]

曲率定义为：

\[
\kappa_i=\frac{\omega_i}{v_i}
\]

前提是 \(|v_i|>v_\epsilon\)。

为了减少差分误差，可以使用平均航向：

\[
\bar{\theta}_i=
\theta_i+\frac{\Delta\theta_i}{2}
\]

然后计算：

\[
s_i=
\cos\bar{\theta}_i\Delta x+
\sin\bar{\theta}_i\Delta y
\]

这样对于圆弧轨迹更准确。

前轮转角：

\[
\delta_i=\arctan(L\kappa_i)
\]

当 \(|v_i|\) 很小时，不能直接计算 \(\omega_i/v_i\)。此时应：

- 直接限制 \(\omega_i\) 接近零；
- 或将该段标记为不可行；
- 不允许通过减小速度来“掩盖”过大的角速度。

---

## 3. 车辆参数

建议新增以下配置参数：

```yaml
robot:
  wheelbase: 0.854
  rear_track: 0.84

  max_steering_angle: 1.04719755       # 60 deg
  max_steering_rate: 0.5               # rad/s
  max_steering_acceleration: 1.0       # rad/s^2

  steering_min_speed: 0.03
  no_inplace_rotation: true
```

现有参数中：

```yaml
robot:
  wheelbase: 0.854
  cmd_angle_instead_rotvel: false
```

必须设置为 `false`，因为输出接口要求的是：

```text
linear.x  = 后轴中心线速度 v
linear.y  = 0
angular.z = 后轴中心角速度 omega
```

当前将角速度转换为前轮角度的代码在 `teb_local_planner_ros.cpp:1570`。这个转换不能作为最终输出格式。

---

## 4. 最大转角约束

最大前轮转角：

\[
\delta_{\max}=60^\circ=1.0472\text{ rad}
\]

对应最大曲率：

\[
\kappa_{\max}
=\frac{\tan(1.0472)}{0.854}
\approx2.028\text{ m}^{-1}
\]

对应最小转弯半径：

\[
R_{\min}
=\frac{1}{\kappa_{\max}}
=\frac{0.854}{\tan60^\circ}
\approx0.493\text{ m}
\]

因此已有的：

```yaml
robot:
  min_turning_radius: 0.493
```

可以继续保留。

但是建议不要只用最小转弯半径约束，而是增加显式转角约束：

\[
|\delta_i|\leq1.0472
\]

等价地，也可以使用：

\[
|\kappa_i|\leq2.028
\]

或者使用不含除法的形式：

\[
|L\omega_i|
\leq
|v_i|\tan\delta_{\max}
\]

这个形式在优化器中通常比 \(\kappa=\omega/v\) 更稳定。

---

## 5. 速度相关的角速度约束

前轮转向车辆的最大角速度不是固定值，而是随线速度变化：

\[
|\omega_i|
\leq
|v_i|\frac{\tan\delta_{\max}}{L}
\]

代入参数：

\[
|\omega_i|\leq2.028|v_i|
\]

例如：

| 后轴速度 \(v\) | 最大角速度 \(|\omega|\) |
|---:|---:|
| 0.2 m/s | 0.406 rad/s |
| 0.5 m/s | 1.014 rad/s |
| 1.0 m/s | 2.028 rad/s |

因此当前固定的 `max_vel_theta` 不足以表达前轮转向约束。

当前速度饱和在 `teb_local_planner_ros.cpp:1566`，需要增加：

```cpp
double omega_limit =
    std::abs(vx) * std::tan(max_steering_angle) / wheelbase;

omega = clamp(omega, -omega_limit, omega_limit);
```

更好的做法是把它直接加入优化边，而不是只在输出阶段裁剪。输出阶段仍可以保留安全检查。

注意：

```yaml
robot:
  max_vel_theta
```

应该作为全局安全上限：

\[
|\omega|\leq \min(
\omega_{\text{configured}},
|v|\kappa_{\max})
\]

而不是单独依赖固定的 `max_vel_theta`。

---

## 6. 禁止原地旋转

前轮转向模型不满足：

\[
v=0,\quad \omega\neq0
\]

所以必须增加：

\[
|v_i|\leq v_\epsilon
\Rightarrow
|\omega_i|\leq\omega_\epsilon
\]

或者直接使用：

\[
|\omega_i|
\leq
\kappa_{\max}|v_i|
\]

这样在 \(v=0\) 时自动要求：

\[
\omega=0
\]

建议额外设置：

```yaml
robot:
  steering_min_speed: 0.03
  no_inplace_rotation: true
```

需要检查并修改以下行为：

- rotate-to-heading；
- 原地旋转恢复行为；
- 目标角度对齐；
- 只输出角速度的控制分支；
- 速度接近零时的角速度命令。

对于前轮转向车，目标角度调整必须通过小速度前进或后退完成，不能直接绕后轴中心原地转。

---

## 7. 转角速度约束

每一段有一个隐式转角：

\[
\delta_i=\arctan(L\kappa_i)
\]

相邻两段转角速度：

\[
\dot{\delta}_i
=
\frac{\delta_{i+1}-\delta_i}
{\bar{\Delta t}_i}
\]

其中：

\[
\bar{\Delta t}_i=
\frac{\Delta t_i+\Delta t_{i+1}}{2}
\]

约束为：

\[
|\dot{\delta}_i|
\leq0.5\text{ rad/s}
\]

离散形式：

\[
\left|
\frac{\delta_{i+1}-\delta_i}
{(\Delta t_i+\Delta t_{i+1})/2}
\right|
\leq0.5
\]

由于 \(\delta_i\) 是由两个位姿得到的，所以一个转角速度边需要连接：

```text
Pose_i
Pose_i+1
Pose_i+2
TimeDiff_i
TimeDiff_i+1
```

可以新增：

```cpp
EdgeSteeringRate
```

它本质上是一个三位姿边。

注意必须使用实际优化后的 \(\Delta t_i\)，不能固定使用 `dt_ref`，因为 TEB 会同时优化时间间隔。

---

## 8. 转角加速度约束

定义相邻转角速度：

\[
\dot{\delta}_i=
\frac{\delta_{i+1}-\delta_i}{\bar{\Delta t}_i}
\]

则转角加速度为：

\[
\ddot{\delta}_i
=
\frac{\dot{\delta}_{i+1}-\dot{\delta}_i}
{\bar{\Delta t}^{\,a}_i}
\]

其中：

\[
\bar{\Delta t}^{\,a}_i
=
\frac{
\bar{\Delta t}_i+
\bar{\Delta t}_{i+1}
}{2}
\]

约束：

\[
|\ddot{\delta}_i|
\leq1.0\text{ rad/s}^2
\]

这需要三个连续的段转角：

\[
\delta_i,\delta_{i+1},\delta_{i+2}
\]

因此需要连接四个位姿：

```text
Pose_i
Pose_i+1
Pose_i+2
Pose_i+3
```

可以新增：

```cpp
EdgeSteeringAcceleration
```

这是四位姿边。

---

## 9. 建议新增的优化边

当前 carlike 边只包含非完整约束和最小转弯半径，见 `edge_kinematics.h:163`。

建议增加以下边：

### `EdgeSteeringAngleLimit`

连接：

```text
Pose_i, Pose_i+1, TimeDiff_i
```

约束：

\[
|\delta_i|\leq\delta_{\max}
\]

或：

\[
|\kappa_i|\leq\kappa_{\max}
\]

### `EdgeSteeringRate`

连接：

```text
Pose_i, Pose_i+1, Pose_i+2,
TimeDiff_i, TimeDiff_i+1
```

约束：

\[
|\dot{\delta}_i|\leq0.5
\]

### `EdgeSteeringAcceleration`

连接：

```text
Pose_i, Pose_i+1, Pose_i+2, Pose_i+3,
TimeDiff_i, TimeDiff_i+1, TimeDiff_i+2
```

约束：

\[
|\ddot{\delta}_i|\leq1.0
\]

### `EdgeNoInplaceRotation`

连接：

```text
Pose_i, Pose_i+1, TimeDiff_i
```

约束：

\[
v_i\approx0
\Rightarrow
\omega_i\approx0
\]

如果已经使用：

\[
|L\omega_i|\leq|v_i|\tan\delta_{\max}
\]

那么这个边可以与转角边合并，但建议保留单独的零速保护，便于调试。

---

## 10. 曲率连续性

对于位姿顶点模型，曲率是“段变量”：

\[
\kappa_i=
\frac{\omega_i}{v_i}
\]

因此曲率连续可以写成：

\[
|\kappa_{i+1}-\kappa_i|
\]

或者更符合转向执行器的形式：

\[
|\delta_{i+1}-\delta_i|
\]

由于：

\[
\delta=\arctan(L\kappa)
\]

限制转角变化率就等价于限制曲率变化率。

建议至少加入：

\[
\sum_i
w_{\delta\_rate}
(\delta_{i+1}-\delta_i)^2
\]

以及：

\[
\sum_i
w_{\delta\_acc}
(\dot{\delta}_{i+1}-\dot{\delta}_i)^2
\]

但是代价项不能代替硬约束，实际边界仍要通过 `penaltyBoundToInterval` 一类的约束实现。

---

## 11. 优化图的加入位置

当前优化图在 `optimal_planner.cpp:400` 附近添加速度、加速度和运动学边。

建议把新约束放在：

```cpp
AddEdgesVelocity();
AddEdgesAcceleration();
AddEdgesKinematicsCarlike();
AddEdgesSteeringLimits();
AddEdgesSteeringRate();
AddEdgesSteeringAcceleration();
```

其中：

```cpp
AddEdgesKinematicsCarlike()
```

保留：

- 非完整约束；
- 基本曲率/最小转弯半径约束。

新增：

```cpp
void AddEdgesSteeringLimits();
void AddEdgesSteeringRate();
void AddEdgesSteeringAcceleration();
void AddEdgesNoInplaceRotation();
```

模型选择仍可保留当前逻辑，但建议以后增加明确的车辆模型开关，例如：

```yaml
robot:
  motion_model: carlike
```

不要再仅通过 `min_turning_radius != 0` 推断车型。

---

## 12. 目标函数建议

可以使用：

\[
J=
J_{\text{obstacle}}
+J_{\text{goal}}
+J_{\text{time}}
+J_{\text{kinematics}}
+J_{\text{velocity}}
+J_{\text{steering}}
\]

其中转向部分：

\[
J_{\text{steering}}
=
w_\delta\sum_i\delta_i^2
+
w_{\dot\delta}\sum_i\dot{\delta}_i^2
+
w_{\ddot\delta}\sum_i\ddot{\delta}_i^2
\]

建议优先级：

1. 非完整约束权重最高；
2. 最大转角和最大转角速度作为强约束；
3. 转角加速度作为强约束或高权重约束；
4. 曲率/转角变化作为平滑代价；
5. 在不影响避障的情况下再优化时间和路径长度。

初始权重可以按照：

```yaml
optim:
  weight_kinematics_nh: 1000
  weight_kinematics_turning_radius: 1000
  weight_steering_angle: 500
  weight_steering_rate: 300
  weight_steering_acceleration: 200
  weight_steering_smoothness: 10
```

实际还需要根据 TEB 其他边的量纲重新调整。关键不是绝对数值，而是转向约束不能被障碍物、时间最优或路径长度代价“压过去”。

---

## 13. 命令输出阶段

当前 `getVelocityCommand()` 已经输出：

```cpp
vx
vy
omega
```

实现位于 `optimal_planner.cpp:1556`。

这部分可以继续保留，但需要保证：

```cpp
vy = 0.0;
```

输出流程建议为：

```text
TEB优化
  ↓
提取后轴中心 v, omega
  ↓
检查 |omega| <= |v| * tan(delta_max) / wheelbase
  ↓
检查与上一条命令对应的 steering rate
  ↓
检查 steering acceleration
  ↓
输出 Twist:
  linear.x  = v
  linear.y  = 0
  angular.z = omega
```

不要在这里把 `omega` 转换为 `delta`，因为你的底盘接口要求的是角速度。

---

## 14. 命令级保护

仅靠轨迹内部约束还不够，因为每次重规划可能会导致第一段命令变化较大。

建议保存上一周期输出：

```text
v_prev
omega_prev
delta_prev
delta_rate_prev
```

将当前输出换算为：

\[
\delta_{\text{cmd}}
=
\arctan\left(
\frac{L\omega_{\text{cmd}}}{v_{\text{cmd}}}
\right)
\]

然后进行命令级保护：

\[
|\delta_{\text{cmd}}-\delta_{\text{prev}}|
\leq0.5\Delta t_{\text{ctrl}}
\]

\[
|\dot{\delta}_{\text{cmd}}-\dot{\delta}_{\text{prev}}|
\leq1.0\Delta t_{\text{ctrl}}
\]

如果超过限制，优先降低角速度，必要时降低线速度，而不是直接把 \(\omega\) 截断后继续发送。

更好的方式是将当前转角作为第一段轨迹的边界条件：

\[
\delta_0=\delta_{\text{measured}}
\]

如果暂时没有真实转角反馈，可以使用上一周期命令估计。

---

## 15. 后轮轮距的使用方式

后轮轮距：

\[
W=0.84\text{ m}
\]

不参与理想自行车模型的中心运动学，主要用于：

### 左右后轮速度换算

\[
v_L=v-\frac{W}{2}\omega
\]

\[
v_R=v+\frac{W}{2}\omega
\]

### 碰撞检测

需要将车体真实外形配置到 `footprint_model` 中，不能只使用一个过小的圆形 footprint。

### 轮速限制

如果左右后轮有最大轮速，还需要增加：

\[
|v_L|\leq v_{\text{wheel,max}}
\]

\[
|v_R|\leq v_{\text{wheel,max}}
\]

这可以形成额外的速度约束。

---

## 16. 第一阶段最终方案

推荐最终采用以下结构：

```text
TEB顶点：
[x, y, theta]

每段隐式变量：
v_i
omega_i
kappa_i
delta_i

约束：
1. 非完整约束
2. |delta| <= 60 deg
3. |kappa| <= 2.028 1/m
4. |omega| <= |v| * 2.028
5. |delta_dot| <= 0.5 rad/s
6. |delta_ddot| <= 1.0 rad/s^2
7. v ≈ 0 时 omega ≈ 0
8. vy = 0
9. 输出为后轴中心 v, omega
```

这套方案不改变 TEB 顶点维度，但已经能够表达：

- 前轮最大转角；
- 速度相关的最大角速度；
- 曲率连续；
- 前轮转角速度限制；
- 前轮转角加速度限制；
- 前轮转向车不能原地旋转；
- 后轮中心速度输出。

其中最重要的三个新增约束是：

\[
|L\omega|\leq|v|\tan\delta_{\max}
\]

\[
|\dot{\delta}|\leq0.5
\]

\[
|\ddot{\delta}|\leq1.0
\]

这比单纯设置 `min_turning_radius` 更完整，也比直接把优化状态扩展为 \([x,y,\theta,v,\delta]\) 的改动小很多。