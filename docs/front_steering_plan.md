当前程序已经有一部分“前轮转向/汽车模型”支持，但目前主要是“差速模型 + 最小转弯半径”的近似，还没有完整利用你给出的前轮转向动态约束。

## 1. 当前程序已有的能力

当前轨迹节点仍然是：

\[
[x_i,y_i,\theta_i]
\]

速度通过相邻位姿计算：

\[
v_x,\;v_y,\;\omega=\frac{\Delta\theta}{\Delta t}
\]

对应实现位于 `optimal_planner.cpp:1556`。

当前已经有两种运动学边：

- 差速模型：`AddEdgesKinematicsDiffDrive()`
- 汽车型模型：`AddEdgesKinematicsCarlike()`

模型选择逻辑位于 `optimal_planner.cpp:414`。

当设置：

```yaml
robot:
  min_turning_radius: 非零
```

程序就会使用 carlike 运动学约束。当前 carlike 约束主要是：

1. 非完整约束，禁止横向运动；
2. 最小转弯半径约束。

对应实现位于 `edge_kinematics.h:163`。

但是，它还没有直接约束：

- 前轮最大转角；
- 前轮转角速度；
- 前轮转角加速度；
- 曲率变化率。

---

## 2. 你的车辆参数换算

给定：

```text
前轮转向角速度上限：0.5 rad/s
前轮转向角加速度上限：1.0 rad/s²
最大前轮转角：60 deg
前轮到后轴距离：0.854 m
后轮轮距：0.84 m
```

这里轴距为：

\[
L=0.854\text{ m}
\]

最大转角：

\[
\delta_{\max}=60^\circ=1.0472\text{ rad}
\]

最大曲率：

\[
\kappa_{\max}
=\frac{\tan\delta_{\max}}{L}
=\frac{\sqrt{3}}{0.854}
\approx 2.03\text{ m}^{-1}
\]

最小转弯半径：

\[
R_{\min}
=\frac{L}{\tan\delta_{\max}}
\approx 0.493\text{ m}
\]

因此至少需要设置：

```yaml
robot:
  wheelbase: 0.854
  min_turning_radius: 0.493
  cmd_angle_instead_rotvel: false
```

这里 `cmd_angle_instead_rotvel` 必须为 `false`，因为你要求输出的是：

```text
后轮中心线速度 v
后轮中心角速度 omega
```

而不是前轮转角。

当前程序如果把该参数设为 `true`，会把 `cmd_vel.angular.z` 从角速度转换成前轮转角，代码在 `teb_local_planner_ros.cpp:1570`。这与你的接口定义不一致。

---

## 3. 输出的角速度应该如何解释

对于后轮中心参考点的自行车模型：

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

所以：

\[
\omega=\dot{\theta}
=v\frac{\tan\delta}{L}
\]

反过来：

\[
\delta=\arctan\left(\frac{L\omega}{v}\right)
\]

因此输出给底盘的应该是：

```text
linear.x  = 后轴中心线速度 v
linear.y  = 0
angular.z = 后轴中心角速度 omega
```

底盘控制器再根据：

\[
\delta=\arctan\left(\frac{L\omega}{v}\right)
\]

换算成前轮转角。

注意：当 \(v\) 接近零时，这个换算会变得病态。前轮转向车不能像差速车一样原地旋转，因此还需要关闭或修改“原地转向/旋转到目标角度”的逻辑。

---

## 4. 仅设置最小转弯半径还不够

当前 carlike 模型使用：

\[
R=\frac{v}{\omega}
\]

来限制最小转弯半径。这只能保证：

\[
|\delta|\leq 60^\circ
\]

但不能保证前轮转角变化平滑。

例如，相邻两个轨迹点可能出现：

```text
delta_1 = 0 deg
delta_2 = 40 deg
delta_3 = -20 deg
```

虽然每个点的转角都没有超过 60°，但是车辆实际不可能以有限转向速度完成这种变化。

所以需要增加至少两类约束：

### 转角速度约束

\[
|\dot{\delta}|\leq 0.5\text{ rad/s}
\]

离散形式：

\[
\left|
\frac{\delta_{i+1}-\delta_i}{\Delta t_i}
\right|
\leq 0.5
\]

### 转角加速度约束

\[
|\ddot{\delta}|\leq 1.0\text{ rad/s}^2
\]

离散形式：

\[
\left|
\frac{
\frac{\delta_{i+1}-\delta_i}{\Delta t_i}
-
\frac{\delta_i-\delta_{i-1}}{\Delta t_{i-1}}
}{
(\Delta t_i+\Delta t_{i-1})/2
}
\right|
\leq 1.0
\]

---

## 5. 建议的优化状态

### 方案一：较小改动，继续使用 TEB 的位姿节点

仍然优化：

\[
[x_i,y_i,\theta_i]
\]

由相邻节点推导：

\[
v_i,\quad \omega_i,\quad
\delta_i=\arctan\left(\frac{L\omega_i}{v_i}\right)
\]

然后增加新的边：

```text
EdgeSteeringAngle
EdgeSteeringRate
EdgeSteeringAcceleration
```

这些边分别约束：

\[
|\delta_i|\leq 1.0472
\]

\[
|\delta_{i+1}-\delta_i|/\Delta t_i\leq0.5
\]

\[
|\dot\delta_{i+1}-\dot\delta_i|/\Delta t\leq1.0
\]

优点是不用改变 TEB 顶点结构，改动相对小。

缺点是：

- 当 \(v\to0\) 时，\(\delta=\arctan(L\omega/v)\) 不稳定；
- 转角只是由位姿差分反推；
- 优化器对转向动态的表达不够自然。

---

### 方案二：更完整，增加转角状态

推荐的时间轨迹状态是：

\[
[x_i,y_i,\theta_i,v_i,\delta_i]
\]

控制量为：

\[
[a_i,\dot{\delta}_i]
\]

运动学：

\[
\dot{x}=v\cos\theta
\]

\[
\dot{y}=v\sin\theta
\]

\[
\dot{\theta}=\frac{v}{L}\tan\delta
\]

\[
\dot{v}=a
\]

\[
\dot{\delta}=\dot{\delta}_{cmd}
\]

约束：

\[
|\delta|\leq1.0472
\]

\[
|\dot{\delta}|\leq0.5
\]

\[
|\ddot{\delta}|\leq1.0
\]

这是最符合实际前轮转向车辆的建模方式，但需要扩展 TEB 的顶点和 g2o 边，改动较大。

---

## 6. 对当前程序最重要的适配点

### A. 运动学模型选择

必须确保不再走差速模型：

```yaml
robot:
  min_turning_radius: 0.493
```

并保证：

```yaml
optim:
  weight_kinematics_nh: 较大
  weight_kinematics_turning_radius: 较大
```

否则程序会把车辆当成可以原地旋转的差速车。

### B. 速度约束必须变成速度相关的角速度约束

前轮最大转角并不对应一个固定的 `max_vel_theta`，而是：

\[
|\omega|
\leq
|v|\frac{\tan 60^\circ}{0.854}
\approx 2.03|v|
\]

也就是说：

```text
v = 0.2 m/s 时，omega 最大约为 0.406 rad/s
v = 0.5 m/s 时，omega 最大约为 1.014 rad/s
v = 1.0 m/s 时，omega 最大约为 2.028 rad/s
```

当前程序在 `teb_local_planner_ros.cpp:1566` 使用固定的 `max_vel_theta` 做饱和，这对前轮转向模型不够准确。

应增加类似：

```cpp
omega_limit = std::abs(vx) * std::tan(delta_max) / wheelbase;
omega = clamp(omega, -omega_limit, omega_limit);
```

但更理想的是把它作为优化约束，而不是最后简单裁剪，否则裁剪后轨迹和实际控制指令可能不一致。

### C. 增加转向速度和加速度约束

需要新增配置参数：

```yaml
robot:
  max_steering_angle: 1.04719755
  max_steering_rate: 0.5
  max_steering_acceleration: 1.0
```

当前配置结构中只有 `wheelbase` 和 `min_turning_radius`，见 `teb_config.h:194`，需要扩展参数声明、读取、动态更新和校验。

### D. 禁止原地旋转

前轮转向车不满足：

```text
v = 0, omega != 0
```

因此以下逻辑需要检查：

- rotate-to-heading；
- 原地旋转恢复行为；
- 目标角度对齐；
- `omega` 单独规划；
- `v` 很小时的角速度输出。

当 \(v\) 接近零时，应该：

- 降低 \(\omega\)；
- 或先规划一段前进/后退运动；
- 不允许直接输出原地旋转命令。

### E. 后轮轮距 0.84 m

后轮轮距对理想自行车模型的中心运动学没有直接影响，因为输出的是后轴中心速度和角速度。

它主要影响：

1. 车体 footprint；
2. 后轮左右轮速度；
3. 车辆碰撞边界；
4. 如果需要差速后轮驱动，则计算左右轮速度。

如果后轮为左右两个驱动轮，则：

\[
v_L=v-\frac{\omega W}{2}
\]

\[
v_R=v+\frac{\omega W}{2}
\]

其中：

\[
W=0.84\text{ m}
\]

但 TEB 仍然输出后轴中心的 \(v,\omega\)，底盘驱动节点负责换算左右轮速度。

---

## 7. 推荐改造路线

我建议分两阶段进行。

### 第一阶段：保持现有 TEB 顶点结构

继续使用：

\[
[x,y,\theta]
\]

但增加：

- 曲率/转角上限；
- 速度相关的角速度上限；
- 转角速度约束；
- 转角加速度约束；
- 禁止原地旋转；
- 输出保持为后轴中心 \(v,\omega\)。

这是改动最小、风险最低的方案。

### 第二阶段：扩展为完整车辆模型

将轨迹状态扩展为：

\[
[x,y,\theta,v,\delta]
\]

控制量扩展为：

\[
[a,\dot{\delta}]
\]

这样可以真正让优化器同时考虑：

- 车辆位置；
- 航向；
- 速度；
- 前轮转角；
- 加速度；
- 转向速度；
- 转向加速度。

## 结论

你的车辆不能仅通过设置 `wheelbase=0.854` 和 `min_turning_radius=0.493` 就完成适配。当前程序还需要重点处理：

1. 将车体参考点明确为后轴中心；
2. 输出 `v` 和后轴中心角速度 `omega`，不要转换成前轮转角；
3. 将最大前轮转角转换为速度相关的角速度约束；
4. 增加前轮转角速度 \(0.5\ \mathrm{rad/s}\) 约束；
5. 增加前轮转角加速度 \(1.0\ \mathrm{rad/s^2}\) 约束；
6. 禁止原地旋转；
7. 使用 \(0.84\) m 轮距完善 footprint 或左右后轮速度换算；
8. 最终最好增加曲率或转角状态，以保证曲率连续。