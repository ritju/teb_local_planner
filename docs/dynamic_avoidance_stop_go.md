# 动态避障一走一停：控制层排查与转向滞后降速

## 本次能确认的范围

用户反馈为“速度频繁归零”。本次没有新的动态场景 controller 日志，工具环境也未发现运行中的 Controller Server，因此不能把全部停车归因于同一分支。代码检查和回归能确认一个可复现的控制层启停机制，并提供开关做对照。

当前仓库 YAML 中：

```yaml
include_dynamic_obstacles: false
dynamic_safety_predictable_mode: false
path_wait_enable: false
```

这份配置没有启用基于障碍物速度的时空预测，也没有启用 PATH 走廊等待。移动障碍物仍可通过 costmap/自定义当前轮廓参与静态位置避障。实际运行可能加载其他 YAML 或参数覆盖，应读取运行参数确认，不能据此推断运行中的开关状态。本次不自动打开预测，不降低障碍物权重或碰撞保护。

## 可复现的启停链路

```text
障碍物位置变化 → 优化目标曲率变化
→ 目标速度下的转向误差超过门槛
→ 零速对准 → 已发布线速度变为 0
→ 转向满足门槛后重新斜坡加速
→ 下次目标曲率改变，再次归零
```

原分支以请求线速度评估转向误差，而最终发布速度还会经过纵向加速度斜坡，通常比请求速度小。只考虑请求速度，可能对一条本来可以低速执行的指令做出全停决定。

## 新增可选处理

```yaml
front_steer_drive_interface:
  steering_lag_speed_scaling: true
  max_projection_omega_error: 0.2
```

代码默认 `steering_lag_speed_scaling=false`，保持旧零速对准行为；当前参数文件显式开启。参数在 configure 时固定，修改后重启或重新 configure。

静态可实现性仍按原请求检查，转角目标仍受角度、速率、加速度及制动余量约束。对于转向动态滞后，定义：

```text
k_request = omega_request / v_request
k_command = tan(delta_command) / wheelbase
k_actual  = tan(delta_measured) / wheelbase

k_error = max(abs(k_request-k_command), abs(k_request-k_actual))
v_lag_limit = max_projection_omega_error / k_error
```

误差为零时该上限不限速。只降低线速度，不修改转角，也不扩大角速度误差预算。以允许下发的速度计算，要求：

```text
abs(v_out * (k_request-k_command)) <= max_projection_omega_error
abs(v_out * (k_request-k_actual))  <= max_projection_omega_error
```

参考角速度随着速度降低而同比缩小，并非维持原请求角速度。允许速度仍高于低速门槛时，连续减速通过；允许速度低到门槛以内时，继续采用零速转向对准。

该速度上限也应用到最终发布端，防止正常减速斜坡在一段时间内仍输出高于安全上限的速度。上限解除后按 `acc_lim_x` 恢复速度；更严格的上限允许优先降速。

例如 `v_request=0.3 m/s、omega_request=0.3 rad/s`，实际转角接近零且轴距 0.854 m 时，原分支容易进入对准停车。新分支可先限制到约 0.2 m/s，再经过发布端纵向斜坡从更低速度起步；前轮逐步转到位后逐渐恢复速度。

## 没有改变的停车条件

- 静态曲率不可实现、轨迹运动学或碰撞检查失败。
- 状态缺失/非法/过期，或控制间隔不可信。
- 显式停车、路径等待/控制门控、到点、取消和异常停车。
- 允许行驶速度低于低速门槛。

此开关是转向滞后下的速度调节，不是障碍物通行许可。需要真实转角反馈；推荐使用原生 `/front_steer_stu`，关闭 odom 近似状态转换。底盘自己的超时和制动保护保持开启。

## 现场区分停车来源

| 日志/现象 | 判断方向 |
| --- | --- |
| `Front-steer aligning at zero speed` | 零速对准分支；确认新开关是否生效、实际转角是否跟随 |
| `Front-steer steering lag speed cap` | 新的连续降速分支；`cap` 是上限，实际输出还受加速度斜坡约束 |
| `trajectory ... not feasible` / `kinematics invalid` | 轨迹可行性拒绝；查看段号、转角及障碍物，不应强行保持运动 |
| `missing, invalid or stale chassis feedback` | 状态链路、反馈频率及线程阻塞 |
| `Invalid ... control interval` / `Control loop missed` | 控制耗时及调度问题；不属于纯转向滞后 |
| `PathWait` 进入等待 | 检查运行时开关、障碍物 ID 与离开去抖 |
| 原生指令有零值但 TEB debug 未更新 | 可能来自 Controller Server 的独立停车路径 |
| 指令一直非零但机器人停顿 | 排查执行器、插件指令超时、实际转角及驱动反馈 |

先确认实际加载的参数：

```bash
ros2 param get /controller_server FollowPath.front_steer_drive_interface.steering_lag_speed_scaling
ros2 param get /controller_server FollowPath.include_dynamic_obstacles
ros2 param get /controller_server FollowPath.dynamic_safety_predictable_mode
ros2 param get /controller_server FollowPath.path_wait_enable
```

可在复现场景中记录：

```bash
ros2 bag record /front_steer_cmd /front_steer_stu /odom /dynamic_obstacles /local_plan /rosout
```

检查原生指令归零时同一时刻的日志原因，以及“目标转角—实际转角—实际线速度”的时序；不要只从 RViz 路径摆动推断停车原因。

## 其他代码风险与待确认项

自定义障碍物更新仍有最长 0.5 s 的 TF 查询等待，并在 TF 失败时使用单位变换；若障碍物消息不是 map 帧且 TF 不稳定，这可能造成周期阻塞或障碍物位置跳变。当前常见发布器使用 map 帧，不能在没有现场 TF 错误的情况下认定这是本次原因。应先根据日志和输入帧确认，再针对性处理，不能通过忽略障碍物来掩盖失败。

若启用传统动态预测，同时开启 `dynamic_obstacles_as_static_edges`，当前障碍物也可能同时产生当前位置静态边和预测边。需要按实际使用模式检查权重与建边意图，不应把所有动态相关开关一起打开。

## 回归验证

新增回归在 240 周期内反复切换转向目标，模拟较慢的实际转角反馈，比较旧分支的零速次数与新分支的非零速度，并逐周期验证同一角速度误差预算及转向动态约束。ROS 话题回归覆盖最终速度上限在纵向斜坡后仍生效，以及反馈失效时仍立即停车。该验证不等同于已完成现场动态绕障验收。
