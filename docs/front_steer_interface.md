# 前轮转向底盘接口部署

完整控制原理、执行层职责及状态流程见 [前轮转向后桥驱动控制方法](front_steer_rear_drive_control.md)。

`FollowPath.front_steer_drive_interface.mode` 默认是 `front_steer_cmd`。新消息通过 `rclcpp::SystemDefaultsQoS()` 通信，TEB 内部和 Nav2 返回值始终使用后轴中心 `v/omega`。

```yaml
front_steer_drive_interface:
  mode: front_steer_cmd       # cmd_vel | front_steer_cmd
  publish_twist: false        # Requires the matching workspace nav2_controller
  legacy_command_topic: /cmd_vel
  command_topic: /front_steer_cmd
  state_topic: /front_steer_stu
  state_timeout: 0.5
  max_projection_omega_error: 0.2 # rad/s
```

这些参数在 configure 时读取，运行时修改会被拒绝。直连模式要求启用 `front_steering_rear_drive`，关闭 `cmd_angle_instead_rotvel`，并配置有效轴距、转角及动态限制。

两个模式均订阅状态：有效且新鲜时，状态覆盖 Nav2 传入速度，供规划、停车和旋转判断使用。传统模式允许状态缺失或超时后回退；直连模式必须等待有效状态，故障时发布 `{0,0}` 并抛出 Nav2 控制异常。状态时效使用本地单调时钟，不受 ROS 仿真时间暂停或回跳影响。`front_wheel_speed` 仅做有限值检查，不代替 `linear_x`。

直连模式复用已有转向投影，使用实际控制间隔和上一条已发布指令约束转角变化。轨迹首段运动学、静态可实现性和最终指令检查失败时，清理规划结果并停车。`max_projection_omega_error` 限制速度饱和后的角速度与静态可实现角速度之差，并作为动态/实际转角是否允许行驶的判据，需通过仿真及台架标定。动态转向尚未满足该判据时，进入下文的零速转向对准阶段。运动或转向中的控制间隔超过 `state_timeout` 时拒绝该周期；已下发显式零指令、转角及速率历史为零且反馈新鲜静止时，允许用正常控制周期重新启动。显式停车优先于转角速率和加速度限制。

插件在传统模式通过 Nav2 Controller 接口返回 Twist；`legacy_command_topic` 是部署说明值，不能改变 Controller Server 发布器。实际话题需在 Controller Server 的 launch 中 remap。直连模式仍返回 Twist 供 Nav2 内部使用，可通过下文的 `publish_twist: false` 禁止 Controller Server 的旧速度话题下发，并禁用旧 Ackermann 桥接节点。插件不重复发布旧 Twist，也不使用定时器重复下发历史底盘指令。底盘仍需自身的指令超时停车机制。

首次部署需先重新编译包含 `FrontSteerDriveCmd`、`FrontSteerDriveStu` 的 `capella_ros_msg`，再编译本包；本适配没有修改消息定义。测试目标 `teb_front_steer_interface_test` 覆盖状态换算、倒车、低速、异常值、超时恢复、消息映射及配置错误。

## rqt 查看和画图

加载已编译的 `capella_ros_msg` 环境后，`rqt_topic` 可直接展开自定义消息；`rqt_plot` 也可按数值字段订阅。若 rqt 所在环境没有该消息类型，需要先 source 工作区的 `install/setup.bash`，再启动 rqt。

为便于画图，接口额外发布以下标准消息。下表为默认话题；覆盖 `command_topic` 或 `state_topic` 时，对应调试话题的前缀随之变化。

| 话题 | 类型 | 含义/单位 |
| --- | --- | --- |
| `/front_steer_cmd/debug/linear_x` | `std_msgs/msg/Float64` | 实际下发后桥中心线速度，m/s |
| `/front_steer_cmd/debug/front_steer_angle` | `std_msgs/msg/Float64` | 实际下发前轮转角，rad，左正 |
| `/front_steer_cmd/debug/twist` | `geometry_msgs/msg/Twist` | `linear.x` 为实际下发线速度；`angular.z = v*tan(delta)/L`，rad/s；其他分量为 0 |
| `/front_steer_stu/debug/linear_x` | `std_msgs/msg/Float64` | 接收到的后桥中心实际线速度，m/s |
| `/front_steer_stu/debug/front_steer_angle` | `std_msgs/msg/Float64` | 接收到的实际前轮转角，rad |
| `/front_steer_stu/debug/front_wheel_speed` | `std_msgs/msg/Float64` | 接收到的转向关节转速，rpm |

指令调试话题仅在 `front_steer_cmd` 模式发布，镜像完成保护检查后实际下发的消息，包括故障、低速和 lifecycle 停车时的零值。Twist 的 `angular.z` 是角速度，转角应查看独立的 `front_steer_angle` 标量。这些话题只用于观测，不要 remap 为 `/cmd_vel` 或接到底盘控制输入。

反馈调试话题在两种模式均可用，每收到一条反馈就发布一次原始字段，包括非法 NaN/Inf，便于发现底盘问题；它们不代表状态已通过有效性检查。反馈停止后不重复发布旧值，也不伪造零值，因此图上的最后一个值不能用来判断状态仍然新鲜。所有调试话题采用 `SystemDefaultsQoS()`；cleanup 时随接口一起清理。

例如比较控制和实际转角：

```bash
ros2 run rqt_plot rqt_plot /front_steer_cmd/debug/front_steer_angle/data /front_steer_stu/debug/front_steer_angle/data
```

查看 Twist 的线速度和角速度：

```bash
ros2 run rqt_plot rqt_plot /front_steer_cmd/debug/twist/linear/x /front_steer_cmd/debug/twist/angular/z
```

## 仿真：从 /odom 生成底盘状态

这是旧的近似反馈工具，无法观测静止转向，不适用于当前零速对准闭环。其正半径分支填入的是滚动 rpm，与已确认的转向关节 rpm 约定不同；原生闭环应关闭此转换，单独调试时保持 `wheel_radius: 0`。

在 `teb_params.yaml` 的 `FollowPath.front_steer_drive_interface` 下配置：

```yaml
odom_to_state:
  enabled: true       # 默认 false；修改后重新 configure/重启 controller_server
  topic: /odom
  angular_deadband: 0.01 # rad/s，静止时的角速度噪声死区
  wheel_radius: 0.0   # m；0 表示未知，front_wheel_speed 填 0；配置实际半径后计算 rpm
```

开启后，控制器 configure 时创建转换订阅和发布器，cleanup 时释放。输入为 `nav_msgs/msg/Odometry`，使用 SensorDataQoS（兼容 best-effort 和 reliable 发布器）；输出为 `capella_ros_msg/msg/FrontSteerDriveStu`，发布到 `state_topic`（默认 `/front_steer_stu`），使用 SystemDefaultsQoS。开关和其他接口参数一样在 configure 时固定，关闭时不会订阅 odom 或发布模拟状态。仅用于仿真，开启时需停用同一状态话题上的真实底盘发布器。

转换直接复用 `FollowPath.wheelbase: 0.854`、`rear_track: 0.84`、`max_steering_angle` 和 `steering_min_speed`。要求 odom 的 twist 已在车辆坐标系中表示后轴中心速度（x 前、y 左、z 上）；不进行 TF 或速度参考点变换。仿真器若输出车体中心速度，需要先转换到后轴中心。

- 后轴中心速度：`linear_x = odom.twist.twist.linear.x`。
- 等效前轮转角：`delta = atan(wheelbase * angular.z / linear.x)`，使用 atan 保证倒车时转角符号正确。
- 左右后轮速度分别为 `v - omega * rear_track / 2` 和 `v + omega * rear_track / 2`，参与数值合法性检查；当前状态消息没有这两个字段。轮距不参与等效前轮转角计算。
- 配置正数轮胎半径后，诊断轮速按等效前轮滚动速度计算：`rpm = v / cos(delta) / wheel_radius * 60 / (2*pi)`，倒车为负。未配置半径时的 0 代表未知轮速。

所有 twist 分量必须有限。三个线速度分量的绝对值都不超过 `steering_min_speed`（当前 0.03 m/s），且三个角速度分量的绝对值都不超过 `odom_to_state.angular_deadband`（默认 0.01 rad/s）时，视为静止噪声，将 `linear_x`、`front_steer_angle`、`front_wheel_speed` 全部设为 0 并正常发布，保持反馈新鲜，避免近零值触发超时。边界值包含在死区内。仅对确认近零的有限值归零，不把 NaN/Inf 转成停车状态。

超出静止死区时，侧向、竖向线速度及 x/y 角速度绝对值仍不得超过 `1e-6`，且 `|v| <= steering_min_speed` 时的明显旋转仍被拒绝。轴距/轮距必须为有限正数，半径和角速度死区为有限非负数；非法配置在 configure 时拒绝。非有限数、float32 溢出及超过配置最大转角的输入仍被丢弃并节流告警。

每条有效 odom 仅转换发布一次，不使用定时器重发旧数据。丢弃非法消息或 odom 停止后，已有反馈按 `state_timeout` 超时。转换以收到消息为准，不检查 ROS header 时间戳；避免将录包旧数据作为实时仿真输入。此转换只提供反馈，不负责把 `/front_steer_cmd` 转换成仿真器的控制指令。

## 静止时出现 projection exceeds 告警

`Front-steer projection exceeds max_projection_omega_error` 表示规划指令经过前轮转角、转向速率/加速度及低速限制后，角速度变化量超过配置阈值；它不表示 odom 转换失败。导航任务处于 FOLLOW 状态时，即使机器人实际静止，控制器仍可能计算非零指令。修复前，转向动态限制也被当作投影故障，停车后将历史清零，导致启动时反复触发保护。修复后静态曲率不可实现仍报错，正常转向建立过程改为零速对准。

静态不可实现告警会打印 `v`、`omega_before`、`limit`、`previous_delta`、`previous_rate`、`measured_delta` 和 `dt`。`v/omega_before` 为规划指令，`measured_delta` 为实际转角反馈，两者不能混同。先根据这些数值区分低速旋转请求、曲率超限或启动时的转向动态限制，再调整轨迹或参数；不要仅因机器人静止就绕过检查或增大阈值。如果没有导航任务，应确认 Controller Server 是否仍在执行 FollowPath。

## 关闭 Controller Server 的 Twist 下发

```yaml
front_steer_drive_interface:
  mode: front_steer_cmd
  publish_twist: false
```

`publish_twist` 的代码默认值为 `true`，本仓库 `teb_params.yaml` 显式设置为 `false`。该参数在 configure 时固定，修改后需要重启/重新 configure。只在当前选中控制器的模式为 `front_steer_cmd` 时生效；`cmd_vel` 模式始终保留 Twist 输出。尚未选择控制器时，如果仅配置了一个控制器，也按该控制器的设置处理启动或 lifecycle 的零速度消息。

Twist 由 `nav2_controller::ControllerServer` 发布，因此必须同时重新构建并加载本工作区的 `nav2_controller` 和 `teb_local_planner`。继续使用未适配的系统 Controller Server，或只更新 TEB 插件，都不能实现该发布开关。

关闭后，Controller Server 不向其速度输出话题发送 Twist，包括正常速度与零速度消息；TEB 的返回值和内部 `last_cmd_` 仍保存真实规划速度，保持 Nav2 内部处理。`/front_steer_cmd` 指令及其安全停车逻辑继续工作，`/front_steer_cmd/debug/twist` 观测话题仍保留。该设置不控制 velocity_smoother、behavior_server 或其他节点创建的发布器；实际底盘仍应只消费已选定的指令链，并保留指令超时保护。


## 转向建立与统一控制间隔

当请求的曲率在车辆静态限制内，但动态转角或实际反馈转角对应的角速度偏差仍大于 `max_projection_omega_error` 时，TEB 输出 `linear_x=0` 和逐步变化的非零 `front_steer_angle`。此时车体 `v/omega` 都为 0，转向关节按既有速率和加速度限制建立角度。已下发角度和速率作为下一周期历史保留，满足动态与实测转角判据后恢复行驶，不再因正常转向建立而清空历史。

零速对准同样要求新鲜有效的状态和可信控制间隔，最终校验仍检查转角、速率、加速度；发生故障或显式停车仍发送 `{0,0}`。需要底盘支持静止转向并持续发布真实转角；本仓库 Gazebo tricycle 插件支持这一行为。仅凭 `/odom` 无法观测静止转向，使用该对准策略时应关闭 odom 状态转换并使用原生 `/front_steer_stu`。

投影阶段计算的控制间隔会固定供本周期最终动态校验与速率历史更新使用，避免可视化、日志等发布前耗时改变加速度推导。最终发布仍独立检查真实距离上次发布的时间和反馈时效，不能通过固定间隔绕过超时保护。


## 大转角、变速时的动态超限

`delta=-1.007 rad` 小于当前最大转角 `1.047 rad`；若同一条日志的原因是 `steering acceleration limit`，应检查转角历史的一阶变化，而非增大最大转角。

直连投影采用带制动余量的角度速率区间，使指令在接近目标和转角边界时提前减小转向速率。角度确定后，车体角速度超过上限时降低线速度，保持转角及其动态约束；不再将截断后的角速度按原线速度反算为另一转角。最终拒绝日志包含 `previous_delta/previous_rate`；轨迹运动学拒绝日志包含失败段号、段速度、角速度、转角及上限，便于区分优化轨迹不合法和最终指令动态不合法。


## 线速度加速度约束

`front_steer_cmd` 最终发布端复用 `FollowPath.acc_lim_x` 约束正常线速度指令的变化，直连模式要求该参数为有限正数。它使用上一条实际下发速度作为历史，按 `acc_lim_x × dt` 生成斜坡；正常换向先降至零再反向起步。当前 `acc_lim_x: 0.3` 在 10 Hz 下对应每周期约 0.03 m/s 的变化量。

斜坡中的非零小速度不会因为 `steering_min_speed` 而被当成停车，从而避免起步历史反复归零。转角保持不变，车体角速度按最终速度重算，Nav2 返回值和调试话题同步反映最终下发值。明确停车、故障停车、零速对准，以及更严格速度/角速度边界要求的降速优先于斜坡。此约束作用于 ROS 指令，不能替代驱动器的实际加速度及制动控制。


## 终点、取消与控制器停车

到达终点后，TEB 插件不会再收到常规计算调用。Controller Server 的 `publishZeroVelocity()` 现会先向当前直连控制器配置的 `command_topic` 发布原生 `{linear_x: 0, front_steer_angle: 0}`，再处理旧 Twist 停车；前者不受 `publish_twist: false` 影响。正常到点、取消、异常以及控制门控停车共用该路径，生命周期停用时也执行。底盘自身超时保护仍需保留。

Controller Server 在 configure 时创建、activate 时启用一个仅用于停车的原生发布器，正常运动指令仍只由 TEB 发布。因此原生指令话题可能显示 TEB 和 Controller Server 两个发布者，后者只发送零值；状态反馈话题仍应只有一个真实状态源。该补发不经过 TEB 自己的 debug 镜像，判断最终是否停车应查看原生 `/front_steer_cmd`，不能仅依据最后一个 TEB debug 标量值。


## 动态绕障时减少反复零速对准

`steering_lag_speed_scaling: true` 按目标曲率、指令转角和实测转角的差异计算连续速度上限，在相同 `max_projection_omega_error` 预算内降速，不再仅因目标速度下的转向滞后频繁全停。当前 YAML 已开启，代码默认关闭以保持兼容。最终发布端同样执行该速度上限；允许速度过低、真实不可行或状态/时序失效时仍停车。计算方法与现场排查见 [动态避障启停排查](dynamic_avoidance_stop_go.md)。
