# 前轮转向、前轮驱动的 Gazebo 功能仿真

车辆控制原理、TEB 流程与后桥执行层说明见 [前轮转向后桥驱动控制方法](front_steer_rear_drive_control.md)。

本模型用于验证后轴中心 `v/delta` 接口和导航运动学。实际驱动力施加在前轮，不能代替后桥驱动的牵引力/打滑测试。

## 修改内容

- `gazebo_plugins/src/gazebo_ros_tricycle_drive.cpp`：新增 `command_interface=front_steer_cmd` 模式；该模式只订阅自定义指令，避免与 `/cmd_vel` 同时控制。默认 `cmd_vel` 模式仍使用原有的“前轮速度 + 转角”语义，`angular.z` 不是标准 Twist 角速度。两种模式均检查指令合法性并执行超时停车。
- `gazebo_plugins/CMakeLists.txt` / `package.xml`：注册 tricycle 动态库和 `capella_ros_msg` 依赖。
- `nav2_bringup/worlds/model.sdf`：单个前轮负责转向及驱动，两个后轮自由滚动，移除差速驱动和后脚轮。轴距 0.854 m、后轮距 0.84 m，仿真轮胎半径统一取 0.10 m；该半径是仿真设定值，并非实车测量值。保留传感器话题，将传感器移至新车体上方/前方。
- `nav2_bringup/urdf/turtlebot3_waffle.urdf`：同步关节、车体和传感器位置，供现有 `tb3_simulation_launch.py` 的 robot_state_publisher 发布 TF。插件发布 joint_states 和 odom TF，轮组 TF 交由 robot_state_publisher 发布，避免重复。

后轴中心和 `base_footprint/base_link` 的 x/y 原点位于后轴中心；前轮在 x=0.854，后轮 y=±0.42。左右滚动轴均采用车体 +Y，转向轴采用 +Z。

## 接口与保护

默认模型参数：

```xml
<command_interface>front_steer_cmd</command_interface>
<command_topic>/front_steer_cmd</command_topic>
<state_topic>/front_steer_stu</state_topic>
<command_timeout>0.5</command_timeout>
<max_linear_speed>0.5</max_linear_speed>
<max_steering_angle>1.04719755</max_steering_angle>
<max_steering_speed>0.5</max_steering_speed>
<max_steering_acceleration>1.0</max_steering_acceleration>
```

`linear_x` 是后轴中心速度，驱动前轮目标角速度为 `v / (r*cos(delta_actual))`。采用实际转角补偿转向过程中的速度参考点差异；倒车保持转角符号不变。转向使用有限力矩的关节速度电机，以角度误差闭环，并约束转向速率和速率变化。`max_wheel_acceleration/deceleration` 的单位为轮轴 rad/s²，当前分别为 3 和 6；超时、非法指令和显式零速指令优先停车。

状态以 50 Hz 从实际关节测量发布，静止时也持续发布：

- `linear_x = (rear_left_velocity + rear_right_velocity)*rear_radius/2`，代表后轮编码器测量值，存在接触噪声时保留测量结果。
- `front_steer_angle` 为转向关节实际位置；静止时不将实际转角强制清零。
- `front_wheel_speed = steering_joint_velocity * 60/(2*pi)`，表示绕 +Z 的转向转速 rpm，**不是轮胎滚动转速**。

NaN/Inf、超过线速度或转角限制的指令会使当前指令失效，下一仿真更新停止驱动和转向电机，直到收到新有效指令。指令时间使用本地单调时钟，暂停后恢复也不重放超时指令。重置仿真会清除缓存指令。非有限反馈不发布，交由 TEB 的状态超时检查处理。

`odometry_source=1` 使用 Gazebo 的 base_footprint 实际运动，速度位于后轴参考点且转换到车体坐标系；`odometry_source=0` 使用后轮编码器积分，左转角速度为正，twist 使用车体坐标系。

TEB 必须关闭 odom 转换，防止两个发布器竞争同一状态话题：

```yaml
front_steer_drive_interface:
  mode: front_steer_cmd
  publish_twist: false
  odom_to_state:
    enabled: false
```

## 构建和启动

容器内工作区根目录是 `/workspaces/capella_coverage_server`，对应宿主机 `/home/tj2022/capella_coverage_server`。加载 ROS 环境后，在工作区根目录单线程构建：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --packages-select capella_ros_msg nav2_controller teb_local_planner gazebo_plugins nav2_bringup \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

以上同时构建 `nav2_controller` 和 TEB，以使 `publish_twist: false` 开关生效。重新启动现有 `tb3_simulation_launch.py` 即可加载修改后的默认 SDF 和 URDF。确保 Gazebo 加载的是本工作区新生成的 `libgazebo_ros_tricycle_drive.so`；只修改源码不会更新系统安装的插件。TEB 自身也应先完成编译安装，并加载对应参数文件。

## 验证

`gazebo_plugins/test/test_front_steer_tricycle.py` 是独立的无界面 Gazebo 物理回归脚本。随附 `test/worlds/front_steer_tricycle.world` 使用相同底盘几何和驱动设置，省略渲染传感器；它不依赖 TurtleBot 的 mesh 或显示服务器。

加载新插件与消息环境后，指定独立 ROS domain 和 Gazebo master 执行，避免连接正在使用的仿真：

```bash
ROS_DOMAIN_ID=173 GAZEBO_MASTER_URI=http://127.0.0.1:11359 \
  python3 src/gazebo_ros_pkgs/gazebo_plugins/test/test_front_steer_tricycle.py
```

脚本覆盖静止持续反馈、直行、左右转、较大转角的速度补偿、倒车、保留实际转角的停车、非法输入、转角限制、指令超时以及世界重置。完整含传感器的 SDF 另行通过 `gz sdf -k` 检查，URDF 通过 `check_urdf` 检查；渲染传感器图像质量不属于该无界面物理回归的覆盖范围。
