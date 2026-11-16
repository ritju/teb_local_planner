# controller_server 调试日志开关

在 controller_server.ros__parameters 下设置 debug_mode（bool，默认 false），与 controller_frequency 同级，不放在 FollowPath 下。

```yaml
controller_server:
  ros__parameters:
    debug_mode: false
```

运行时切换：

```bash
ros2 param set /controller_server debug_mode true
ros2 param set /controller_server debug_mode false
```

true 时按原触发条件输出 `[PathWait]`、`原地转:`、`[NearHorizon]`、`[TEB cost dbg]` 日志；false 时关闭这四类日志。导航运行期间可以切换，无需停止 FollowPath。

开关仅控制日志，不更改 logger 等级，不修改 INFO / DEBUG 宏及节流周期，不改变规划、碰撞检查、停车或 Marker 发布。已有 DEBUG 级日志仍受 logger 原等级过滤；普通 INFO 诊断保持 INFO 级。

`[TEB cost dbg]` 还遵循 FollowPath.debug_wall_obstacle_cost_interval（当前示例为 2.0 秒，<=0 关闭成本日志）及既有最优候选输出规则。关闭 debug_mode 时跳过仅为该日志进行的成本统计。

controller_server 声明根级参数；各 TEB 配置加载同一个参数并通过参数回调更新独立原子开关，兼容多控制器和优化工作线程。

当前运行的旧节点需要重启导航以加载此功能；加载之后即可用上述命令动态切换。

## 验证

- controller_server 动态参数测试 2 项通过，包含持有控制循环参数锁时切换 debug_mode；专用超时零速并返回 SUCCEEDED 的 action 测试 1 项通过。
- TEB 参数测试 3 项通过，包含根级启动参数加载、多个配置同步动态切换及 logger 等级保持。
- 终点占用停车测试 21 项、原地转测试 2 项通过。
- 源码核对确认原日志宏和级别不变，五个日志模块除输出条件外其他语句保持一致；两份 YAML 仅新增根级 debug_mode。
