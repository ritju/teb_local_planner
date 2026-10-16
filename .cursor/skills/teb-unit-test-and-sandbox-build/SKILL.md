---
name: teb-unit-test-and-sandbox-build
description: >-
  为 teb_local_planner 的每个单元测试构造测试数据，跑单元测试并打开 RViz
  可视化。沙箱编译使用 -j2。在本仓库编写或运行单元测试、构造测试数据、打开
  rviz，或在沙箱里编译时使用。
---

1、为每个单元测试构造测试数据，并跑单元测试和开rviz可视化界面
2、沙箱编译时，设置参数j2

测试数据放在 `teb_local_planner/test/split_module_scenarios.hpp`，单元测试和 `split_modules_viz_node` 共用。路径停车障碍沿中心线随时间移动，RViz 用同一条轨迹播放。

编译：

```bash
source /opt/ros/humble/setup.bash
cmake --build /workspaces/capella_coverage_server/build/teb_local_planner -j2
cmake --install /workspaces/capella_coverage_server/build/teb_local_planner
```

测试二进制在 `build/teb_local_planner` 下，参数用 `--gtest_print_time=0`。可视化：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/capella_coverage_server/install/setup.bash
ros2 launch teb_local_planner teb_main_flow_viz.launch.py
```

拆分模块总览仍可用 `ros2 launch teb_local_planner split_modules_viz.launch.py`。
