# TEB 控制器源文件拆分方案

> 分支：`lane_edge`  
> 日期：2026-10-15  
> 目的：把 `teb_local_planner_ros.cpp`（原约 9577 行）按功能拆开，主文件只留控制主流程，便于后续查找和修改。类接口、参数、话题和算法行为不变。

---

## 1. 拆分原则

- 类仍是 `teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h` 里的 `TebLocalPlannerROS`。不拆类，不改对外接口。
- 成员函数定义按已有 `front_scan_protrusion.cpp`、`vehicle_scan_grid.cpp` 的方式放到新 `.cpp`，每个文件包含 `teb_local_planner_ros.h`。
- 只搬代码，不改算法。
- 匿名命名空间函数具有内部链接，不能跨 `.cpp` 调用。被两个以上源文件使用的工具放到 `include/teb_local_planner/detail/`，命名空间为 `teb_local_planner::detail`。
- 只被一个源文件使用的工具函数留在该文件的匿名命名空间。
- 插件导出类名不变：`PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)`，仍只在主文件末尾。

`computeVelocityCommands` 里对贴边、窄通道、原地转、等待的调用保持原样，只是被调函数的定义换了文件。

## 2. 主文件保留内容

[`teb_local_planner/src/teb_local_planner_ros.cpp`](teb_local_planner/src/teb_local_planner_ros.cpp)（约 1925 行）只保留插件入口和每一拍的控制顺序：

- 构造、析构、`initialize`、`configure`、`activate`、`deactivate`、`cleanup`
- `setPlan`
- `computeVelocityCommands`（整函数保留）
- 速度饱和与限速：`saturateVelocity`、`convertTransRotVelToSteeringAngle`、`validateFootprints`、`setSpeedLimit`、`speedLimitCallback`、`configureBackupModes`
- 文件末尾 `PLUGINLIB_EXPORT_CLASS`

## 3. 按功能拆出的源文件

| 源文件 | 约行数 | 职责 |
| --- | ---: | --- |
| `src/teb_plan_window.cpp` | 1120 | 全局路径裁剪、局部路径窗、终点退让 |
| `src/teb_obstacle_update.cpp` | 907 | 代价图 / converter / 自定义障碍、经由点、墙多边形裁剪 |
| `src/teb_edge_line_update.cpp` | 705 | 墙线、路沿更新，贴边退出走廊 |
| `src/teb_edge_following.cpp` | 1733 | 参考线配对、线段选择、融合、贴边调度 |
| `src/teb_narrow_backward.cpp` | 692 | 窄通道、倒车段、`/backward_mode`、参数模式切换 |
| `src/teb_costmap_params.cpp` | 554 | 静态层与足迹服务、正常足迹恢复、后台线程 |
| `src/teb_path_hold.cpp` | 364 | 路径等待、终点退让停车、终点占用近距停车 |
| `src/teb_rotation.cpp` | 749 | 原地转、优弧锁、转向碰撞预检 |
| `src/teb_corner_collision_debug.cpp` | 712 | 角点射线检测、碰撞足迹调试、路径足迹采样 |

`vehiclePosesCallback` 并入已有 [`src/vehicle_scan_grid.cpp`](teb_local_planner/src/vehicle_scan_grid.cpp)，不新开文件。

此前已经独立的文件保持不动：`vehicle_scan_grid.cpp`、`front_scan_protrusion.cpp`、`wall_protrusion_check.cpp`、`path_dynamic_wait.cpp`。`teb_path_hold.cpp` 只放 ROS 侧等待与停车，动态障碍等待算法仍在 `path_dynamic_wait.cpp`。

### 3.1 路径窗 `teb_plan_window.cpp`

- `pruneArrivedLockedCorner`、`pruneGlobalPlan`
- `transformGlobalPlan`、`estimateLocalGoalOrientation`
- `lateralPaddedFootprintSpec`、`globalPlanPoseLateralPaddedFootprintFreeInControllerFrame`
- `orientedFootprintNavigableOnPartialCostmap`、`selectGoalStandoffAlongPlan`
- 匿名命名空间：足迹与代价图矩形分离、裁剪后轮廓碰障判定

### 3.2 障碍物 `teb_obstacle_update.cpp`

- `updateObstacleContainerWithCostmap`
- `updateObstacleContainerWithCostmapConverter`
- `updateObstacleContainerWithCustomObstacles`
- `updateViaPointsContainer`
- `customObstacleCB`、`customViaPointsCB`
- 匿名命名空间：墙多边形裁剪（`clipPolygonDeepRobotSide`、`processWallFilteredPolygon`、`pushObstacleFromPolygonPoints`）

### 3.3 墙线与路沿 `teb_edge_line_update.cpp`

- `isVehicleInEdgeFollowingExitCorridor`
- `updateWallLineVec`、`updateCurbLineVec`、`curb_line_callback`

### 3.4 贴边调度 `teb_edge_following.cpp`

- 参考线配对：`pairedMissionAndReferencePathCallback`、`removedPlanCallback`、`clearActiveReferencePair`、`refreshActiveReferencePair` 及 snapshot
- 线段几何：`extractLineSegmentFromPath`、`pointToSegmentDistance`、`segmentToSegmentDistance`、`segmentDirectionAngleDifferenceDeg`
- 进入与选择：`shouldRunEdgeFollowingForTransformedPlan`、`edgeFollowingEntryGuards`、`trySelectWallSegmentFromCandidates`、`trySelectSegmentFromTwoPointPath`、`trySelectBestReferencePathFromList`
- 权重与融合：`computeWallLineDistWeightFromRobotDistance`、`updateScheduledWallDistWeights`、`applyWallLineSegmentAndVisual`、`mergeFusionPrimaryWithReference`
- 保持与退出：`exitEdgeFollowingMode`、`cacheReferenceLineHold`、`pruneExpiredCurbLinePath`、`applyReferenceLineHoldFallbackOrExit`、`updateReferenceLineVec`、`runEdgeFollowingPathUpdate`
- 匿名命名空间：折线重叠、`segmentsCloseForFusion`

### 3.5 窄通道与倒车 `teb_narrow_backward.cpp`

- 窄通道：`narrowPassagesCallback`、`publishNarrowPassagesMarkers`、`narrowPolygonsAvailable`、`isPointInNarrowPassage`、`isFootprintFullyOutsideNarrowPassages`、`updateNarrowPassageLatch`
- 倒车：`hasReverseSegmentAhead`、`hasReverseSegmentInPlan`、`publishBackwardMode`、`updateBackwardModePublication`、`enterDualArcReverseLatch`、`exitDualArcReverseLatch`、`resetDualArcReverseLatch`
- 参数：`applyNarrowPassageTebSettings`、`restoreNarrowPassageTebSettings`、`updateNarrowPassageTebSettings`、`enableBackwardCallback`、`switchParameterMode`

### 3.6 代价图参数 `teb_costmap_params.cpp`

- `setStaticLayerEnabled`、`setLocalFootprintEnabled`、`setGlobalFootprintEnabled`
- `armPendingNormalFootprintRestore`、`clearPendingNormalFootprintRestore`、`isNormalFootprintFreeAtRobot`、`updatePendingNormalFootprintRestore`
- 后台线程：`isBackgroundWorkAllowed`、`spawnBackgroundTask`、`joinBackgroundTasks`、`stopCostmapConverterWorker`、`waitForServiceInterruptible`

### 3.7 路径停车 `teb_path_hold.cpp`

- `pathWaitObstacleCB`、`collectPathWaitObstacles`
- `tryPathWaitHold`、`tryGoalStandoffHold`、`tryOccupiedGoalNearHold`

### 3.8 原地转 `teb_rotation.cpp`

- `shouldRotateInPlace`
- 优弧：`resetMajorArcLock`、`captureMajorArcProgress`、`remainingYawAlongMajorArc`、`checkMajorArcStagnationUnlock`、`buildMajorArcOrientedPlan`
- 转向指令：`rotateToHeadingOmegaMagnitude`、`computeRotateToHeadingCommand`、`isRotationCollisionFree`、`isRotationCollisionFreeDecel`、`checkRotateToHeadingCollisionNominal`
- `buildInPlaceRotationPredictedPath`、`publishInPlaceRotationLocalPlan`

### 3.9 角点与碰撞调试 `teb_corner_collision_debug.cpp`

- 角点：`cornerApproachCostIsBlocking`、`cornerApproachCheckMargin`、`cornerApproachFootprintSpec`、`checkCornerFootprintPoseBlocked`、`checkCornerApproachRayBlocked`、`buildCornerApproachCheckPoses`、`publishCornerApproachFootprintMarkers`、`clearCornerApproachFootprintMarkers`
- 碰撞调试：`collisionDebugFrameId`、`publishCollisionFootprintDebug`、`clearCollisionFootprintDebug`、`orientedFootprintAnyVertexOutsideLocalCostmap`、`isTransformedPlanFootprintSamplesCollisionFree`
- 匿名命名空间：`pathLengthToIndex`、`interpolatePoseAlongPlanToIndex`

## 4. 共享内部头文件

| 头文件 | 内容 | 调用方 |
| --- | --- | --- |
| `include/teb_local_planner/detail/plan_corner_geometry.hpp` | `polylineVertexAngleDeg`、`findFirstSharpCornerIdx`、`planArcLength`、`findEuclideanClosestPlanIdx`、`clampClosestIdxBeforeUnprunedCorner`、`lockedCornerIdx`、`lastIdxWithinArc`、`findCurrentRowStartIdx`、`yawAbsDiff` | 主循环、路径裁剪、`transformGlobalPlan`、倒车段前视 |
| `include/teb_local_planner/detail/async_wait.hpp` | `joinThreadQuietly`、`detachThreadQuietly`、`waitForFutureInterruptible`、`kBackgroundPollSlice` | `initialize`、足迹/静态层服务、后台线程收尾 |
| `include/teb_local_planner/detail/costmap_segment_clip.hpp` | `computeCohenSutherlandOutCode`、`clipWorldSegmentToAlignedRect` | 墙多边形裁剪、足迹跨界检测 |
| `include/teb_local_planner/detail/edge_mode_env.hpp` | `useCurbOrWallEnv()`、`normalizeEdgeModeEnv` | `initialize`、`runEdgeFollowingPathUpdate` |

原来的文件作用域变量 `use_curb_or_wall` 收成 `detail::useCurbOrWallEnv()`，仍只读一次 `USE_CURB_OR_WALL`。

## 5. 编译

新源文件已加入 [`teb_local_planner/CMakeLists.txt`](teb_local_planner/CMakeLists.txt) 的 `add_library(teb_local_planner ...)`。

2026-10-15 用 `-j2` 编译通过：

```bash
source /opt/ros/humble/setup.bash
cmake --build /workspaces/capella_coverage_server/build/teb_local_planner -j2
```

`libteb_local_planner.so` 与 `path_wait_test_node` 均链接成功。
