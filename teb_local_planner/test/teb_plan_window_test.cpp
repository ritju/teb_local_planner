#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/plan_corner_geometry.hpp"

#include <tf2/utils.h>

#include <vector>

TEST_F(SplitModuleFixture, sharpCornerAndArcLength)
{
  for (const std::uint32_t seed : {1u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::planWindow(seed);
    const auto & plan = data.plan;
    ASSERT_GT(plan.size(), 8u);

    const double angle = teb_local_planner::detail::polylineVertexAngleDeg(plan, data.corner_index);
    EXPECT_NEAR(angle, data.corner_angle_deg, 0.2) << "seed " << seed;

    const int corner = teb_local_planner::detail::lockedCornerIdx(plan, 0, 120.0, 40.0);
    EXPECT_EQ(corner, data.corner_index) << "seed " << seed;
    EXPECT_NEAR(
      teb_local_planner::detail::planArcLength(plan, 0, data.corner_index),
      data.arc_to_corner_m, 1e-6);
    EXPECT_EQ(
      teb_local_planner::detail::lastIdxWithinArc(plan, 0, data.arc_query_m),
      data.arc_query_idx);
  }
}

TEST_F(SplitModuleFixture, emptyPruneAndGoalYaw)
{
  std::vector<geometry_msgs::msg::PoseStamped> empty_plan;
  EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::pruneGlobalPlan(
    *planner_, makePose(0.0, 0.0), empty_plan, 1.0, 8.0));

  const auto data = teb_local_planner::test_data::planWindow(99u);
  geometry_msgs::msg::TransformStamped identity;
  identity.transform.rotation.w = 1.0;
  const auto & goal = data.plan.back();
  const int goal_idx = static_cast<int>(data.plan.size()) - 1;
  const double yaw = teb_local_planner::TebSplitModuleTestAccess::estimateLocalGoalOrientation(
    *planner_, data.plan, goal, goal_idx, identity, 3);
  EXPECT_NEAR(yaw, tf2::getYaw(goal.pose.orientation), 1e-6);
}
