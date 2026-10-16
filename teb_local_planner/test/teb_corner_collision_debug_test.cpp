#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

TEST_F(SplitModuleFixture, blockingCostModes)
{
  const auto data = teb_local_planner::test_data::corner(42u);
  ASSERT_GE(data.samples.size(), 8u);
  for (const auto & sample : data.samples) {
    EXPECT_EQ(
      teb_local_planner::TebSplitModuleTestAccess::cornerApproachCostIsBlocking(
        *planner_, sample.cost, "lethal"),
      sample.lethal_blocks);
    EXPECT_EQ(
      teb_local_planner::TebSplitModuleTestAccess::cornerApproachCostIsBlocking(
        *planner_, sample.cost, "both"),
      sample.both_blocks);
  }
  EXPECT_GE(teb_local_planner::TebSplitModuleTestAccess::cornerApproachCheckMargin(*planner_), 0.0);
}

TEST_F(SplitModuleFixture, emptyPlanFootprintSamplesStayClear)
{
  const auto data = teb_local_planner::test_data::planWindow(15u);
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::isTransformedPlanFootprintSamplesCollisionFree(
    *planner_, plan, 0.2));
  EXPECT_GT(data.plan.size(), 8u);
}
