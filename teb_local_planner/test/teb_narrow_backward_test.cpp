#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/plan_corner_geometry.hpp"

#include <garage_utils_msgs/msg/polygons.hpp>
#include <std_msgs/msg/bool.hpp>

TEST_F(SplitModuleFixture, pointInNarrowPolygon)
{
  for (const std::uint32_t seed : {4u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::narrowPassage(seed);
    garage_utils_msgs::msg::Polygons polygons;
    polygons.polygons.push_back(data.polygon);
    teb_local_planner::TebSplitModuleTestAccess::narrowPassagesCallback(
      *planner_, std::make_shared<garage_utils_msgs::msg::Polygons>(polygons));

    EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::narrowPolygonsAvailable(*planner_));
    EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::isPointInNarrowPassage(
      *planner_, data.inside_x, data.inside_y)) << "seed " << seed;
    EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::isPointInNarrowPassage(
      *planner_, data.outside_x, data.outside_y)) << "seed " << seed;
  }
}

TEST_F(SplitModuleFixture, reverseSegmentAndBackwardPublication)
{
  for (const std::uint32_t seed : {4u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::narrowPassage(seed);
    EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::hasReverseSegmentInPlan(
      *planner_, data.forward_plan));
    EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::hasReverseSegmentInPlan(
      *planner_, data.reverse_plan)) << "seed " << seed;
    EXPECT_EQ(
      teb_local_planner::detail::lastIdxWithinArc(data.reverse_plan, 0, data.arc_query_m),
      data.arc_query_idx);
  }

  std_msgs::msg::Bool enable;
  enable.data = true;
  teb_local_planner::TebSplitModuleTestAccess::enableBackwardCallback(
    *planner_, std::make_shared<std_msgs::msg::Bool>(enable));
}
