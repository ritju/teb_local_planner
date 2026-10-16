#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/costmap_segment_clip.hpp"

#include <nav_msgs/msg/path.hpp>

TEST(ObstacleClip, segmentInsideWindowIsKept)
{
  for (const std::uint32_t seed : {3u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::obstacleClip(seed);
    for (const auto & segment : data.segments) {
      if (!segment.kept) {
        continue;
      }
      double x0 = segment.x0;
      double y0 = segment.y0;
      double x1 = segment.x1;
      double y1 = segment.y1;
      EXPECT_TRUE(teb_local_planner::detail::clipWorldSegmentToAlignedRect(
        x0, y0, x1, y1, segment.xmin, segment.ymin, segment.xmax, segment.ymax));
      EXPECT_NEAR(x0, segment.cx0, 1e-5);
      EXPECT_NEAR(y0, segment.cy0, 1e-5);
      EXPECT_NEAR(x1, segment.cx1, 1e-5);
      EXPECT_NEAR(y1, segment.cy1, 1e-5);
    }
  }
}

TEST(ObstacleClip, segmentOutsideWindowIsRejected)
{
  for (const std::uint32_t seed : {3u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::obstacleClip(seed);
    for (const auto & segment : data.segments) {
      if (segment.kept) {
        continue;
      }
      double x0 = segment.x0;
      double y0 = segment.y0;
      double x1 = segment.x1;
      double y1 = segment.y1;
      EXPECT_FALSE(teb_local_planner::detail::clipWorldSegmentToAlignedRect(
        x0, y0, x1, y1, segment.xmin, segment.ymin, segment.xmax, segment.ymax));
    }
  }
  const int code = teb_local_planner::detail::computeCohenSutherlandOutCode(
    -1.0, 2.0, 0.0, 0.0, 1.0, 1.0);
  EXPECT_NE(code, 0);
}

TEST_F(SplitModuleFixture, customViaPointIgnoredWhenPlanSepPositive)
{
  const auto follow = teb_local_planner::test_data::edgeFollow(42u);
  teb_local_planner::TebSplitModuleTestAccess::setViaPointSeparation(*planner_, 0.5);
  nav_msgs::msg::Path path;
  path.poses = follow.path;
  teb_local_planner::TebSplitModuleTestAccess::customViaPointsCB(
    *planner_, std::make_shared<nav_msgs::msg::Path>(path));
  EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::customViaPointsActive(*planner_));
}
