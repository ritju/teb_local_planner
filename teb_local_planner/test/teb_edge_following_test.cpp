#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/edge_mode_env.hpp"

#include <nav_msgs/msg/path.hpp>

TEST(EdgeModeEnv, normalizesCaseAndWhitespace)
{
  const auto data = teb_local_planner::test_data::edgeFollow();
  for (const auto & mode : data.modes) {
    EXPECT_EQ(
      teb_local_planner::detail::normalizeEdgeModeEnv(mode.raw.c_str()),
      mode.normalized);
  }
  EXPECT_EQ(teb_local_planner::detail::normalizeEdgeModeEnv(nullptr), "");
}

TEST_F(SplitModuleFixture, segmentGeometry)
{
  for (const std::uint32_t seed : {9u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::edgeFollow(seed);
    nav_msgs::msg::Path path;
    path.poses.push_back(data.path.front());
    Eigen::Vector2d unused_start;
    Eigen::Vector2d unused_end;
    EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::extractLineSegmentFromPath(
      *planner_, path, unused_start, unused_end));

    path.poses = data.path;
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    ASSERT_TRUE(teb_local_planner::TebSplitModuleTestAccess::extractLineSegmentFromPath(
      *planner_, path, start, end));
    EXPECT_NEAR((end - start).norm(), data.chord_m, 1e-6);

    const Eigen::Vector2d query(data.query_x, data.query_y);
    EXPECT_NEAR(
      teb_local_planner::TebSplitModuleTestAccess::pointToSegmentDistance(
        *planner_, query, start, end),
      data.query_distance_m, 1e-5);

    const Eigen::Vector2d chord = end - start;
    const Eigen::Vector2d normal(-chord.y(), chord.x());
    const Eigen::Vector2d unit = normal.normalized();
    EXPECT_NEAR(
      teb_local_planner::TebSplitModuleTestAccess::segmentToSegmentDistance(
        *planner_, start, end,
        start + unit * data.parallel_distance_m,
        end + unit * data.parallel_distance_m),
      data.parallel_distance_m, 1e-4);
    const Eigen::Vector2d mid = 0.5 * (start + end);
    EXPECT_NEAR(
      teb_local_planner::TebSplitModuleTestAccess::segmentDirectionAngleDifferenceDeg(
        *planner_, start, end, mid, mid + unit),
      90.0, 1e-3);
  }
}
