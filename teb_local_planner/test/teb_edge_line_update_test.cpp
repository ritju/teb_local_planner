#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

TEST_F(SplitModuleFixture, vehicleFarFromWallIsOutsideExitCorridor)
{
  for (const std::uint32_t seed : {5u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::edgeLine(seed);
    ASSERT_GE(data.wall.size(), 2u);
    const Eigen::Vector2d wall_start(data.wall.front().first, data.wall.front().second);
    const Eigen::Vector2d wall_end(data.wall.back().first, data.wall.back().second);
    geometry_msgs::msg::Pose vehicle;
    vehicle.position.x = data.far_vehicle_x;
    vehicle.position.y = data.far_vehicle_y;
    EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::isVehicleInEdgeFollowingExitCorridor(
      *planner_, data.robot, wall_start, wall_end, vehicle)) << "seed " << seed;
  }
}

TEST_F(SplitModuleFixture, zeroLengthWallDoesNotFormExitCorridor)
{
  const auto data = teb_local_planner::test_data::edgeLine(8u);
  const Eigen::Vector2d wall_point(data.wall.front().first, data.wall.front().second);
  geometry_msgs::msg::Pose vehicle = data.robot.pose;
  EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::isVehicleInEdgeFollowingExitCorridor(
    *planner_, data.robot, wall_point, wall_point, vehicle));
}
