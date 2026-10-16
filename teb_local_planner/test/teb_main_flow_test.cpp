#include "main_flow_checks.hpp"

#include "teb_local_planner/robot_footprint_model.h"

TEST_F(SplitModuleFixture, usesFollowPathParamsFromYaml)
{
  const auto & config = teb_local_planner::test_data::followPathConfig();
  EXPECT_NEAR(config.obstacles.min_obstacle_dist, 0.15, 1e-6);
  EXPECT_NEAR(config.optim.weight_obstacle, 1000.0, 1e-6);
  EXPECT_EQ(config.trajectory.max_samples, 100);
  EXPECT_NEAR(config.robot.max_vel_x, 0.5, 1e-6);
  EXPECT_NEAR(config.robot.max_vel_theta, 0.4, 1e-6);
  EXPECT_NEAR(config.rotation.rotate_min_angular_vel, 0.2, 1e-6);
  EXPECT_TRUE(
    std::dynamic_pointer_cast<teb_local_planner::PolygonRobotFootprint>(config.robot_model));
}

TEST_F(SplitModuleFixture, walksEachModuleThenMainFlow)
{
  for (const std::uint32_t seed : {1u, 42u, 20261015u}) {
    const auto items = teb_local_planner::test_data::runMainFlow(*planner_, seed);
    for (const auto & flow_item : items) {
      EXPECT_TRUE(flow_item.passed) << flow_item.name << " seed " << seed << " " << flow_item.detail;
      EXPECT_GE(flow_item.scene.footprint_gap, 0.0)
        << flow_item.name << " seed " << seed << " " << flow_item.detail;
    }
  }
}
