#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/path_dynamic_wait.h"
#include "teb_params_loader.hpp"

TEST(PathWaitGeometry, samplesCenterlineAlongPlan)
{
  for (const std::uint32_t seed : {6u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::pathHold(seed);
    const auto centerline = teb_local_planner::samplePathWaitCenterline(
      data.plan, Eigen::Vector2d(data.robot_x, data.robot_y), data.lookahead_m);
    ASSERT_GE(centerline.size(), 2u);
    double arc = 0.0;
    for (std::size_t i = 1; i < centerline.size(); ++i) {
      arc += (centerline[i] - centerline[i - 1]).norm();
    }
    EXPECT_LE(arc, data.lookahead_m + 1e-3) << "seed " << seed;
    EXPECT_GT(arc, 0.5);
  }
}

TEST_F(SplitModuleFixture, holdReturnsFalseWithoutLatchOrMessage)
{
  std::vector<teb_local_planner::PathWaitObstacle> obstacles;
  bool message_valid = true;
  EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::collectPathWaitObstacles(
    *planner_, obstacles, message_valid));
  EXPECT_FALSE(message_valid);

  geometry_msgs::msg::TwistStamped command;
  geometry_msgs::msg::Twist velocity;
  EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::tryGoalStandoffHold(
    *planner_, makePose(0.0, 0.0), velocity, command));
}

TEST(PathWaitGeometry, movingObstacleEntersThenLeavesCorridor)
{
  for (const std::uint32_t seed : {6u, 42u, 20261015u}) {
    const auto data = teb_local_planner::test_data::pathHold(seed);
    teb_local_planner::TebConfig & config = teb_local_planner::test_data::followPathConfig();
    teb_local_planner::PathDynamicWaitGate gate;
    rclcpp::Clock clock(RCL_ROS_TIME);
    const auto now = clock.now();

    teb_local_planner::PathWaitObstacle aside;
    aside.id = data.obstacle_id;
    aside.radius = data.obstacle_radius;
    aside.center = Eigen::Vector2d(data.aside_x, data.aside_y);
    const auto clear = gate.evaluate(
      data.plan, Eigen::Vector2d(data.robot_x, data.robot_y), {aside}, 0.2, 0.2, config, now, true);
    EXPECT_TRUE(clear.blocking_ids.empty()) << "seed " << seed;

    teb_local_planner::PathWaitObstacle on_path;
    on_path.id = data.obstacle_id;
    on_path.radius = data.obstacle_radius;
    on_path.center = Eigen::Vector2d(data.on_path_x, data.on_path_y);
    const auto blocked = gate.evaluate(
      data.plan, Eigen::Vector2d(data.robot_x, data.robot_y), {on_path}, 0.2, 0.2, config,
      now + rclcpp::Duration::from_seconds(2.0), true);

    const auto moving = teb_local_planner::test_data::movingObstacleXY(data, 1.5);
    teb_local_planner::PathWaitObstacle traveler;
    traveler.id = data.obstacle_id + 1;
    traveler.radius = data.obstacle_radius;
    traveler.center = Eigen::Vector2d(moving.first, moving.second);
    const auto traveled = gate.evaluate(
      data.plan, Eigen::Vector2d(data.robot_x, data.robot_y), {traveler}, 0.2, 0.2, config,
      now + rclcpp::Duration::from_seconds(3.5), true);
    if (config.obstacles.path_wait_enable) {
      EXPECT_FALSE(blocked.blocking_ids.empty()) << "seed " << seed;
      EXPECT_FALSE(traveled.blocking_ids.empty()) << "seed " << seed;
    } else {
      EXPECT_TRUE(blocked.blocking_ids.empty()) << "seed " << seed;
      EXPECT_TRUE(traveled.blocking_ids.empty()) << "seed " << seed;
      EXPECT_EQ(blocked.reason, "disabled") << "seed " << seed;
    }
  }
}
