#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/path_dynamic_wait.h"
#include "teb_local_planner/occupied_goal_braking.hpp"
#include "teb_params_loader.hpp"
#include <nav2_core/exceptions.hpp>

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
    const auto projection = teb_local_planner::projectPathWaitPose(
      data.plan, Eigen::Vector2d(data.robot_x, data.robot_y));
    EXPECT_LE(arc, data.lookahead_m +
      (projection.point - Eigen::Vector2d(data.robot_x, data.robot_y)).norm() + 1e-3) << "seed " << seed;
    EXPECT_GT(arc, 0.5);
  }
}

TEST(PathWaitGeometry, sparsePathStartsAtProjectionAndChecksConnector)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  const auto projection = teb_local_planner::projectPathWaitPose(path, Eigen::Vector2d(6.0, 0.0));
  ASSERT_EQ(projection.segment, 0);
  EXPECT_NEAR(projection.point.x(), 6.0, 1e-9);
  auto line = teb_local_planner::samplePathWaitCenterline(path, Eigen::Vector2d(6.0, 0.0), 3.0);
  ASSERT_GE(line.size(), 2u);
  EXPECT_NEAR(line.front().x(), 6.0, 1e-9);
  EXPECT_NEAR(line.back().x(), 9.0, 1e-9);
  line = teb_local_planner::samplePathWaitCenterline(path, Eigen::Vector2d(6.0, 0.5), 3.0);
  ASSERT_GE(line.size(), 3u);
  EXPECT_NEAR(line.front().y(), 0.5, 1e-9);
  EXPECT_NEAR(line[1].y(), 0.0, 1e-9);
}

TEST(PathWaitGeometry, footprintDetectsLineObstacleBeforeItsCenter)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 7;
  obstacle.polygon.push_back(Eigen::Vector2d(7.0, -0.3));
  obstacle.polygon.push_back(Eigen::Vector2d(7.0, 0.3));
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.3; footprint[0].y = 0.2;
  footprint[1].x = 0.3; footprint[1].y = -0.2;
  footprint[2].x = -0.3; footprint[2].y = -0.2;
  footprint[3].x = -0.3; footprint[3].y = 0.2;
  auto& config = teb_local_planner::test_data::followPathConfig();
  config.obstacles.path_wait_enable = true;
  teb_local_planner::PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto result = gate.evaluate(path, Eigen::Vector2d(6.0, 0.0), {obstacle},
    0.2, 0.1, config, clock.now(), true, 0.0, footprint);
  EXPECT_EQ(result.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_TRUE(result.has_collision);
  EXPECT_EQ(result.collision_id, 7);
  EXPECT_GT(result.collision_arc, 0.5);
  EXPECT_LT(result.collision_arc, 0.8);
  visualization_msgs::msg::MarkerArray markers;
  teb_local_planner::buildPathWaitDebugMarkers(markers, "map", clock.now(), result);
  auto has_marker = [&markers](const std::string& name) {
    for (const auto& marker : markers.markers) {
      if (marker.ns == name) return true;
    }
    return false;
  };
  EXPECT_TRUE(has_marker("path_projection"));
  EXPECT_TRUE(has_marker("path_connector"));
  EXPECT_TRUE(has_marker("path_first_collision"));
}

TEST(PathWaitGeometry, reverseHeadingUsesActualRearFootprint)
{
  const std::vector<geometry_msgs::msg::PoseStamped> forward = {
    makePose(0.0, 0.0, 0.0), makePose(10.0, 0.0, 0.0)};
  const std::vector<geometry_msgs::msg::PoseStamped> reverse = {
    makePose(0.0, 0.0, M_PI), makePose(10.0, 0.0, M_PI)};
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.5; footprint[0].y = 0.2;
  footprint[1].x = 0.5; footprint[1].y = -0.2;
  footprint[2].x = -0.1; footprint[2].y = -0.2;
  footprint[3].x = -0.1; footprint[3].y = 0.2;
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 8;
  obstacle.center = Eigen::Vector2d(7.0, 0.0);
  obstacle.radius = 0.01;
  auto& config = teb_local_planner::test_data::followPathConfig();
  config.obstacles.path_wait_enable = true;
  rclcpp::Clock clock(RCL_ROS_TIME);
  teb_local_planner::PathDynamicWaitGate forward_gate, reverse_gate;
  const auto forward_hit = forward_gate.evaluate(forward, Eigen::Vector2d(6.0, 0.0),
    {obstacle}, 0.2, 0.1, config, clock.now(), true, 0.0, footprint);
  const auto reverse_hit = reverse_gate.evaluate(reverse, Eigen::Vector2d(6.0, 0.0),
    {obstacle}, 0.2, 0.1, config, clock.now(), true, M_PI, footprint);
  ASSERT_TRUE(forward_hit.has_collision);
  ASSERT_TRUE(reverse_hit.has_collision);
  EXPECT_GT(reverse_hit.collision_arc, forward_hit.collision_arc + 0.3);
}

TEST(PathWaitGeometry, movingObstacleEnvelopeDoesNotCoverAllDirections)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 1.16; footprint[0].y = 0.66;
  footprint[1].x = 1.16; footprint[1].y = -0.66;
  footprint[2].x = -0.69; footprint[2].y = -0.66;
  footprint[3].x = -0.69; footprint[3].y = 0.66;
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 0;
  obstacle.center = Eigen::Vector2d(3.0, -1.7);
  obstacle.velocity = Eigen::Vector2d(0.5, 0.3);
  for (const Eigen::Vector2d& corner : {
      Eigen::Vector2d(2.5, -2.2), Eigen::Vector2d(3.5, -2.2),
      Eigen::Vector2d(3.5, -1.2), Eigen::Vector2d(2.5, -1.2)})
    obstacle.polygon.push_back(corner);
  teb_local_planner::TebConfig config;
  config.obstacles.path_wait_enable = true;
  config.obstacles.path_wait_lookahead_dist = 6.0;
  config.obstacles.path_wait_brake_response_time = 1.0;
  config.robot.max_vel_x = 0.5;
  config.robot.acc_lim_x = 0.3;
  teb_local_planner::PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto now = clock.now();
  const auto wait = gate.evaluate(path, Eigen::Vector2d(0.2, 0.0), {obstacle},
    0.66, 0.075, config, now, true, 0.0, footprint);
  ASSERT_TRUE(wait.has_collision);
  EXPECT_GT(wait.collision_arc, 0.2);
  EXPECT_EQ(wait.mode, teb_local_planner::PathWaitMode::Wait);
  ASSERT_EQ(wait.predicted_blocking_obstacles.size(), 1u);
  visualization_msgs::msg::MarkerArray markers;
  teb_local_planner::buildPathWaitDebugMarkers(markers, "map", now, wait);
  bool has_envelope = false;
  bool has_timer = false;
  for (const auto& marker : markers.markers)
  {
    has_envelope |= marker.ns == "blocking_motion_envelope";
    has_timer |= marker.ns == "wait_timer" && marker.text.find("WAIT ID") != std::string::npos;
  }
  EXPECT_TRUE(has_envelope);
  EXPECT_TRUE(has_timer);

  const auto detour = gate.evaluate(path, Eigen::Vector2d(0.2, 0.0), {obstacle},
    0.66, 0.075, config, now + rclcpp::Duration::from_seconds(7.0),
    true, 0.0, footprint);
  EXPECT_EQ(detour.mode, teb_local_planner::PathWaitMode::Detour);
}

TEST(PathWaitGate, sameIdRearmsAfterCurrentOutlineLeavesWhilePredictionStillBlocks)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.3; footprint[0].y = 0.2;
  footprint[1].x = 0.3; footprint[1].y = -0.2;
  footprint[2].x = -0.3; footprint[2].y = -0.2;
  footprint[3].x = -0.3; footprint[3].y = 0.2;
  teb_local_planner::TebConfig config;
  config.obstacles.path_wait_enable = true;
  config.obstacles.path_wait_clear_time = 0.5;
  config.obstacles.path_wait_timeout = 6.0;
  config.obstacles.path_wait_brake_response_time = 1.0;
  config.obstacles.path_wait_lookahead_dist = 6.0;
  config.robot.max_vel_x = 0.5;
  config.robot.acc_lim_x = 0.3;
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 0;
  obstacle.center = Eigen::Vector2d(3.0, 0.0);
  obstacle.radius = 0.2;
  teb_local_planner::PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto now = clock.now();
  auto evaluate = [&](double seconds) {
    return gate.evaluate(path, Eigen::Vector2d(0.0, 0.0), {obstacle},
      0.2, 0.1, config, now + rclcpp::Duration::from_seconds(seconds),
      true, 0.0, footprint);
  };
  const auto first = evaluate(0.0);
  EXPECT_EQ(first.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_EQ(first.observed_ids, std::vector<int>({0}));
  obstacle.center.y() = 2.0;
  obstacle.velocity.y() = -1.0;
  EXPECT_EQ(evaluate(2.0).mode, teb_local_planner::PathWaitMode::Wait);
  obstacle.center.y() = 0.0;
  obstacle.velocity.y() = 0.0;
  const auto short_return = evaluate(2.3);
  EXPECT_EQ(short_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_NEAR(short_return.elapsed_wait_s, 2.3, 1e-6);
  const auto timed_out = evaluate(7.0);
  EXPECT_EQ(timed_out.mode, teb_local_planner::PathWaitMode::Detour);

  obstacle.center.y() = 2.0;
  obstacle.velocity.y() = -1.0;
  const auto leaving = evaluate(7.1);
  EXPECT_EQ(leaving.mode, teb_local_planner::PathWaitMode::Detour);
  EXPECT_EQ(leaving.blocking_ids, std::vector<int>({0}));
  EXPECT_TRUE(leaving.observed_ids.empty());
  const auto outside = evaluate(7.7);
  EXPECT_EQ(outside.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_EQ(outside.waiting_ids, std::vector<int>({0}));
  EXPECT_EQ(outside.latched_ids, std::vector<int>({0}));
  EXPECT_DOUBLE_EQ(outside.elapsed_wait_s, 0.0);
  visualization_msgs::msg::MarkerArray outside_markers;
  teb_local_planner::buildPathWaitDebugMarkers(outside_markers, "map", now, outside);
  bool shows_pending = false;
  for (const auto& marker : outside_markers.markers)
    shows_pending |= marker.ns == "wait_timer" &&
      marker.text.find("RETURN PENDING") != std::string::npos;
  EXPECT_TRUE(shows_pending);

  obstacle.center.y() = 0.0;
  obstacle.velocity.y() = 0.0;
  const auto returned = evaluate(7.8);
  EXPECT_EQ(returned.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_EQ(returned.observed_ids, std::vector<int>({0}));
  EXPECT_DOUBLE_EQ(returned.elapsed_wait_s, 0.0);
  EXPECT_EQ(evaluate(13.7).mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_EQ(evaluate(13.9).mode, teb_local_planner::PathWaitMode::Detour);

  // 回归现场日志：全局走廊清空进入 FOLLOW 后，同一 ID 的预测扫掠先于当前轮廓返回。
  obstacle.center.y() = 2.0;
  obstacle.velocity.y() = -1.0;
  EXPECT_EQ(evaluate(14.0).mode, teb_local_planner::PathWaitMode::Detour);
  EXPECT_EQ(evaluate(14.6).awaiting_return_ids, std::vector<int>({0}));
  obstacle.velocity.y() = 0.0;
  EXPECT_EQ(evaluate(14.7).mode, teb_local_planner::PathWaitMode::Wait);
  const auto globally_clear = evaluate(15.3);
  EXPECT_EQ(globally_clear.mode, teb_local_planner::PathWaitMode::Follow);
  EXPECT_EQ(globally_clear.awaiting_return_ids, std::vector<int>({0}));
  obstacle.velocity.y() = -1.0;
  const auto predicted_return = evaluate(15.4);
  EXPECT_TRUE(predicted_return.observed_ids.empty());
  EXPECT_EQ(predicted_return.awaiting_return_ids, std::vector<int>({0}));
  EXPECT_EQ(predicted_return.waiting_ids, std::vector<int>({0}));
  EXPECT_EQ(predicted_return.reason, "awaiting_current_return");
  EXPECT_EQ(predicted_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_DOUBLE_EQ(predicted_return.elapsed_wait_s, 0.0);
  const auto still_waiting_return = evaluate(20.0);
  EXPECT_EQ(still_waiting_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_DOUBLE_EQ(still_waiting_return.elapsed_wait_s, 0.0);
  obstacle.center.y() = 0.0;
  obstacle.velocity.y() = 0.0;
  const auto actual_return = evaluate(20.1);
  EXPECT_EQ(actual_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_DOUBLE_EQ(actual_return.elapsed_wait_s, 0.0);
  EXPECT_TRUE(actual_return.awaiting_return_ids.empty());
  EXPECT_EQ(evaluate(26.0).mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_EQ(evaluate(26.2).mode, teb_local_planner::PathWaitMode::Detour);
}

TEST(PathWaitGate, predictedOnlyBoundaryFlickerKeepsOneWaitClock)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  teb_local_planner::TebConfig config;
  config.obstacles.path_wait_enable = true;
  config.obstacles.path_wait_clear_time = 0.5;
  config.obstacles.path_wait_timeout = 6.0;
  config.obstacles.path_wait_brake_response_time = 1.0;
  config.obstacles.path_wait_lookahead_dist = 6.0;
  config.robot.max_vel_x = 0.5;
  config.robot.acc_lim_x = 0.3;
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 0;
  obstacle.center = Eigen::Vector2d(3.0, 2.0);
  obstacle.velocity = Eigen::Vector2d(0.0, -1.0);
  obstacle.radius = 0.2;
  teb_local_planner::PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto now = clock.now();
  auto evaluate = [&](double seconds) {
    return gate.evaluate(path, Eigen::Vector2d(0.0, 0.0), {obstacle},
      0.2, 0.1, config, now + rclcpp::Duration::from_seconds(seconds), true);
  };
  EXPECT_EQ(evaluate(0.0).mode, teb_local_planner::PathWaitMode::Wait);
  obstacle.velocity.y() = 0.0;
  EXPECT_EQ(evaluate(3.0).reason, "clear_hold");
  obstacle.velocity.y() = -1.0;
  const auto flicker_return = evaluate(3.3);
  EXPECT_EQ(flicker_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_NEAR(flicker_return.elapsed_wait_s, 3.3, 1e-6);
  obstacle.velocity.y() = 0.0;
  EXPECT_EQ(evaluate(3.4).reason, "clear_hold");
  obstacle.velocity.y() = -1.0;
  const auto unsampled_return = evaluate(4.0);
  EXPECT_EQ(unsampled_return.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_NEAR(unsampled_return.elapsed_wait_s, 4.0, 1e-6);
  EXPECT_EQ(evaluate(6.0).mode, teb_local_planner::PathWaitMode::Detour);

  obstacle.velocity.y() = 0.0;
  EXPECT_EQ(evaluate(6.1).mode, teb_local_planner::PathWaitMode::Detour);
  EXPECT_EQ(evaluate(6.7).mode, teb_local_planner::PathWaitMode::Follow);
  obstacle.velocity.y() = -1.0;
  const auto new_episode = evaluate(6.8);
  EXPECT_EQ(new_episode.mode, teb_local_planner::PathWaitMode::Wait);
  EXPECT_DOUBLE_EQ(new_episode.elapsed_wait_s, 0.0);
}

TEST(PathWaitGate, publishedSquareAheadIntersectsForwardCorridor)
{
  const std::vector<geometry_msgs::msg::PoseStamped> path = {
    makePose(0.0, 0.0), makePose(10.0, 0.0)};
  teb_local_planner::TebConfig config;
  config.obstacles.path_wait_enable = true;
  config.obstacles.path_wait_lookahead_dist = 6.0;
  teb_local_planner::PathWaitObstacle obstacle;
  obstacle.id = 0;
  obstacle.center = Eigen::Vector2d(5.36, -0.28);
  obstacle.polygon = {Eigen::Vector2d(4.86, -0.78), Eigen::Vector2d(5.86, -0.78),
                      Eigen::Vector2d(5.86, 0.22), Eigen::Vector2d(4.86, 0.22)};
  teb_local_planner::PathDynamicWaitGate gate;
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto result = gate.evaluate(path, Eigen::Vector2d(3.63, -0.15), {obstacle},
    0.66, 0.075, config, clock.now(), true);
  EXPECT_EQ(result.observed_ids, std::vector<int>({0}));
  EXPECT_EQ(result.blocking_ids, std::vector<int>({0}));
  EXPECT_EQ(result.mode, teb_local_planner::PathWaitMode::Wait);
}

TEST(OccupiedGoalBraking, speedLimitAccountsForResponseDelay)
{
  const double deceleration = 0.3;
  const double response_time = 0.1;  // controller_frequency = 10 Hz
  const double speed = 0.5;
  const double required = teb_local_planner::occupiedGoalStoppingDistance(
    speed, deceleration, response_time);
  EXPECT_NEAR(required, 0.4666666667, 1e-6);
  EXPECT_NEAR(teb_local_planner::occupiedGoalSafeSpeed(
    required, deceleration, response_time), speed, 1e-6);
  EXPECT_LT(teb_local_planner::occupiedGoalSafeSpeed(
    required - 0.1, deceleration, response_time), speed);
  EXPECT_DOUBLE_EQ(teb_local_planner::occupiedGoalSafeSpeed(
    0.0, deceleration, response_time), 0.0);
}

TEST(OccupiedGoalBraking, honorsSmallConfiguredDeceleration)
{
  EXPECT_NEAR(teb_local_planner::occupiedGoalStoppingDistance(0.5, 0.02, 0.1), 6.3, 1e-9);
  EXPECT_NEAR(teb_local_planner::occupiedGoalSafeSpeed(6.3, 0.02, 0.1), 0.5, 1e-9);
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

namespace
{
class GoalTestCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  explicit GoalTestCritic(nav2_costmap_2d::Costmap2D & map) {costmap_ = &map;}
};
using GoalAccess = teb_local_planner::TebSplitModuleTestAccess;

class OccupiedGoalTest : public SplitModuleFixture
{
protected:
  void SetUp() override
  {
    SplitModuleFixture::SetUp();
    map_ = std::make_unique<nav2_costmap_2d::Costmap2D>(240, 120, 0.05, -2.0, -2.0, 0);
    GoalAccess::configureOccupiedGoal(*planner_, *map_, std::make_shared<GoalTestCritic>(*map_));
    for (int i = 0; i <= 12; ++i) {plan_.push_back(makePose(i * 0.5, 0.0));}
  }
  void blockGoal()
  {
    for (unsigned int x = 156; x <= 164; ++x) {
      for (unsigned int y = 34; y <= 46; ++y) {map_->setCost(x, y, 254);}
    }
  }
  std::unique_ptr<nav2_costmap_2d::Costmap2D> map_;
  std::vector<geometry_msgs::msg::PoseStamped> plan_;
};

TEST_F(OccupiedGoalTest, ignoresOccupiedGoalOutsideWindowThenSelectsSafeTerminal)
{
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 1.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  ASSERT_GE(GoalAccess::standoffIndex(*planner_), 0);
  EXPECT_LT(GoalAccess::standoffIndex(*planner_), 12);
  EXPECT_LT(local.back().pose.position.x, plan_.back().pose.position.x);
  EXPECT_EQ(plan_.size(), 13u);
  // Once entered, a shrinking horizon cannot release the occupied-goal episode.
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 1.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
}

TEST_F(OccupiedGoalTest, detectsObstacleEntirelyInsideGoalFootprint)
{
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(6.025, 0.025, x, y));
  map_->setCost(x, y, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_LT(local.back().pose.position.x, 6.0);
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, {makePose(6.0, 0.0)}));
  map_->setCost(x, y, 255);
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, {makePose(6.0, 0.0)}));
}

TEST_F(OccupiedGoalTest, forwardExtendedWindowDoesNotTreatGoalClippingAsOccupation)
{
  // Ordinary row fallback extends the 5m horizon to x=6. Clipping of the
  // otherwise free goal footprint must not trigger occupied-goal parking.
  map_->resizeMap(161, 120, 0.05, -2.0, -2.0);
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(4.525, 0.025, x, y));
  map_->setCost(x, y, 254);
  GoalAccess::setGoalTailDistance(*planner_, 4.0);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 5.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_DOUBLE_EQ(local.back().pose.position.x, plan_.back().pose.position.x);
}

TEST(OccupiedGoalParameters, clearDurationLoadsAndUpdatesDynamically)
{
  teb_local_planner::TebConfig config;
  teb_local_planner::test_data::loadFollowPathParams(config);
  EXPECT_DOUBLE_EQ(config.trajectory.occupied_goal_clear_duration, 0.4);
  const auto result = config.dynamicParametersCallback(
    {rclcpp::Parameter("FollowPath.occupied_goal_clear_duration", 1.25)});
  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(config.trajectory.occupied_goal_clear_duration, 1.25);
}

TEST_F(OccupiedGoalTest, partiallyVisibleOrOutsideFreeGoalDoesNotStartParking)
{
  std::vector<geometry_msgs::msg::PoseStamped> local;
  map_->resizeMap(161, 120, 0.05, -2.0, -2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  map_->resizeMap(100, 120, 0.05, -2.0, -2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, partialFootprintDetectsVisibleInteriorAcrossEveryMapEdge)
{
  map_->resizeMap(40, 40, 0.05, -1.0, -1.0);
  const std::vector<std::pair<double, double>> directions = {
    {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
  for (const auto & direction : directions) {
    for (double offset : {0.95, 1.05}) {
      for (double yaw : {0.0, M_PI / 4.0}) {
        const auto pose = makePose(direction.first * offset, direction.second * offset, yaw);
        ASSERT_TRUE(GoalAccess::partialFootprintFree(*planner_, pose));
        const double bx = direction.first == 0.0 ? 0.025 : direction.first * 0.975;
        const double by = direction.second == 0.0 ? 0.025 : direction.second * 0.975;
        unsigned int x, y;
        ASSERT_TRUE(map_->worldToMap(bx, by, x, y));
        map_->setCost(x, y, 254);
        EXPECT_FALSE(GoalAccess::partialFootprintFree(*planner_, pose));
        map_->setCost(x, y, 255);
        EXPECT_FALSE(GoalAccess::partialFootprintFree(*planner_, pose));
        map_->setCost(x, y, 0);
      }
    }
  }
}

TEST_F(OccupiedGoalTest, partiallyVisibleGoalChecksVisibleInteriorAndKeepsFrontClearance)
{
  map_->resizeMap(161, 120, 0.05, -2.0, -2.0);
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(6.025, 0.025, x, y));
  map_->setCost(x, y, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  ASSERT_TRUE(GoalAccess::occupiedActive(*planner_));
  const int index = GoalAccess::standoffIndex(*planner_);
  ASSERT_GE(index, 0);
  const auto stop = GoalAccess::standoffPose(*planner_);
  EXPECT_LE(stop.pose.position.x, 5.5);
  EXPECT_TRUE(GoalAccess::brakingPathSafe(*planner_, {stop}));
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, {plan_.back()}));
}

TEST_F(OccupiedGoalTest, skipsFreeCandidateWhoseFootprintIsNotFullyInsideMap)
{
  map_->resizeMap(800, 400, 0.01, -2.0, -2.0);
  plan_.clear();
  for (int i = 0; i <= 60; ++i) {plan_.push_back(makePose(i * 0.1, 0.0));}
  plan_.back() = makePose(6.0, 0.0, M_PI / 4.0);
  GoalAccess::disableStandoffClearanceForGeometryTest(*planner_);
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(5.975, 0.225, x, y));
  map_->setCost(x, y, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  ASSERT_TRUE(GoalAccess::occupiedActive(*planner_));
  ASSERT_GE(GoalAccess::standoffIndex(*planner_), 0);
  const auto stop = GoalAccess::standoffPose(*planner_);
  // x=5.9 has no collision but its right vertices are outside the map.
  EXPECT_NEAR(stop.pose.position.x, 5.8, 1e-6);
  EXPECT_TRUE(GoalAccess::brakingPathSafe(*planner_, {stop}));
}

TEST_F(OccupiedGoalTest, partiallyVisibleGoalCanClearWithoutFullFootprintEnteringMap)
{
  map_->resizeMap(161, 120, 0.05, -2.0, -2.0);
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(6.025, 0.025, x, y));
  map_->setCost(x, y, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  ASSERT_TRUE(GoalAccess::occupiedActive(*planner_));
  GoalAccess::checkGoalWait(*planner_, true);
  map_->setCost(x, y, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  GoalAccess::ageGoalClearBy(*planner_, 2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, {plan_.back()}));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
  EXPECT_DOUBLE_EQ(local.back().pose.position.x, plan_.back().pose.position.x);
}

TEST_F(OccupiedGoalTest, losingObservedBlockerDoesNotReleaseEvenWhileGoalIsPartlyVisible)
{
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(6.125, 0.025, x, y));
  map_->setCost(x, y, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  ASSERT_TRUE(GoalAccess::occupiedActive(*planner_));
  GoalAccess::checkGoalWait(*planner_, true);
  // The previously detected obstacle exits the map. The visible part is clear,
  // which must not be mistaken for actual disappearance of that obstacle.
  map_->resizeMap(161, 120, 0.05, -2.0, -2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::ageGoalClearBy(*planner_, 2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  // Bring only the old blocking area back into view, now free. Full observation
  // of the entire goal footprint is not needed to complete release debounce.
  map_->resizeMap(163, 120, 0.05, -2.0, -2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::ageGoalClearBy(*planner_, 2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, pathFailureStopPreservesTerminalAndHonorsExistingDeadline)
{
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  const auto locked = GoalAccess::standoffPose(*planner_);
  const auto index = GoalAccess::standoffIndex(*planner_);
  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 0.4;
  command.twist.angular.z = 0.3;
  GoalAccess::stopOccupiedGoal(*planner_, plan_.front(), command);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), index);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_DOUBLE_EQ(GoalAccess::standoffPose(*planner_).pose.position.x, locked.pose.position.x);
  GoalAccess::expireGoalWait(*planner_);
  EXPECT_THROW(GoalAccess::stopOccupiedGoal(*planner_, plan_.front(), command),
    nav2_core::OccupiedGoalTimeout);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
}

TEST_F(OccupiedGoalTest, occupiedLockedStopImmediatelyHoldsWithoutSelectingAnotherPoint)
{
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  const int index = GoalAccess::standoffIndex(*planner_);
  ASSERT_GE(index, 0);
  const auto locked = GoalAccess::standoffPose(*planner_);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.3;
  geometry_msgs::msg::TwistStamped command;
  EXPECT_FALSE(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), velocity, command));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(locked.pose.position.x + 0.025, 0.025, x, y));
  map_->setCost(x, y, 254);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), index);
  EXPECT_TRUE(GoalAccess::standoffBlocked(*planner_));
  command.twist.linear.x = 0.3;
  ASSERT_TRUE(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), velocity, command));
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  map_->setCost(x, y, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), index);
  EXPECT_FALSE(GoalAccess::standoffBlocked(*planner_));
  EXPECT_FALSE(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), velocity, command));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  GoalAccess::expireGoalWait(*planner_);
  EXPECT_THROW(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), velocity, command),
    nav2_core::OccupiedGoalTimeout);
}

TEST_F(OccupiedGoalTest, configuredClearDurationCancelsWaitAndRestoresOriginalGoal)
{
  GoalAccess::setGoalClearDuration(*planner_, 2.0);
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::checkGoalWait(*planner_, true);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::ageGoalClearBy(*planner_, 1.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  GoalAccess::ageGoalClearBy(*planner_, 2.1);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  EXPECT_DOUBLE_EQ(local.back().pose.position.x, plan_.back().pose.position.x);
  GoalAccess::expireGoalWait(*planner_);
  EXPECT_NO_THROW(GoalAccess::checkGoalWait(*planner_, false));
  blockGoal();
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, unknownOrOffMapGoalCannotCancelWait)
{
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::checkGoalWait(*planner_, true);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 0);
  map_->setCost(160, 40, 255);
  GoalAccess::ageGoalClearBy(*planner_, 2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  map_->resizeMap(100, 120, 0.05, -2.0, -2.0);
  GoalAccess::ageGoalClearBy(*planner_, 2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, noInitialSafePointDoesNotStartAnotherSearch)
{
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  ASSERT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 0);
  blockGoal();
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  geometry_msgs::msg::TwistStamped command;
  EXPECT_TRUE(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), geometry_msgs::msg::Twist{}, command));
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, sameGoalReplanMissingLockedPointStopsWithoutReplacingIt)
{
  GoalAccess::markInitialized(*planner_);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = plan_;
  planner_->setPlan(path);
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  const auto locked = GoalAccess::standoffPose(*planner_);
  const int index = GoalAccess::standoffIndex(*planner_);
  ASSERT_GE(index, 0);
  path.poses.erase(path.poses.begin() + index);
  planner_->setPlan(path);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, path.poses, 8.0, local));
  EXPECT_TRUE(GoalAccess::standoffBlocked(*planner_));
  EXPECT_DOUBLE_EQ(GoalAccess::standoffPose(*planner_).pose.position.x, locked.pose.position.x);
  geometry_msgs::msg::TwistStamped command;
  EXPECT_TRUE(GoalAccess::holdOccupiedGoal(*planner_, plan_.front(), geometry_msgs::msg::Twist{}, command));
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, zeroClearDurationReleasesImmediately)
{
  GoalAccess::setGoalClearDuration(*planner_, 0.0);
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::checkGoalWait(*planner_, true);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, noSafeTerminalAndStableClearRelease)
{
  map_->resetMap(0, 0, 240, 120);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  EXPECT_EQ(GoalAccess::standoffIndex(*planner_), -1);
  std::fill(map_->getCharMap(), map_->getCharMap() + 240 * 120, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_TRUE(GoalAccess::occupiedActive(*planner_));
  GoalAccess::ageGoalClear(*planner_);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
}

TEST_F(OccupiedGoalTest, sameGoalReplanPreservesDeadlineAndTimeoutStaysTerminal)
{
  GoalAccess::markInitialized(*planner_);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = plan_;
  planner_->setPlan(path);
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::checkGoalWait(*planner_, true);
  GoalAccess::expireGoalWait(*planner_);
  planner_->setPlan(path);
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
  EXPECT_THROW(GoalAccess::checkGoalWait(*planner_, false), nav2_core::OccupiedGoalTimeout);
  EXPECT_THROW(GoalAccess::checkGoalWait(*planner_, false), nav2_core::OccupiedGoalTimeout);
  // A new setPlan after the terminal abort permits a new attempt, even to the same goal.
  planner_->setPlan(path);
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_NO_THROW(GoalAccess::checkGoalWait(*planner_, false));
}

TEST_F(OccupiedGoalTest, differentGoalReplanResetsDeadline)
{
  GoalAccess::markInitialized(*planner_);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = plan_;
  planner_->setPlan(path);
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  GoalAccess::checkGoalWait(*planner_, true);
  GoalAccess::expireGoalWait(*planner_);
  path.poses.back().pose.position.x += 1.0;
  planner_->setPlan(path);
  EXPECT_FALSE(GoalAccess::occupiedActive(*planner_));
  EXPECT_FALSE(GoalAccess::goalWaitStarted(*planner_));
}

TEST_F(OccupiedGoalTest, brakingChecksBetweenPosesAndRejectsUnknownOrOffMap)
{
  const std::vector<geometry_msgs::msg::PoseStamped> sparse = {makePose(0.0, 0.0), makePose(1.0, 0.0)};
  EXPECT_TRUE(GoalAccess::brakingPathSafe(*planner_, sparse));
  // Obstacle intersects the padded footprint between the two published endpoints.
  map_->setCost(50, 43, 254);
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, sparse));
  map_->setCost(50, 43, 255);
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, sparse));
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, {makePose(-2.0, 0.0)}));
}

TEST_F(OccupiedGoalTest, publishesFullOptimizedPlanDuringBrakingAndStopsForNearObstacle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("occupied_goal_publication_test");
  auto listener = std::make_shared<rclcpp::Node>("occupied_goal_plan_listener");
  nav_msgs::msg::Path::SharedPtr received;
  auto sub = listener->create_subscription<nav_msgs::msg::Path>("local_plan", 1,
    [&](nav_msgs::msg::Path::SharedPtr msg) {received = msg;});
  std::vector<geometry_msgs::msg::PoseStamped> optimized;
  for (int i = 0; i <= 10; ++i) {
    auto pose = makePose(i * 0.1, 0.0);
    pose.header.stamp = node->now();
    optimized.push_back(pose);
  }
  GoalAccess::setGoalVisualization(*planner_, node, optimized);
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < discovery_deadline) {
    // Republish to survive DDS discovery; consume this seed before checking braking output.
    GoalAccess::seedLocalPlan(*planner_, optimized);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  received.reset();
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.5;
  geometry_msgs::msg::TwistStamped command;
  do {
    command.twist.linear.x = 0.9;
    GoalAccess::brakeGoal(*planner_, optimized.front(), velocity, command);
    rclcpp::spin_some(listener);
    if (received) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < until);
  ASSERT_NE(received, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.linear.x, 0.9);
  EXPECT_EQ(received->poses.size(), optimized.size());
  EXPECT_DOUBLE_EQ(received->poses.back().pose.position.x, 1.0);
  map_->setCost(46, 43, 254);
  command.twist.linear.x = 0.9;
  GoalAccess::brakeGoal(*planner_, optimized.front(), velocity, command);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  EXPECT_TRUE(GoalAccess::goalWaitStarted(*planner_));
}
TEST_F(OccupiedGoalTest, reportsCollisionUnknownOutOfMapAndMissingLockedPoint)
{
  blockGoal();
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  const auto locked = GoalAccess::standoffPose(*planner_);
  unsigned int x, y;
  ASSERT_TRUE(map_->worldToMap(locked.pose.position.x, locked.pose.position.y, x, y));
  for (const auto cost : {254, 255}) {
    map_->setCost(x, y, cost);
    ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
    EXPECT_TRUE(GoalAccess::standoffBlocked(*planner_));
    EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_),
      cost == 254 ? "footprint_collision" : "footprint_unknown");
    EXPECT_NE(GoalAccess::standoffFailureDetail(*planner_).find("blocking_cell="), std::string::npos);
    EXPECT_NE(GoalAccess::standoffFailureDetail(*planner_).find("cost=" + std::to_string(cost)),
      std::string::npos);
  }
  map_->setCost(x, y, 0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_), "valid");
  const auto width = static_cast<unsigned int>(std::round((locked.pose.position.x + 2.1) / 0.05));
  map_->resizeMap(width, 120, 0.05, -2.0, -2.0);
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_), "footprint_outside_costmap");
  plan_.erase(plan_.begin() + GoalAccess::standoffIndex(*planner_));
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_), "locked_point_missing");
}

TEST_F(OccupiedGoalTest, reportsFrontAndBackClearanceFailureDespiteFreeLockedFootprint)
{
  plan_.clear();
  for (int i = 0; i <= 30; ++i) {plan_.push_back(makePose(i * 0.2, 0.0));}
  unsigned int gx, gy;
  ASSERT_TRUE(map_->worldToMap(6.025, 0.025, gx, gy));
  map_->setCost(gx, gy, 254);
  std::vector<geometry_msgs::msg::PoseStamped> local;
  ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
  const auto locked = GoalAccess::standoffPose(*planner_);
  for (const double direction : {1.0, -1.0}) {
    unsigned int x, y;
    ASSERT_TRUE(map_->worldToMap(locked.pose.position.x + direction * 0.55, 0.025, x, y));
    map_->setCost(x, y, 254);
    ASSERT_TRUE(GoalAccess::brakingPathSafe(*planner_, {locked}));
    ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
    EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_), direction > 0.0 ?
      "front_clearance_insufficient" : "back_clearance_insufficient");
    EXPECT_NE(GoalAccess::standoffFailureDetail(*planner_).find("front_clear="), std::string::npos);
    EXPECT_NE(GoalAccess::standoffFailureDetail(*planner_).find("back_clear="), std::string::npos);
    map_->setCost(x, y, 0);
    ASSERT_TRUE(GoalAccess::occupiedWindow(*planner_, plan_, 8.0, local));
    EXPECT_EQ(GoalAccess::standoffFailureReason(*planner_), "valid");
  }
}

TEST_F(OccupiedGoalTest, publishesMeasuredBrakingUntilStoppedEvenWhenPredictionCollides)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("occupied_goal_coasting_test");
  auto listener = std::make_shared<rclcpp::Node>("occupied_goal_coasting_listener");
  nav_msgs::msg::Path::SharedPtr received;
  nav_msgs::msg::Path::SharedPtr braking_path;
  teb_msgs::msg::TrajectoryMsg::SharedPtr braking_trajectory;
  auto sub = listener->create_subscription<nav_msgs::msg::Path>("local_plan", 1,
    [&](nav_msgs::msg::Path::SharedPtr msg) {received = msg;});
  auto braking_sub = listener->create_subscription<nav_msgs::msg::Path>(
    "teb_occupied_goal_braking_path", 1,
    [&](nav_msgs::msg::Path::SharedPtr msg) {braking_path = msg;});
  auto trajectory_sub = listener->create_subscription<teb_msgs::msg::TrajectoryMsg>(
    "teb_occupied_goal_braking_trajectory", 1,
    [&](teb_msgs::msg::TrajectoryMsg::SharedPtr msg) {braking_trajectory = msg;});
  GoalAccess::setGoalVisualization(*planner_, node, {makePose(0.0, 0.0), makePose(1.0, 0.0)});
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.5;
  geometry_msgs::msg::TwistStamped command;
  // A collision in the coasting corridor must not hide the predicted path.
  map_->setCost(46, 43, 254);
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  do {
    GoalAccess::stopOccupiedGoal(*planner_, makePose(0.0, 0.0), velocity, command);
    rclcpp::spin_some(listener);
    if (received && braking_path && braking_trajectory && received->poses.size() > 2 &&
        received->poses.back().pose.position.x < 0.5) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < until);
  ASSERT_NE(received, nullptr);
  ASSERT_GT(received->poses.size(), 2u);
  ASSERT_NE(braking_path, nullptr);
  ASSERT_NE(braking_trajectory, nullptr);
  EXPECT_EQ(braking_path->poses.size(), received->poses.size());
  ASSERT_EQ(braking_trajectory->trajectory.size(), received->poses.size());
  EXPECT_DOUBLE_EQ(braking_trajectory->trajectory.front().velocity.linear.x, 0.5);
  EXPECT_NEAR(braking_trajectory->trajectory.back().velocity.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(braking_trajectory->trajectory.back().time_from_start).seconds(),
    0.1 + 0.5 / 0.3, 1e-6);
  EXPECT_NEAR(received->poses.back().pose.position.x, 0.4666666667, 0.001);
  EXPECT_FALSE(GoalAccess::brakingPathSafe(*planner_, received->poses));
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  // The next hold must regenerate from current odometry, not replay the old path.
  velocity.linear.x = 0.2;
  GoalAccess::stopOccupiedGoal(*planner_, makePose(0.1, 0.0), velocity, command);
  const auto & shorter = GoalAccess::localPlan(*planner_);
  ASSERT_GT(shorter.size(), 1u);
  EXPECT_DOUBLE_EQ(shorter.front().pose.position.x, 0.1);
  EXPECT_NEAR(shorter.back().pose.position.x, 0.1 + 0.0866666667, 0.001);
  velocity.linear.x = 0.0;
  GoalAccess::stopOccupiedGoal(*planner_, makePose(0.18, 0.0), velocity, command);
  ASSERT_EQ(GoalAccess::localPlan(*planner_).size(), 1u);
  EXPECT_DOUBLE_EQ(GoalAccess::localPlan(*planner_).front().pose.position.x, 0.18);
}

TEST_F(OccupiedGoalTest, stopPredictionCoversReverseAndPureRotationAndMissingOptimizedPath)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("occupied_goal_rotation_stop_test");
  GoalAccess::setGoalVisualization(*planner_, node, {makePose(0.0, 0.0), makePose(1.0, 0.0)});
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = -0.5;
  geometry_msgs::msg::TwistStamped command;
  GoalAccess::stopOccupiedGoal(*planner_, makePose(0.0, 0.0, M_PI / 2.0), velocity, command);
  EXPECT_NEAR(GoalAccess::localPlan(*planner_).back().pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(GoalAccess::localPlan(*planner_).back().pose.position.y, -0.4666666667, 0.001);
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.6;
  GoalAccess::stopOccupiedGoal(*planner_, makePose(0.0, 0.0), velocity, command);
  const auto & rotation = GoalAccess::localPlan(*planner_);
  ASSERT_GT(rotation.size(), 1u);
  EXPECT_DOUBLE_EQ(rotation.back().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(rotation.back().pose.position.y, 0.0);
  EXPECT_GT(tf2::getYaw(rotation.back().pose.orientation), 0.06);
  GoalAccess::seedLocalPlan(*planner_, {});
  velocity.angular.z = 0.0;
  velocity.linear.x = 0.5;
  command.twist.linear.x = 0.5;
  GoalAccess::brakeGoal(*planner_, makePose(0.0, 0.0), velocity, command);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  ASSERT_GT(GoalAccess::localPlan(*planner_).size(), 1u);
  EXPECT_GT(GoalAccess::localPlan(*planner_).back().pose.position.x, 0.46);
  GoalAccess::seedLocalPlan(*planner_, {makePose(0.0, 0.0), makePose(0.4, 0.0)});
  command.twist.linear.x = 0.5;
  GoalAccess::brakeGoal(*planner_, makePose(0.0, 0.0), velocity, command);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_NEAR(GoalAccess::localPlan(*planner_).back().pose.position.x, 0.4666666667, 0.001);
  auto mismatched = makePose(1.0, 0.0);
  mismatched.header.frame_id = "odom";
  GoalAccess::seedLocalPlan(*planner_, {makePose(0.0, 0.0), mismatched});
  command.twist.linear.x = 0.5;
  GoalAccess::brakeGoal(*planner_, makePose(0.0, 0.0), velocity, command);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_NEAR(GoalAccess::localPlan(*planner_).back().pose.position.x, 0.4666666667, 0.001);
}
}  // namespace
