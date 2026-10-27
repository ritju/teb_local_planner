#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/front_steering_plot.hpp"
#include "teb_local_planner/g2o_types/edge_front_steering.h"
#include "teb_local_planner/g2o_types/edge_kinematics.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

void fillFrontSteeringConfig(teb_local_planner::TebConfig & cfg)
{
  cfg.robot.front_steering_rear_drive = true;
  cfg.robot.wheelbase = teb_local_planner::kFrontSteeringWheelbase;
  cfg.robot.rear_track = teb_local_planner::kFrontSteeringRearTrack;
  cfg.robot.max_steering_angle = teb_local_planner::kFrontSteeringMaxAngle;
  cfg.robot.max_steering_rate = teb_local_planner::kFrontSteeringMaxRate;
  cfg.robot.max_steering_acceleration = teb_local_planner::kFrontSteeringMaxAcceleration;
  cfg.robot.steering_min_speed = teb_local_planner::kFrontSteeringMinSpeed;
  cfg.robot.steering_omega_epsilon = teb_local_planner::kFrontSteeringOmegaEpsilon;
  cfg.robot.no_inplace_rotation = true;
  cfg.robot.cmd_angle_instead_rotvel = false;
  cfg.robot.min_turning_radius = teb_local_planner::minimumTurningRadius(
    teb_local_planner::kFrontSteeringWheelbase, teb_local_planner::kFrontSteeringMaxAngle);
  cfg.robot.max_vel_y = 0.0;
  cfg.robot.max_vel_theta = 3.0;
  cfg.optim.penalty_epsilon = 0.0;
  cfg.optim.weight_kinematics_nh = 1000.0;
  cfg.optim.weight_kinematics_turning_radius = 1000.0;
  cfg.optim.weight_steering_kinematics = 500.0;
  cfg.optim.weight_steering_rate = 300.0;
  cfg.optim.weight_steering_acceleration = 200.0;
}

double edgeError(teb_local_planner::BaseTebMultiEdge<1, double> & edge)
{
  Eigen::Matrix<double, 1, 1> information;
  information << 1.0;
  edge.setInformation(information);
  return edge.getError()[0];
}

double kinematicsError(
  const teb_local_planner::TebConfig & cfg, const teb_local_planner::PoseSE2 & first,
  const teb_local_planner::PoseSE2 & second, double dt)
{
  teb_local_planner::VertexPose pose1(first, true);
  teb_local_planner::VertexPose pose2(second, true);
  teb_local_planner::VertexTimeDiff timediff(dt, true);
  teb_local_planner::EdgeSteeringKinematics edge;
  edge.setTebConfig(cfg);
  edge.setVertex(0, &pose1);
  edge.setVertex(1, &pose2);
  edge.setVertex(2, &timediff);
  return edgeError(edge);
}

double rateError(
  const teb_local_planner::TebConfig & cfg, const teb_local_planner::PoseSE2 & first,
  const teb_local_planner::PoseSE2 & second, const teb_local_planner::PoseSE2 & third, double dt)
{
  teb_local_planner::VertexPose pose1(first, true);
  teb_local_planner::VertexPose pose2(second, true);
  teb_local_planner::VertexPose pose3(third, true);
  teb_local_planner::VertexTimeDiff dt1(dt, true);
  teb_local_planner::VertexTimeDiff dt2(dt, true);
  teb_local_planner::EdgeSteeringRate edge;
  edge.setTebConfig(cfg);
  edge.setVertex(0, &pose1);
  edge.setVertex(1, &pose2);
  edge.setVertex(2, &pose3);
  edge.setVertex(3, &dt1);
  edge.setVertex(4, &dt2);
  return edgeError(edge);
}

double accelerationError(
  const teb_local_planner::TebConfig & cfg, const std::vector<teb_local_planner::PoseSE2> & poses, double dt)
{
  teb_local_planner::VertexPose pose1(poses[0], true);
  teb_local_planner::VertexPose pose2(poses[1], true);
  teb_local_planner::VertexPose pose3(poses[2], true);
  teb_local_planner::VertexPose pose4(poses[3], true);
  teb_local_planner::VertexTimeDiff dt1(dt, true);
  teb_local_planner::VertexTimeDiff dt2(dt, true);
  teb_local_planner::VertexTimeDiff dt3(dt, true);
  teb_local_planner::EdgeSteeringAcceleration edge;
  edge.setTebConfig(cfg);
  edge.setVertex(0, &pose1);
  edge.setVertex(1, &pose2);
  edge.setVertex(2, &pose3);
  edge.setVertex(3, &pose4);
  edge.setVertex(4, &dt1);
  edge.setVertex(5, &dt2);
  edge.setVertex(6, &dt3);
  return edgeError(edge);
}

double carlikeRadiusError(
  const teb_local_planner::TebConfig & cfg, const teb_local_planner::PoseSE2 & first,
  const teb_local_planner::PoseSE2 & second)
{
  teb_local_planner::VertexPose pose1(first, true);
  teb_local_planner::VertexPose pose2(second, true);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setTebConfig(cfg);
  edge.setVertex(0, &pose1);
  edge.setVertex(1, &pose2);
  Eigen::Matrix<double, 2, 2> information;
  information.setZero();
  information(1, 1) = 1.0;
  edge.setInformation(information);
  return edge.getError()[1];
}

std::vector<teb_local_planner::PoseSE2> circularArc(double radius, double step, int segments)
{
  std::vector<teb_local_planner::PoseSE2> poses;
  for (int index = 0; index <= segments; ++index)
  {
    const double phi = index * step;
    poses.push_back(teb_local_planner::PoseSE2(
      radius * std::sin(phi), radius * (1.0 - std::cos(phi)), phi));
  }
  return poses;
}

}  // namespace

TEST(FrontSteeringMath, straightLineHasZeroSteeringError)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const double dt = 1.0;
  std::vector<teb_local_planner::PoseSE2> poses;
  poses.push_back(teb_local_planner::PoseSE2(0.0, 0.0, 0.0));
  poses.push_back(teb_local_planner::PoseSE2(1.0, 0.0, 0.0));
  poses.push_back(teb_local_planner::PoseSE2(2.0, 0.0, 0.0));
  poses.push_back(teb_local_planner::PoseSE2(3.0, 0.0, 0.0));

  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    poses[0], poses[1], dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, segment));
  EXPECT_GT(segment.velocity, 0.0);
  EXPECT_NEAR(segment.omega, 0.0, 1e-9);
  EXPECT_NEAR(segment.curvature, 0.0, 1e-9);
  EXPECT_NEAR(segment.steering_angle, 0.0, 1e-9);
  EXPECT_NEAR(kinematicsError(cfg, poses[0], poses[1], dt), 0.0, 1e-9);
  EXPECT_NEAR(rateError(cfg, poses[0], poses[1], poses[2], dt), 0.0, 1e-9);
  EXPECT_NEAR(accelerationError(cfg, poses, dt), 0.0, 1e-9);
}

TEST(FrontSteeringMath, constantArcKeepsSteeringInsideLimit)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const auto poses = circularArc(1.0, 0.2, 3);
  const double dt = 0.2;
  teb_local_planner::SegmentKinematics first;
  teb_local_planner::SegmentKinematics second;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    poses[0], poses[1], dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, first));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    poses[1], poses[2], dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, second));
  const double expected_delta = std::atan(cfg.robot.wheelbase / 1.0);
  EXPECT_NEAR(first.steering_angle, expected_delta, 0.02);
  EXPECT_LT(std::abs(first.steering_angle), cfg.robot.max_steering_angle);
  EXPECT_NEAR(first.steering_angle, second.steering_angle, 1e-9);
  EXPECT_NEAR(teb_local_planner::steeringRate(first, second), 0.0, 1e-9);
  EXPECT_NEAR(kinematicsError(cfg, poses[0], poses[1], dt), 0.0, 1e-9);
  EXPECT_NEAR(rateError(cfg, poses[0], poses[1], poses[2], dt), 0.0, 1e-9);
  EXPECT_NEAR(accelerationError(cfg, poses, dt), 0.0, 1e-9);
}

TEST(FrontSteeringMath, tightRadiusProducesSteeringError)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const auto poses = circularArc(0.4, 0.2, 1);
  const double dt = 0.2;
  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    poses[0], poses[1], dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, segment));
  const double documented_delta = std::atan(cfg.robot.wheelbase / 0.4);
  EXPECT_NEAR(segment.steering_angle, documented_delta, 0.03);
  EXPECT_GT(std::abs(segment.steering_angle), cfg.robot.max_steering_angle);
  EXPECT_GT(kinematicsError(cfg, poses[0], poses[1], dt), 0.0);
  EXPECT_GT(carlikeRadiusError(cfg, poses[0], poses[1]), 0.0);
}

TEST(FrontSteeringMath, steeringRateBoundary)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const double dt = 0.2;
  const teb_local_planner::PoseSE2 start(0.0, 0.0, 0.0);
  const teb_local_planner::PoseSE2 middle =
    teb_local_planner::poseAfterLongitudinalSegment(start, 0.4, 0.0, cfg.robot.wheelbase);
  const teb_local_planner::PoseSE2 on_limit =
    teb_local_planner::poseAfterLongitudinalSegment(middle, 0.4, 0.1, cfg.robot.wheelbase);
  const teb_local_planner::PoseSE2 above_limit =
    teb_local_planner::poseAfterLongitudinalSegment(middle, 0.4, 0.11, cfg.robot.wheelbase);

  teb_local_planner::SegmentKinematics first;
  teb_local_planner::SegmentKinematics bounded;
  teb_local_planner::SegmentKinematics exceeded;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, middle, dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, first));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    middle, on_limit, dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, bounded));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    middle, above_limit, dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, exceeded));
  EXPECT_NEAR(first.steering_angle, 0.0, 1e-9);
  EXPECT_NEAR(bounded.steering_angle, 0.1, 1e-9);
  EXPECT_NEAR(teb_local_planner::steeringRate(first, bounded), 0.5, 1e-9);
  EXPECT_NEAR(rateError(cfg, start, middle, on_limit, dt), 0.0, 1e-9);
  EXPECT_NEAR(teb_local_planner::steeringRate(first, exceeded), 0.55, 1e-9);
  EXPECT_NEAR(rateError(cfg, start, middle, above_limit, dt), 0.05, 1e-9);
}

TEST(FrontSteeringMath, steeringRateScalesWithTime)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const teb_local_planner::PoseSE2 start(0.0, 0.0, 0.0);
  const teb_local_planner::PoseSE2 middle =
    teb_local_planner::poseAfterLongitudinalSegment(start, 0.4, 0.0, cfg.robot.wheelbase);
  const teb_local_planner::PoseSE2 end =
    teb_local_planner::poseAfterLongitudinalSegment(middle, 0.4, 0.1, cfg.robot.wheelbase);
  teb_local_planner::SegmentKinematics slow_first;
  teb_local_planner::SegmentKinematics slow_second;
  teb_local_planner::SegmentKinematics fast_first;
  teb_local_planner::SegmentKinematics fast_second;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, middle, 0.2, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, slow_first));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    middle, end, 0.2, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, slow_second));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, middle, 0.1, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, fast_first));
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    middle, end, 0.1, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, fast_second));
  EXPECT_NEAR(teb_local_planner::steeringRate(slow_first, slow_second), 0.5, 1e-9);
  EXPECT_NEAR(teb_local_planner::steeringRate(fast_first, fast_second), 1.0, 1e-9);
  EXPECT_NEAR(rateError(cfg, start, middle, end, 0.2), 0.0, 1e-9);
  EXPECT_GT(rateError(cfg, start, middle, end, 0.1), 0.0);
}

TEST(FrontSteeringMath, reverseKeepsSteeringSign)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const teb_local_planner::PoseSE2 start(0.0, 0.0, 0.0);
  const teb_local_planner::PoseSE2 end =
    teb_local_planner::poseAfterLongitudinalSegment(start, -0.4, -0.2, cfg.robot.wheelbase);
  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, end, 0.4, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, segment));
  EXPECT_LT(segment.velocity, 0.0);
  EXPECT_GT(segment.omega, 0.0);
  EXPECT_LT(segment.steering_angle, 0.0);
  EXPECT_NEAR(segment.steering_angle, std::atan(cfg.robot.wheelbase * segment.omega / segment.velocity), 1e-9);
  const double unsigned_delta = std::atan(
    cfg.robot.wheelbase * std::abs(segment.omega) / std::abs(segment.velocity));
  EXPECT_NE(segment.steering_angle, unsigned_delta);
}

TEST(FrontSteeringMath, inplaceRotationIsInfeasible)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const teb_local_planner::PoseSE2 start(1.0, 2.0, 0.3);
  const teb_local_planner::PoseSE2 end(1.0, 2.0, 0.8);
  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, end, 0.2, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, segment));
  EXPECT_NEAR(segment.velocity, 0.0, 1e-9);
  EXPECT_GT(std::abs(segment.omega), cfg.robot.steering_omega_epsilon);
  EXPECT_TRUE(segment.inplace_rotation);
  EXPECT_GT(kinematicsError(cfg, start, end, 0.2), 0.2);
}

TEST(FrontSteeringMath, angleWrapUsesShortestDelta)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const double from = 179.0 * M_PI / 180.0;
  const double to = -179.0 * M_PI / 180.0;
  const teb_local_planner::PoseSE2 start(0.0, 0.0, from);
  const teb_local_planner::PoseSE2 end(0.2, 0.0, to);
  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    start, end, 0.2, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
    cfg.robot.steering_omega_epsilon, segment));
  EXPECT_NEAR(segment.delta_theta, 2.0 * M_PI / 180.0, 1e-9);
  EXPECT_GT(segment.omega, 0.0);
  EXPECT_LT(std::abs(segment.delta_theta), 0.1);
}

TEST(FrontSteeringCommand, projectsOntoBicycleAndRearWheels)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const double vx = 0.5;
  const double omega = 0.2;
  teb_local_planner::FrontSteeringCommandState previous;
  const auto command = teb_local_planner::projectFrontSteeringCommand(
    vx, 0.15, omega, 0.1, cfg.robot, previous);
  const double delta = std::atan(cfg.robot.wheelbase * command.omega / command.vx);
  EXPECT_NEAR(command.vy, 0.0, 1e-12);
  EXPECT_NEAR(command.vx, vx, 1e-12);
  EXPECT_NEAR(command.omega, omega, 1e-12);
  EXPECT_NEAR(command.steering_angle, delta, 1e-12);
  EXPECT_NEAR(command.omega, command.vx * std::tan(command.steering_angle) / cfg.robot.wheelbase, 1e-12);
  EXPECT_NEAR(command.rear_left, vx - 0.5 * cfg.robot.rear_track * omega, 1e-12);
  EXPECT_NEAR(command.rear_right, vx + 0.5 * cfg.robot.rear_track * omega, 1e-12);

  const auto stopped = teb_local_planner::projectFrontSteeringCommand(
    0.0, 0.0, 0.8, 0.1, cfg.robot, previous);
  EXPECT_NEAR(stopped.vx, 0.0, 1e-12);
  EXPECT_NEAR(stopped.vy, 0.0, 1e-12);
  EXPECT_NEAR(stopped.omega, 0.0, 1e-12);

  cfg.robot.max_vel_theta = 0.4;
  const auto capped = teb_local_planner::projectFrontSteeringCommand(
    1.0, 0.2, 3.0, 0.1, cfg.robot, previous);
  EXPECT_NEAR(capped.vy, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(capped.omega), cfg.robot.max_vel_theta, 1e-12);
  EXPECT_NEAR(
    capped.omega, capped.vx * std::tan(capped.steering_angle) / cfg.robot.wheelbase, 1e-9);
}

TEST(FrontSteeringCommand, limitsSteeringRateAndAcceleration)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  teb_local_planner::FrontSteeringCommandState previous;
  previous.valid = true;
  previous.steering_angle = 0.0;
  previous.steering_rate = cfg.robot.max_steering_rate;
  const double dt = 0.2;
  const double vx = 0.6;
  const double desired_delta = 0.3;
  const double desired_omega = vx * std::tan(desired_delta) / cfg.robot.wheelbase;
  const auto command = teb_local_planner::projectFrontSteeringCommand(
    vx, 0.0, desired_omega, dt, cfg.robot, previous);
  EXPECT_NEAR(command.steering_angle, cfg.robot.max_steering_rate * dt, 1e-9);
  EXPECT_NEAR(
    command.omega, vx * std::tan(command.steering_angle) / cfg.robot.wheelbase, 1e-9);
  EXPECT_LE(std::abs(command.steering_angle), cfg.robot.max_steering_angle);
}

TEST(FrontSteeringOptimization, optimizedArcMatchesBicycleModel)
{
  const auto plot = teb_local_planner::buildFrontSteeringVerificationPlot();
  ASSERT_TRUE(plot.arc_optimized);
  EXPECT_LT(plot.max_heading_error, 0.08);
  EXPECT_LT(plot.max_position_error, 0.05);
  EXPECT_LT(plot.max_abs_delta, teb_local_planner::kFrontSteeringMaxAngle);
  EXPECT_LE(plot.max_abs_rate, teb_local_planner::kFrontSteeringMaxRate + 1e-3);
  EXPECT_LE(plot.max_abs_acceleration, teb_local_planner::kFrontSteeringMaxAcceleration + 1e-3);
  EXPECT_NEAR(plot.max_model_omega_error, 0.0, 1e-8);
  ASSERT_FALSE(plot.optimized_commands.empty());
  EXPECT_GT(plot.optimized_commands.front().v, 0.0);
  EXPECT_NEAR(plot.optimized_commands.front().rear_right - plot.optimized_commands.front().rear_left,
    teb_local_planner::kFrontSteeringRearTrack * plot.optimized_commands.front().omega, 1e-9);
}

TEST(FrontSteeringOptimization, timeStretchRemovesRateViolation)
{
  const auto plot = teb_local_planner::buildFrontSteeringVerificationPlot();
  ASSERT_TRUE(plot.rate_optimized);
  EXPECT_NEAR(plot.rate_before, 1.0, 1e-6);
  EXPECT_LE(std::abs(plot.rate_after), teb_local_planner::kFrontSteeringMaxRate + 0.05);
  EXPECT_LT(std::abs(plot.rate_after), std::abs(plot.rate_before));
  ASSERT_GE(plot.time_stretched_commands.size(), 2u);
  for (const auto & command : plot.time_stretched_commands)
  {
    if (std::abs(command.v) > teb_local_planner::kFrontSteeringMinSpeed)
    {
      EXPECT_NEAR(
        command.omega,
        command.v * std::tan(command.steering_angle) / teb_local_planner::kFrontSteeringWheelbase,
        1e-8);
    }
    EXPECT_LE(std::abs(command.steering_angle), teb_local_planner::kFrontSteeringMaxAngle + 1e-6);
  }
}

TEST(FrontSteeringOptimization, timeScalingDoesNotRemoveTightRadiusError)
{
  const auto plot = teb_local_planner::buildFrontSteeringVerificationPlot();
  teb_local_planner::SegmentKinematics segment;
  ASSERT_TRUE(teb_local_planner::computeSteeringSegment(
    plot.tight_arc[0], plot.tight_arc[1], 0.2, teb_local_planner::kFrontSteeringWheelbase,
    teb_local_planner::kFrontSteeringMinSpeed, teb_local_planner::kFrontSteeringOmegaEpsilon, segment));
  EXPECT_GT(std::abs(segment.steering_angle), teb_local_planner::kFrontSteeringMaxAngle);
  EXPECT_GT(plot.tight_error_before, 0.0);
  EXPECT_GT(plot.tight_error_after, 0.0);
  EXPECT_LT(plot.tight_error_after, plot.tight_error_before);
}

TEST(FrontSteeringJacobian, finiteDifferenceIsFinitePastSteeringLimit)
{
  teb_local_planner::TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  const auto poses = circularArc(0.4, 0.2, 1);
  const double dt = 0.2;
  const double step = 1e-6;
  auto error_at = [&](double theta) {
    teb_local_planner::PoseSE2 moved = poses[1];
    moved.theta() = theta;
    return kinematicsError(cfg, poses[0], moved, dt);
  };
  const double derivative = (error_at(poses[1].theta() + step) - error_at(poses[1].theta() - step)) / (2.0 * step);
  EXPECT_TRUE(std::isfinite(derivative));
  EXPECT_GT(std::abs(derivative), 1.0);
}
