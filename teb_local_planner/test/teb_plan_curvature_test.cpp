#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/front_steering_plot.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <string>

namespace
{

void expectOptimizedBend(const teb_local_planner::PlanCurvatureOptimization & result)
{
  ASSERT_TRUE(result.succeeded) << result.name;
  EXPECT_EQ(result.optimized.size(), result.reference.size());
  EXPECT_GT(result.logged_peak_curvature, 1.0);
  EXPECT_GT(result.peak_reference_curvature, 1.0);
  EXPECT_GT(result.max_optimized_delta, 0.55);
  EXPECT_LT(result.max_optimized_delta, teb_local_planner::kFrontSteeringMaxAngle);
  EXPECT_NEAR(result.max_model_omega_error, 0.0, 1e-6);
  EXPECT_LT(result.max_position_error, 0.35);
  EXPECT_LT(result.max_abs_lateral, 0.08);
  EXPECT_LE(result.max_abs_rate, teb_local_planner::kFrontSteeringMaxRate + 0.05);
  ASSERT_FALSE(result.commands.empty());
  EXPECT_NEAR(
    result.commands.front().rear_right - result.commands.front().rear_left,
    teb_local_planner::kFrontSteeringRearTrack * result.commands.front().omega, 1e-9);
}

}  // namespace

TEST(PlanCurvature, bend1OptimizedPosesMatchFrontSteering)
{
  expectOptimizedBend(teb_local_planner::optimizePlanCurvatureSegment(0));
}

TEST(PlanCurvature, bend2OptimizedPosesMatchFrontSteering)
{
  expectOptimizedBend(teb_local_planner::optimizePlanCurvatureSegment(1));
}

TEST(PlanCurvature, bend3OptimizedPosesMatchFrontSteering)
{
  expectOptimizedBend(teb_local_planner::optimizePlanCurvatureSegment(2));
}

TEST(PlanCurvature, bend4OptimizedPosesMatchFrontSteering)
{
  expectOptimizedBend(teb_local_planner::optimizePlanCurvatureSegment(3));
}

TEST(PlanCurvature, bend5OptimizedPosesMatchFrontSteering)
{
  expectOptimizedBend(teb_local_planner::optimizePlanCurvatureSegment(4));
}

void expectFullPathSimulation(const teb_local_planner::PlanPathSimulation & result)
{
  ASSERT_TRUE(result.succeeded) << result.name;
  EXPECT_EQ(result.simulated.size(), result.reference.size());
  EXPECT_EQ(result.nearest_reference.size(), result.simulated.size());
  EXPECT_GT(result.pose_count, 200);
  EXPECT_GT(result.path_length, 10.0);
  EXPECT_GT(result.max_sample_spacing, 0.0);
  EXPECT_LT(result.max_cross_track, result.max_sample_spacing);
  EXPECT_LT(result.rms_cross_track, result.max_sample_spacing);
  EXPECT_LT(result.final_position_error, result.max_sample_spacing);
  EXPECT_NEAR(result.max_heading_error, 0.0, 1e-6);
  EXPECT_NEAR(result.max_model_omega_error, 0.0, 1e-9);
  EXPECT_LE(result.max_abs_delta, teb_local_planner::kFrontSteeringMaxAngle + 1e-9);
  EXPECT_LE(result.max_abs_rate, teb_local_planner::kFrontSteeringMaxRate + 1e-6);
  EXPECT_LE(result.max_abs_acceleration, teb_local_planner::kFrontSteeringMaxAcceleration + 1e-6);
  EXPECT_TRUE(result.commands_satisfy_model);
  ASSERT_GT(result.commands.size(), 3u);
  EXPECT_NEAR(
    result.commands.front().rear_right - result.commands.front().rear_left,
    teb_local_planner::kFrontSteeringRearTrack * result.commands.front().omega, 1e-9);
  for (size_t index = 0; index + 1 < result.commands.size(); ++index)
  {
    const auto & current = result.commands[index];
    const auto & next = result.commands[index + 1];
    const double average_dt = 0.5 * (current.segment_dt + next.segment_dt);
    ASSERT_GT(average_dt, 1e-4) << result.name << " segment " << index;
    const double rate = (next.steering_angle - current.steering_angle) / average_dt;
    EXPECT_NEAR(current.steering_rate, rate, 1e-9) << result.name << " segment " << index;
    EXPECT_LE(std::abs(current.steering_rate), teb_local_planner::kFrontSteeringMaxRate + 1e-6);
  }
  for (size_t index = 0; index + 2 < result.commands.size(); ++index)
  {
    const auto & first = result.commands[index];
    const auto & second = result.commands[index + 1];
    const auto & third = result.commands[index + 2];
    const double first_average = 0.5 * (first.segment_dt + second.segment_dt);
    const double second_average = 0.5 * (second.segment_dt + third.segment_dt);
    const double acceleration_dt = 0.5 * (first_average + second_average);
    ASSERT_GT(acceleration_dt, 1e-4) << result.name << " segment " << index;
    const double acceleration = (second.steering_rate - first.steering_rate) / acceleration_dt;
    EXPECT_NEAR(first.steering_acceleration, acceleration, 1e-6)
      << result.name << " segment " << index;
    EXPECT_LE(
      std::abs(first.steering_acceleration),
      teb_local_planner::kFrontSteeringMaxAcceleration + 1e-6);
  }
  EXPECT_NEAR(
    result.commands.back().steering_rate,
    result.commands[result.commands.size() - 2].steering_rate, 1e-12);
  EXPECT_NEAR(
    result.commands.back().steering_acceleration,
    result.commands[result.commands.size() - 3].steering_acceleration, 1e-12);

  auto record = [](const char * key, double value) {
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(6);
    text << value;
    ::testing::Test::RecordProperty(key, text.str());
  };
  record("path_length_m", result.path_length);
  record("max_cross_track_m", result.max_cross_track);
  record("rms_cross_track_m", result.rms_cross_track);
  record("final_position_error_m", result.final_position_error);
  record("max_heading_error_rad", result.max_heading_error);
  record("max_delta_rad", result.max_abs_delta);
  record("max_steering_rate_rad_s", result.max_abs_rate);
  record("max_steering_acc_rad_s2", result.max_abs_acceleration);
  record("max_model_omega_error", result.max_model_omega_error);
  record("max_reference_lateral_m", result.max_reference_lateral);
  ::testing::Test::RecordProperty("saturated_steps", std::to_string(result.saturated_steps));
  ::testing::Test::RecordProperty("pose_count", std::to_string(result.pose_count));
}

TEST(PlanPathSimulation, fullPath18TracksAndSatisfiesBicycleModel)
{
  expectFullPathSimulation(teb_local_planner::simulateFullPlanPath(0));
}

TEST(PlanPathSimulation, fullPath23TracksAndSatisfiesBicycleModel)
{
  expectFullPathSimulation(teb_local_planner::simulateFullPlanPath(1));
}

TEST(PlanPathSimulation, fullPath11TracksAndSatisfiesBicycleModel)
{
  expectFullPathSimulation(teb_local_planner::simulateFullPlanPath(2));
}

TEST(PlanPathSimulation, fullPath12TracksAndSatisfiesBicycleModel)
{
  expectFullPathSimulation(teb_local_planner::simulateFullPlanPath(3));
}

TEST(PlanPathSimulation, fullPath19TracksAndSatisfiesBicycleModel)
{
  expectFullPathSimulation(teb_local_planner::simulateFullPlanPath(4));
}
