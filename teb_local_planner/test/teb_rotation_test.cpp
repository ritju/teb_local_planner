#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

TEST_F(SplitModuleFixture, omegaShrinksAsRemainingYawShrinks)
{
  const auto data = teb_local_planner::test_data::rotation(42u);
  const double min_omega = teb_local_planner::TebSplitModuleTestAccess::rotateMinAngularVel(*planner_);
  double previous = -1.0;
  for (const double remaining : data.remaining_rad) {
    const double omega = teb_local_planner::TebSplitModuleTestAccess::rotateToHeadingOmegaMagnitude(
      *planner_, remaining, 0.0);
    EXPECT_GE(omega, previous);
    EXPECT_GE(omega, min_omega);
    previous = omega;
  }
  const double stopped = teb_local_planner::TebSplitModuleTestAccess::rotateToHeadingOmegaMagnitude(
    *planner_, data.zero_remaining_rad, 0.0);
  EXPECT_NEAR(stopped, min_omega, 1e-6);
}

TEST_F(SplitModuleFixture, majorArcLockAndTinyRotationAreClear)
{
  teb_local_planner::TebSplitModuleTestAccess::resetMajorArcLock(*planner_);
  const auto data = teb_local_planner::test_data::rotation(77u);
  const double remaining = teb_local_planner::TebSplitModuleTestAccess::remainingYawAlongMajorArc(
    *planner_, 0.0, data.goal_yaw_rad);
  EXPECT_NEAR(remaining, std::abs(data.goal_yaw_rad), 1e-6);

  EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::isRotationCollisionFreeDecel(
    *planner_, makePose(0.0, 0.0, 0.2), 1.0, 0.0));
}
