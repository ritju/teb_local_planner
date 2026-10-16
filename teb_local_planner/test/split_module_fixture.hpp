#pragma once

#include <gtest/gtest.h>

#include "teb_local_planner/teb_local_planner_ros.h"
#include "teb_params_loader.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>
#include <memory>

namespace teb_local_planner
{

struct TebSplitModuleTestAccess
{
  static void setClock(TebLocalPlannerROS & planner)
  {
    planner.clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  }

  static bool cornerApproachCostIsBlocking(
    const TebLocalPlannerROS & planner, unsigned char cost, const std::string & cost_mode)
  {
    return planner.cornerApproachCostIsBlocking(cost, cost_mode);
  }

  static double cornerApproachCheckMargin(const TebLocalPlannerROS & planner)
  {
    return planner.cornerApproachCheckMargin();
  }

  static double rotateToHeadingOmegaMagnitude(
    const TebLocalPlannerROS & planner, double remaining_angle_rad, double omega_current_abs)
  {
    return planner.rotateToHeadingOmegaMagnitude(remaining_angle_rad, omega_current_abs);
  }

  static void resetMajorArcLock(TebLocalPlannerROS & planner)
  {
    planner.resetMajorArcLock();
  }

  static double remainingYawAlongMajorArc(
    const TebLocalPlannerROS & planner, double yaw_now, double yaw_goal)
  {
    return planner.remainingYawAlongMajorArc(yaw_now, yaw_goal);
  }

  static bool isRotationCollisionFreeDecel(
    const TebLocalPlannerROS & planner,
    const geometry_msgs::msg::PoseStamped & pose,
    double omega_direction_sign,
    double rotation_magnitude_rad)
  {
    return planner.isRotationCollisionFreeDecel(
      pose, omega_direction_sign, rotation_magnitude_rad, 0.0);
  }

  static bool isTransformedPlanFootprintSamplesCollisionFree(
    const TebLocalPlannerROS & planner,
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    double sample_spacing_m)
  {
    return planner.isTransformedPlanFootprintSamplesCollisionFree(plan, sample_spacing_m);
  }

  static void setViaPointSeparation(TebLocalPlannerROS & planner, double separation)
  {
    planner.cfg_->trajectory.global_plan_viapoint_sep = separation;
  }

  static bool customViaPointsActive(const TebLocalPlannerROS & planner)
  {
    return planner.custom_via_points_active_;
  }

  static bool isVehicleInEdgeFollowingExitCorridor(
    TebLocalPlannerROS & planner,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const Eigen::Vector2d & wall_w0,
    const Eigen::Vector2d & wall_w1,
    const geometry_msgs::msg::Pose & vehicle_pose)
  {
    return planner.isVehicleInEdgeFollowingExitCorridor(
      robot_pose, wall_w0, wall_w1, vehicle_pose);
  }

  static bool pruneGlobalPlan(
    TebLocalPlannerROS & planner,
    const geometry_msgs::msg::PoseStamped & global_pose,
    std::vector<geometry_msgs::msg::PoseStamped> & global_plan,
    double dist_behind_robot,
    double max_prune_dist)
  {
    return planner.pruneGlobalPlan(global_pose, global_plan, dist_behind_robot, max_prune_dist);
  }

  static double estimateLocalGoalOrientation(
    const TebLocalPlannerROS & planner,
    const std::vector<geometry_msgs::msg::PoseStamped> & global_plan,
    const geometry_msgs::msg::PoseStamped & local_goal,
    int current_goal_idx,
    const geometry_msgs::msg::TransformStamped & tf_plan_to_global,
    int moving_average_length)
  {
    return planner.estimateLocalGoalOrientation(
      global_plan, local_goal, current_goal_idx, tf_plan_to_global, moving_average_length);
  }

  static void customViaPointsCB(
    TebLocalPlannerROS & planner, const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
  {
    planner.customViaPointsCB(via_points_msg);
  }

  static bool extractLineSegmentFromPath(
    const TebLocalPlannerROS & planner,
    const nav_msgs::msg::Path & path,
    Eigen::Vector2d & segment_start,
    Eigen::Vector2d & segment_end)
  {
    return planner.extractLineSegmentFromPath(path, segment_start, segment_end);
  }

  static double pointToSegmentDistance(
    const TebLocalPlannerROS & planner,
    const Eigen::Vector2d & query_point,
    const Eigen::Vector2d & segment_start,
    const Eigen::Vector2d & segment_end)
  {
    return planner.pointToSegmentDistance(query_point, segment_start, segment_end);
  }

  static double segmentToSegmentDistance(
    const TebLocalPlannerROS & planner,
    const Eigen::Vector2d & first_segment_start,
    const Eigen::Vector2d & first_segment_end,
    const Eigen::Vector2d & second_segment_start,
    const Eigen::Vector2d & second_segment_end)
  {
    return planner.segmentToSegmentDistance(
      first_segment_start, first_segment_end, second_segment_start, second_segment_end);
  }

  static double segmentDirectionAngleDifferenceDeg(
    const TebLocalPlannerROS & planner,
    const Eigen::Vector2d & first_segment_start,
    const Eigen::Vector2d & first_segment_end,
    const Eigen::Vector2d & second_segment_start,
    const Eigen::Vector2d & second_segment_end)
  {
    return planner.segmentDirectionAngleDifferenceDeg(
      first_segment_start, first_segment_end, second_segment_start, second_segment_end);
  }

  static bool narrowPolygonsAvailable(const TebLocalPlannerROS & planner)
  {
    return planner.narrowPolygonsAvailable();
  }

  static void narrowPassagesCallback(
    TebLocalPlannerROS & planner, const garage_utils_msgs::msg::Polygons::SharedPtr msg)
  {
    planner.narrowPassagesCallback(msg);
  }

  static bool isPointInNarrowPassage(const TebLocalPlannerROS & planner, double x, double y)
  {
    return planner.isPointInNarrowPassage(x, y);
  }

  static bool hasReverseSegmentInPlan(
    const TebLocalPlannerROS & planner,
    const std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    return planner.hasReverseSegmentInPlan(plan);
  }

  static void enableBackwardCallback(
    TebLocalPlannerROS & planner, const std_msgs::msg::Bool::SharedPtr msg)
  {
    planner.enableBackwardCallback(msg);
  }

  static void resetBackwardModePublicationState(TebLocalPlannerROS & planner, bool publish_false)
  {
    planner.resetBackwardModePublicationState(publish_false);
  }

  static void publishBackwardMode(TebLocalPlannerROS & planner, bool backward)
  {
    planner.publishBackwardMode(backward);
  }

  static void resetDualArcReverseLatch(TebLocalPlannerROS & planner)
  {
    planner.resetDualArcReverseLatch();
  }

  static bool isBackgroundWorkAllowed(const TebLocalPlannerROS & planner)
  {
    return planner.isBackgroundWorkAllowed();
  }

  static bool waitForServiceInterruptible(
    TebLocalPlannerROS & planner,
    const rclcpp::ClientBase::SharedPtr & client,
    std::chrono::milliseconds total_timeout)
  {
    return planner.waitForServiceInterruptible(client, total_timeout);
  }

  static void spawnBackgroundTask(TebLocalPlannerROS & planner, std::function<void()> task)
  {
    planner.spawnBackgroundTask(std::move(task));
  }

  static void joinBackgroundTasks(TebLocalPlannerROS & planner, std::chrono::milliseconds timeout)
  {
    planner.joinBackgroundTasks(timeout);
  }

  static bool collectPathWaitObstacles(
    TebLocalPlannerROS & planner,
    std::vector<PathWaitObstacle> & out,
    bool & message_valid)
  {
    return planner.collectPathWaitObstacles(out, message_valid);
  }

  static bool tryGoalStandoffHold(
    TebLocalPlannerROS & planner,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    geometry_msgs::msg::TwistStamped & cmd_vel)
  {
    return planner.tryGoalStandoffHold(robot_pose, velocity, cmd_vel);
  }

  static void markInitialized(TebLocalPlannerROS & planner)
  {
    planner.initialized_ = true;
  }

  static void markUninitialized(TebLocalPlannerROS & planner)
  {
    planner.initialized_ = false;
  }

  static void saturateVelocity(
    const TebLocalPlannerROS & planner,
    double & vx, double & vy, double & omega,
    double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards)
  {
    planner.saturateVelocity(
      vx, vy, omega, max_vel_x, max_vel_y, max_vel_theta, max_vel_x_backwards);
  }

  static double convertTransRotVelToSteeringAngle(
    const TebLocalPlannerROS & planner,
    double linear, double omega, double wheelbase, double min_turning_radius)
  {
    return planner.convertTransRotVelToSteeringAngle(
      linear, omega, wheelbase, min_turning_radius);
  }

  static std::size_t globalPlanSize(const TebLocalPlannerROS & planner)
  {
    return planner.global_plan_.size();
  }

  static void loadFollowPathParams(TebLocalPlannerROS & planner)
  {
    test_data::loadFollowPathParams(*planner.cfg_);
  }

  static double rotateMinAngularVel(const TebLocalPlannerROS & planner)
  {
    return planner.cfg_->rotation.rotate_min_angular_vel;
  }
};

}  // namespace teb_local_planner

namespace
{

struct EnsureRclcpp
{
  EnsureRclcpp()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }
};

static EnsureRclcpp ensure_rclcpp;

inline geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

class SplitModuleFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_unique<teb_local_planner::TebLocalPlannerROS>();
    teb_local_planner::TebSplitModuleTestAccess::setClock(*planner_);
    teb_local_planner::TebSplitModuleTestAccess::loadFollowPathParams(*planner_);
  }

  void TearDown() override
  {
    planner_.reset();
  }

  std::unique_ptr<teb_local_planner::TebLocalPlannerROS> planner_;
};

}  // namespace
