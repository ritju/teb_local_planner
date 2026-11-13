#include "teb_local_planner/teb_local_planner_ros.h"
#include "teb_local_planner/front_steer_drive_interface.hpp"
#include "teb_local_planner/plan_initialization.hpp"
#include <gtest/gtest.h>
#include <nav2_core/exceptions.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>

namespace teb_local_planner
{
struct FrontSteerInterfaceTestAccess
{
  static TebConfig::Robot & robot(TebLocalPlannerROS & p) {return p.cfg_->robot;}
  static void initialize(TebLocalPlannerROS & p, nav2_util::LifecycleNode::SharedPtr node,
    const std::string & mode)
  {
    p.name_ = "FollowPath";
    p.clock_ = node->get_clock();
    p.logger_ = node->get_logger();
    p.front_steer_interface_mode_ = mode;
    p.front_steer_command_topic_ = "/teb_interface_test/cmd";
    p.front_steer_state_topic_ = "/teb_interface_test/state";
    p.front_steer_control_dt_ = 0.2;
    p.initializeFrontSteerDriveInterface(node);
  }
  static void enableLagScaling(TebLocalPlannerROS & p) {p.front_steer_lag_speed_scaling_ = true;}
  static double lagCap(TebLocalPlannerROS & p) {return p.front_steer_lag_speed_limit_;}
  static void enableOdom(TebLocalPlannerROS & p)
  {p.front_steer_odom_enabled_ = true; p.front_steer_odom_topic_ = "/teb_interface_test/odom";}
  static bool hasOdomBridge(TebLocalPlannerROS & p)
  {return p.front_steer_odom_sub_ && p.front_steer_sim_state_pub_;}
  static void receive(TebLocalPlannerROS & p, double v, double delta, double rpm = 0.0)
  {
    auto state = std::make_shared<capella_ros_msg::msg::FrontSteerDriveStu>();
    state->header.stamp = p.clock_->now();
    state->header.frame_id = "base_link";
    state->linear_x = v; state->front_steer_angle = delta; state->front_wheel_speed = rpm;
    p.frontSteerDriveStuCallback(state);
  }
  static bool fresh(TebLocalPlannerROS & p, geometry_msgs::msg::Twist & v)
  {return p.getFreshFrontSteerVelocity(v);}
  static void expire(TebLocalPlannerROS & p)
  {p.front_steer_stu_receive_time_ -= std::chrono::seconds(1);}
  static bool publish(TebLocalPlannerROS & p, double v, double delta, double omega = 0.0)
  {return p.publishFrontSteerCommand(v, delta, omega);}
  static geometry_msgs::msg::Twist select(TebLocalPlannerROS & p,
    const geometry_msgs::msg::Twist & nav2_velocity)
  {p.selectFrontSteerVelocity(nav2_velocity); return p.robot_vel_;}
  static bool hasPublisher(TebLocalPlannerROS & p) {return bool(p.front_steer_cmd_pub_);}
  static bool hasCommandDebugPublishers(TebLocalPlannerROS & p)
  {return p.front_steer_cmd_linear_debug_pub_ || p.front_steer_cmd_angle_debug_pub_ ||
      p.front_steer_cmd_twist_debug_pub_;}
  static void advanceControlTime(TebLocalPlannerROS & p, double dt)
  {p.front_steer_last_publish_time_ = std::chrono::steady_clock::now() -
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));}
  static void project(TebLocalPlannerROS & p, double & v, double & w)
  {double vy = 0.0; p.applyFrontSteeringCommandLimits(v, vy, w);}
  static double projectedAngle(TebLocalPlannerROS & p) {return p.front_steering_delta_;}
  static void prepareCommand(TebLocalPlannerROS & p, double delta, double rate,
    double dt, double age, bool aligning)
  {
    p.front_steer_publish_valid_ = true;
    p.front_steer_published_angle_ = delta;
    p.front_steer_published_rate_ = rate;
    p.front_steer_last_publish_time_ = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(age));
    p.front_steer_control_dt_ = dt;
    p.front_steer_control_dt_prepared_ = true;
    p.front_steer_aligning_ = aligning;
  }
  static void linearHistory(TebLocalPlannerROS & p, double v) {p.front_steer_published_linear_ = v;}
  static geometry_msgs::msg::Twist actualCommand(TebLocalPlannerROS & p) {return p.last_cmd_;}
  static double previousAngle(TebLocalPlannerROS & p) {return p.front_steer_published_angle_;}
};
}
namespace
{
using namespace teb_local_planner;
using Access = FrontSteerInterfaceTestAccess;

void configureRobot(TebConfig::Robot & r)
{
  r.front_steering_rear_drive = true;
  r.cmd_angle_instead_rotvel = false;
  r.wheelbase = 0.854;
  r.max_steering_angle = 1.04719755;
  r.max_steering_rate = 0.5;
  r.max_steering_acceleration = 1.0;
  r.steering_min_speed = 0.03;
  r.max_vel_x = 0.5;
  r.max_vel_x_backwards = 0.3;
  r.max_vel_theta = 1.0;
  // Most existing cases isolate steering/state behavior. Ramp-specific cases
  // override this with the deployment value, 0.3 m/s^2.
  r.acc_lim_x = 1000.0;
}

geometry_msgs::msg::PoseStamped seedPose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw * 0.5); p.pose.orientation.w = std::cos(yaw * 0.5);
  return p;
}

TEST(FrontSteerInterface, InitializationSamplingPreservesEndpointsAndCutsDenseStraightSeed)
{
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  for (int i = 0; i <= 500; ++i) {plan.push_back(seedPose(i * 0.01, 0.0));}
  const auto sampled = sampleInitializationPlan(plan, 0.1, 0.1);
  EXPECT_LT(sampled.size(), 60u); EXPECT_GT(sampled.size(), 40u);
  EXPECT_EQ(sampled.front().pose.position.x, 0.0);
  EXPECT_EQ(sampled.back().pose.position.x, 5.0);
  EXPECT_EQ(sampleInitializationPlan(plan, 0.0, 0.1).size(), plan.size());
}

TEST(FrontSteerInterface, InitializationSamplingPreservesCornersCuspsAndStationaryTurns)
{
  const std::vector<geometry_msgs::msg::PoseStamped> plan = {
    seedPose(0, 0), seedPose(0.02, 0), seedPose(0.04, 0),
    seedPose(0.04, 0.02), seedPose(0.04, 0.04), seedPose(0.04, 0.04, 0.4),
    seedPose(0.02, 0.04, 0.4), seedPose(0, 0.04, 0.4), seedPose(0.02, 0.04, 0.4)};
  const auto sampled = sampleInitializationPlan(plan, 0.1, 0.1);
  const auto contains = [&](double x, double y, double yaw) {
      for (const auto & p : sampled) {
        if (p.pose.position.x == x && p.pose.position.y == y &&
          std::abs(tf2::getYaw(p.pose.orientation) - yaw) < 1e-9) {return true;}
      }
      return false;
    };
  EXPECT_TRUE(contains(0.04, 0, 0));
  EXPECT_TRUE(contains(0.04, 0.04, 0));
  EXPECT_TRUE(contains(0.04, 0.04, 0.4));
  EXPECT_TRUE(contains(0, 0.04, 0.4));
}

TEST(FrontSteerInterface, OdomConversion)
{
  TebConfig cfg; configureRobot(cfg.robot); cfg.robot.rear_track = 0.84;
  geometry_msgs::msg::Twist twist;
  capella_ros_msg::msg::FrontSteerDriveStu state;
  for (double v : {0.3, -0.3}) {
    twist.linear.x = v;
    twist.angular.z = v * std::tan(0.4) / cfg.robot.wheelbase;
    ASSERT_TRUE(odomToFrontSteerState(twist, cfg.robot, 0.1, state));
    EXPECT_NEAR(state.linear_x, v, 1e-7);
    EXPECT_NEAR(state.front_steer_angle, 0.4, 1e-7);
    EXPECT_EQ(state.front_wheel_speed, 0.0f); // odom does not measure steering-joint rate
  }
  twist = geometry_msgs::msg::Twist();
  ASSERT_TRUE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  EXPECT_EQ(state.front_wheel_speed, 0.0f);
  twist.linear.x = 0.01;
  ASSERT_TRUE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  EXPECT_EQ(state.front_steer_angle, 0.0f);
  EXPECT_EQ(state.linear_x, 0.0f);
  for (double sign : {-1.0, 1.0}) {
    twist.linear.x = sign * cfg.robot.steering_min_speed;
    twist.linear.y = -sign * 1e-4;
    twist.linear.z = sign * 1e-4;
    twist.angular.x = sign * 1e-4;
    twist.angular.y = -sign * 1e-4;
    twist.angular.z = sign * 0.01;
    state.linear_x = 1.0f; state.front_steer_angle = 0.4f; state.front_wheel_speed = 50.0f;
    ASSERT_TRUE(odomToFrontSteerState(twist, cfg.robot, 0.1, state));
    EXPECT_EQ(state.linear_x, 0.0f);
    EXPECT_EQ(state.front_steer_angle, 0.0f);
    EXPECT_EQ(state.front_wheel_speed, 0.0f);
  }
  twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.01;
  twist.angular.z = 0.011;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.1, state));
  ASSERT_TRUE(odomToFrontSteerState(twist, cfg.robot, 0.1, state, 0.02));
  EXPECT_EQ(state.linear_x, 0.0f);
  twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.1, state));
  twist.angular.z = 0.0;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.1, state, -0.01));
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.1, state,
    std::numeric_limits<double>::infinity()));
  twist.angular.z = 0.1;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  twist.linear.x = 0.3; twist.angular.z = 10.0;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  twist.angular.z = 0.0;
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()}) {
    twist.linear.x = bad;
    EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  }
  twist.linear.x = 0.3;
  twist.linear.y = 0.1;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  twist.linear.y = 0.0;
  twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  twist.angular.x = 0.0;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, -0.1, state));
  EXPECT_TRUE(odomToFrontSteerState(twist, cfg.robot, 1e-310, state));
  EXPECT_EQ(state.front_wheel_speed, 0.0f); // legacy wheel radius does not imply steering rate
  cfg.robot.wheelbase = 0.0;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
  cfg.robot.wheelbase = 0.854; cfg.robot.rear_track = -0.84;
  EXPECT_FALSE(odomToFrontSteerState(twist, cfg.robot, 0.0, state));
}

TEST(FrontSteerInterface, LoggedStartupAlignsThenDrivesWithoutRelaxingLimits)
{
  TebConfig cfg; configureRobot(cfg.robot);
  cfg.robot.max_vel_theta = 0.4;
  FrontSteeringCommandState previous;
  previous.valid = true;
  double measured = 0.0;
  bool drove = false;
  for (int i = 0; i < 40; ++i) {
    const double dt = i % 2 == 0 ? 0.067666 : 0.109986;
    FrontSteeringCommand command;
    bool aligning = false;
    ASSERT_EQ(projectFrontSteerDriveCommand(0.309243, 0.0, -0.352808, dt,
      cfg.robot, previous, measured, 0.2, command, aligning), nullptr);
    ASSERT_EQ(frontSteerCommandError(command.vx, command.steering_angle, command.omega,
      dt, cfg.robot, previous.steering_angle, previous.steering_rate, aligning), nullptr);
    if (i == 0) {EXPECT_TRUE(aligning);}
    if (aligning) {
      EXPECT_EQ(command.vx, 0.0); EXPECT_EQ(command.omega, 0.0);
      EXPECT_LT(command.steering_angle, 0.0);
    } else {
      EXPECT_NEAR(command.vx, 0.309243, 1e-9);
      EXPECT_LE(std::abs(command.omega + 0.352808), 0.2);
      drove = true;
    }
    measured = previous.steering_angle;  // delayed actuator feedback
    previous.steering_angle = command.steering_angle;
    previous.steering_rate = command.steering_rate;
  }
  EXPECT_TRUE(drove);
}

TEST(FrontSteerInterface, AlignmentWaitsForFeedbackAndStaticViolationStillFails)
{
  TebConfig cfg; configureRobot(cfg.robot);
  FrontSteeringCommandState previous;
  previous.valid = true;
  previous.steering_angle = std::atan(0.854 * -0.352808 / 0.309243);
  FrontSteeringCommand command;
  bool aligning = false;
  ASSERT_EQ(projectFrontSteerDriveCommand(0.309243, 0.0, -0.352808, 0.1,
    cfg.robot, previous, 0.0, 0.2, command, aligning), nullptr);
  EXPECT_TRUE(aligning); EXPECT_EQ(command.vx, 0.0);
  ASSERT_EQ(projectFrontSteerDriveCommand(0.309243, 0.0, -0.352808, 0.1,
    cfg.robot, previous, previous.steering_angle, 0.2, command, aligning), nullptr);
  EXPECT_FALSE(aligning); EXPECT_GT(command.vx, 0.0);
  EXPECT_NE(projectFrontSteerDriveCommand(0.01, 0.0, 0.4, 0.1,
    cfg.robot, previous, 0.0, 0.2, command, aligning), nullptr);
  EXPECT_NE(projectFrontSteerDriveCommand(0.3, 0.0, 1.0, 0.1,
    cfg.robot, previous, 0.0, 0.2, command, aligning), nullptr);
  EXPECT_NE(projectFrontSteerDriveCommand(0.3, 0.0, 0.0, 0.1,
    cfg.robot, previous, std::numeric_limits<double>::quiet_NaN(), 0.2, command, aligning), nullptr);
}

TEST(FrontSteerInterface, AngularVelocityCapReducesSpeedWithoutBreakingSteeringDynamics)
{
  TebConfig cfg; configureRobot(cfg.robot); cfg.robot.max_vel_theta = 0.4;
  struct Sample {double v; double dt; double delta; double rate;};
  for (const auto sample : {Sample{0.215791, 0.100442, -0.99, -0.3},
      Sample{0.237523, 0.093642, -0.945, -0.3}}) {
    for (double direction : {-1.0, 1.0}) {
      FrontSteeringCommandState previous;
      previous.valid = true; previous.steering_angle = sample.delta;
      previous.steering_rate = sample.rate;
      FrontSteeringCommand command;
      bool aligning = false;
      ASSERT_EQ(projectFrontSteerDriveCommand(direction * sample.v, 0.0, -direction * 0.4,
        sample.dt, cfg.robot, previous, previous.steering_angle, 0.2, command, aligning), nullptr);
      ASSERT_EQ(frontSteerCommandError(command.vx, command.steering_angle, command.omega,
        sample.dt, cfg.robot, previous.steering_angle, previous.steering_rate, aligning), nullptr);
      EXPECT_FALSE(aligning);
      EXPECT_LT(std::abs(command.vx), sample.v);
      EXPECT_GT(direction * command.vx, 0.0);
      EXPECT_LE(std::abs(command.omega), 0.4 + 1e-9);
      EXPECT_LE(std::abs(command.steering_rate - sample.rate), sample.dt + 1e-9);
    }
  }
}

TEST(FrontSteerInterface, SteeringBrakesAtBothLimitsAcrossSpeedAndDirectionChanges)
{
  TebConfig cfg; configureRobot(cfg.robot);
  cfg.robot.max_vel_theta = 0.4; cfg.robot.max_vel_x_backwards = 0.5;
  FrontSteeringCommandState previous; previous.valid = true;
  double peak = 0.0;
  int moving = 0;
  const double intervals[] = {0.04, 0.16, 0.07, 0.12};
  for (int i = 0; i < 600; ++i) {
    const double dt = intervals[i % 4];
    const double direction = (i / 90) % 2 ? -1.0 : 1.0;
    const double turn = (i / 120) % 2 ? -1.0 : 1.0;
    const double v = direction * (i % 3 == 0 ? 0.5 : 0.2);
    FrontSteeringCommand command;
    bool aligning = false;
    SCOPED_TRACE(i);
    ASSERT_EQ(projectFrontSteerDriveCommand(v, 0.0, direction * turn * 0.4, dt,
      cfg.robot, previous, previous.steering_angle, 0.2, command, aligning), nullptr);
    ASSERT_EQ(frontSteerCommandError(command.vx, command.steering_angle, command.omega, dt,
      cfg.robot, previous.steering_angle, previous.steering_rate, aligning), nullptr);
    EXPECT_LE(std::abs(command.steering_angle), cfg.robot.max_steering_angle + 1e-9);
    EXPECT_LE(std::abs(command.steering_rate), cfg.robot.max_steering_rate + 1e-9);
    const double margin = command.steering_rate >= 0.0 ?
      cfg.robot.max_steering_angle - command.steering_angle :
      cfg.robot.max_steering_angle + command.steering_angle;
    EXPECT_GE(margin + 1e-9,
      command.steering_rate * command.steering_rate / (2 * cfg.robot.max_steering_acceleration));
    if (command.vx != 0.0) {++moving;}
    peak = std::max(peak, std::abs(command.steering_angle));
    // Match the real float32 command transport and the published-history path.
    previous.steering_angle = static_cast<float>(command.steering_angle);
    previous.steering_rate = command.steering_rate;
  }
  EXPECT_GT(moving, 0);
  EXPECT_GT(peak, 0.9);
}

TEST(FrontSteerInterface, LongitudinalRampAndReversal)
{
  EXPECT_NEAR(limitFrontSteerLinearAcceleration(0.5, 0.0, 0.3, 0.1), 0.03, 1e-12);
  EXPECT_NEAR(limitFrontSteerLinearAcceleration(0.5, 0.03, 0.3, 0.1), 0.06, 1e-12);
  EXPECT_NEAR(limitFrontSteerLinearAcceleration(0.1, 0.3, 0.3, 0.1), 0.27, 1e-12);
  EXPECT_EQ(limitFrontSteerLinearAcceleration(-0.5, 0.02, 0.3, 0.1), 0.0);
  EXPECT_NEAR(limitFrontSteerLinearAcceleration(-0.5, 0.0, 0.3, 0.1), -0.03, 1e-12);
  EXPECT_TRUE(std::isnan(limitFrontSteerLinearAcceleration(0.5, 0.0, 0.0, 0.1)));
  EXPECT_TRUE(std::isnan(limitFrontSteerLinearAcceleration(0.5, 0.0, 0.3, -0.1)));
  double previous = 0.0;
  for (int i = 0; i < 300; ++i) {
    const double dt = i % 2 ? 0.04 : 0.13;
    const double target = (i / 40) % 2 ? -0.3 : 0.5;
    const double v = limitFrontSteerLinearAcceleration(target, previous, 0.3, dt);
    EXPECT_LE(std::abs(v - previous), 0.3 * dt + 1e-12);
    EXPECT_GE(v, -0.3); EXPECT_LE(v, 0.5);
    previous = v;
  }
}

TEST(FrontSteerInterface, LagScalingAvoidsStopGoWithoutIncreasingYawErrorBudget)
{
  TebConfig cfg; configureRobot(cfg.robot); cfg.robot.max_vel_theta = 0.4;
  FrontSteeringCommandState previous; previous.valid = true;
  double measured = 0.0;
  int old_stops = 0;
  for (int i = 0; i < 240; ++i) {
    const double omega = (i / 30) % 2 ? -0.3 : 0.3;
    FrontSteeringCommand old_command, command;
    bool old_aligning = false, aligning = false;
    double cap;
    ASSERT_EQ(projectFrontSteerDriveCommand(0.3, 0.0, omega, 0.1,
      cfg.robot, previous, measured, 0.2, old_command, old_aligning), nullptr);
    if (old_command.vx == 0.0) {++old_stops;}
    ASSERT_EQ(projectFrontSteerDriveCommand(0.3, 0.0, omega, 0.1,
      cfg.robot, previous, measured, 0.2, command, aligning, true, &cap), nullptr);
    ASSERT_FALSE(aligning);
    ASSERT_GT(command.vx, cfg.robot.steering_min_speed);
    EXPECT_LE(std::abs(command.vx), cap + 1e-9);
    const double reference_omega = command.vx * omega / 0.3;
    const double measured_omega = command.vx * std::tan(measured) / cfg.robot.wheelbase;
    EXPECT_LE(std::abs(reference_omega - command.omega), 0.2 + 1e-9);
    EXPECT_LE(std::abs(reference_omega - measured_omega), 0.2 + 1e-9);
    ASSERT_EQ(frontSteerCommandError(command.vx, command.steering_angle, command.omega,
      0.1, cfg.robot, previous.steering_angle, previous.steering_rate), nullptr);
    measured += clampDouble(command.steering_angle - measured, -0.02, 0.02);
    previous.steering_angle = static_cast<float>(command.steering_angle);
    previous.steering_rate = command.steering_rate;
  }
  EXPECT_GT(old_stops, 20);
  FrontSteeringCommand command; bool aligning;
  EXPECT_NE(projectFrontSteerDriveCommand(0.01, 0.0, 0.4, 0.1,
    cfg.robot, previous, measured, 0.2, command, aligning, true), nullptr);
  previous.steering_angle = 0.0; previous.steering_rate = 0.0;
  ASSERT_EQ(projectFrontSteerDriveCommand(0.3, 0.0, 0.3, 0.1,
    cfg.robot, previous, 0.0, 0.001, command, aligning, true), nullptr);
  EXPECT_TRUE(aligning); EXPECT_EQ(command.vx, 0.0);
}

TEST(FrontSteerInterface, StateConversionForwardReverseAndLowSpeed)
{
  TebConfig cfg; configureRobot(cfg.robot);
  capella_ros_msg::msg::FrontSteerDriveStu state;
  geometry_msgs::msg::Twist v;
  state.front_steer_angle = 0.4;
  for (float speed : {0.3f, -0.3f, 0.01f, 0.0f}) {
    state.linear_x = speed;
    ASSERT_TRUE(frontSteerStateToVelocity(state, 0.1, 0.5, cfg.robot, v));
    EXPECT_FLOAT_EQ(v.linear.x, speed);
    EXPECT_EQ(v.linear.y, 0.0);
    EXPECT_NEAR(v.angular.z, std::abs(speed) <= 0.03 ? 0.0 :
      speed * std::tan(state.front_steer_angle) / cfg.robot.wheelbase, 1e-7);
  }
}

TEST(FrontSteerInterface, RejectsExpiredInvalidStateAndInvalidWheelbase)
{
  TebConfig cfg; configureRobot(cfg.robot);
  capella_ros_msg::msg::FrontSteerDriveStu state;
  geometry_msgs::msg::Twist v;
  EXPECT_FALSE(frontSteerStateToVelocity(state, 0.501, 0.5, cfg.robot, v));
  EXPECT_FALSE(frontSteerStateToVelocity(state, -0.1, 0.5, cfg.robot, v));
  state.front_steer_angle = 1.1;
  EXPECT_FALSE(frontSteerStateToVelocity(state, 0.1, 0.5, cfg.robot, v));
  state.front_steer_angle = 0.0;
  state.front_wheel_speed = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(frontSteerStateToVelocity(state, 0.1, 0.5, cfg.robot, v));
  state.front_wheel_speed = 0.0;
  cfg.robot.wheelbase = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(frontSteerStateToVelocity(state, 0.1, 0.5, cfg.robot, v));
}

TEST(FrontSteerInterface, FinalCheckRejectsLimitsAndInconsistency)
{
  TebConfig cfg; configureRobot(cfg.robot);
  auto error = [&](double v, double delta, double omega, double dt = 0.2) {
    return frontSteerCommandError(v, delta, omega, dt, cfg.robot, 0.0, 0.0);
  };
  EXPECT_EQ(error(0.2, 0.0, 0.0), nullptr);
  EXPECT_EQ(error(-0.2, 0.0, 0.0), nullptr);
  EXPECT_NE(error(0.6, 0.0, 0.0), nullptr);
  EXPECT_NE(error(-0.4, 0.0, 0.0), nullptr);
  EXPECT_NE(error(0.2, 1.1, 0.0), nullptr);
  EXPECT_NE(error(0.2, 0.2, 0.0), nullptr); // rate
  EXPECT_NE(error(0.2, 0.06, 0.0), nullptr); // acceleration
  EXPECT_NE(error(0.2, 0.02, 0.2), nullptr); // omega mismatch
  EXPECT_NE(error(0.01, 0.1, 0.0), nullptr);
  EXPECT_NE(error(0.2, 0.0, 0.0, 0.0), nullptr);
  EXPECT_NE(error(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), nullptr);
  EXPECT_EQ(frontSteerCommandError(0, 0, 0, 0, cfg.robot, 1.0, 0.5), nullptr);
}

class FrontSteerRosInterface : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("front_steer_interface_test");
    observer = std::make_shared<rclcpp::Node>("front_steer_interface_observer");
    planner = std::make_unique<TebLocalPlannerROS>();
    configureRobot(Access::robot(*planner));
    sub = observer->create_subscription<capella_ros_msg::msg::FrontSteerDriveCmd>(
      "/teb_interface_test/cmd", rclcpp::SystemDefaultsQoS(),
      [this](capella_ros_msg::msg::FrontSteerDriveCmd::ConstSharedPtr msg) {
        last = *msg; ++received;
      });
  }
  void waitForCommand(int count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received < count && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(observer); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GE(received, count);
  }
  void pumpUntil(const std::function<bool()> & ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!ready() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(observer);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(ready());
  }
  void connect()
  {
    Access::initialize(*planner, node, "front_steer_cmd");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (node->count_subscribers("/teb_interface_test/cmd") == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  nav2_util::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr observer;
  std::unique_ptr<TebLocalPlannerROS> planner;
  rclcpp::Subscription<capella_ros_msg::msg::FrontSteerDriveCmd>::SharedPtr sub;
  capella_ros_msg::msg::FrontSteerDriveCmd last;
  int received{0};
};

TEST_F(FrontSteerRosInterface, SamplingReducesActualInitializedBand)
{
  TebConfig cfg; configureRobot(cfg.robot);
  cfg.optim.optimization_activate = false;  // inspect the seed before any auto-resize
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  for (int i = 0; i <= 500; ++i) {plan.push_back(seedPose(i * 0.01, 0.0));}
  TebOptimalPlanner dense(node, cfg);
  dense.plan(plan);
  EXPECT_EQ(dense.teb().sizePoses(), 501);
  cfg.trajectory.global_plan_init_min_spacing = 0.1;
  cfg.trajectory.global_plan_init_max_angle = 0.1;
  TebOptimalPlanner sampled(node, cfg);
  sampled.plan(plan);
  EXPECT_LT(sampled.teb().sizePoses(), 60);
  EXPECT_GT(sampled.teb().sizePoses(), 40);
  EXPECT_EQ(sampled.teb().BackPose().x(), 5.0);
}

TEST_F(FrontSteerRosInterface, OdomBridgeDisabledByDefault)
{
  connect();
  EXPECT_FALSE(Access::hasOdomBridge(*planner));
}

TEST_F(FrontSteerRosInterface, PublishesMeasuredSteeringRateConvertedFromRpm)
{
  connect();
  double measured_rate = 0.0;
  int count = 0;
  auto rate_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/front_steering/measured_steering_angular_velocity", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {
      measured_rate = msg->data;
      ++count;
    });
  pumpUntil([&]() {return rate_sub->get_publisher_count() > 0;});
  Access::receive(*planner, 0.2, 0.4, 60.0);
  pumpUntil([&]() {return count == 1;});
  EXPECT_NEAR(measured_rate, 2.0 * std::acos(-1.0), 1e-9);
  Access::receive(*planner, -0.2, -0.4, -30.0);
  pumpUntil([&]() {return count == 2;});
  EXPECT_NEAR(measured_rate, -std::acos(-1.0), 1e-9);
}

TEST_F(FrontSteerRosInterface, DebugLoggingCanBeChangedAtRuntime)
{
  connect();
  const std::string name = "FollowPath.front_steer_drive_interface.debug_logging";
  EXPECT_FALSE(node->get_parameter(name).as_bool());
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, true)).successful);
  EXPECT_TRUE(node->get_parameter(name).as_bool());
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(name, false)).successful);
  EXPECT_FALSE(node->get_parameter(name).as_bool());
}

TEST_F(FrontSteerRosInterface, OdomBridgePublishesOnlyValidInput)
{
  Access::robot(*planner).rear_track = 0.84;
  Access::enableOdom(*planner);
  connect();
  ASSERT_TRUE(Access::hasOdomBridge(*planner));
  int count = 0;
  capella_ros_msg::msg::FrontSteerDriveStu state;
  auto state_sub = observer->create_subscription<capella_ros_msg::msg::FrontSteerDriveStu>(
    "/teb_interface_test/state", rclcpp::SystemDefaultsQoS(),
    [&](capella_ros_msg::msg::FrontSteerDriveStu::ConstSharedPtr msg) {state = *msg; ++count;});
  auto odom_pub = observer->create_publisher<nav_msgs::msg::Odometry>(
    "/teb_interface_test/odom", rclcpp::SensorDataQoS());
  pumpUntil([&]() {return odom_pub->get_subscription_count() > 0 &&
      node->count_subscribers("/teb_interface_test/state") >= 2;});
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = node->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = -0.3;
  odom.twist.twist.angular.z = -0.1;
  odom_pub->publish(odom);
  pumpUntil([&]() {rclcpp::spin_some(node->get_node_base_interface()); return count == 1;});
  EXPECT_EQ(state.header.frame_id, "base_link");
  EXPECT_EQ(state.header.stamp, odom.header.stamp);
  EXPECT_FLOAT_EQ(state.linear_x, -0.3f);
  EXPECT_NEAR(state.front_steer_angle, std::atan(0.854 / 3.0), 1e-7);
  EXPECT_EQ(state.front_wheel_speed, 0.0f);
  odom.twist.twist.linear.x = 1e-4;
  odom.twist.twist.linear.y = -1e-4;
  odom.twist.twist.angular.z = -1e-3;
  odom_pub->publish(odom);
  pumpUntil([&]() {rclcpp::spin_some(node->get_node_base_interface()); return count == 2;});
  EXPECT_EQ(state.linear_x, 0.0f);
  EXPECT_EQ(state.front_steer_angle, 0.0f);
  EXPECT_EQ(state.front_wheel_speed, 0.0f);
  odom.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  odom_pub->publish(odom);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node->get_node_base_interface());
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(count, 2);
  planner->cleanup();
  EXPECT_FALSE(Access::hasOdomBridge(*planner));
}

TEST_F(FrontSteerRosInterface, LoggedStartupPublishesAlignmentSequenceThenMovingCommand)
{
  connect();
  Access::robot(*planner).max_vel_theta = 0.4;
  bool moved = false;
  for (int i = 0; i < 30; ++i) {
    Access::receive(*planner, 0.0, last.front_steer_angle);
    if (i > 0) {Access::advanceControlTime(*planner, i % 2 ? 0.067666 : 0.109986);}
    double v = 0.309243, w = -0.352808;
    ASSERT_NO_THROW(Access::project(*planner, v, w));
    ASSERT_TRUE(Access::publish(*planner, v, Access::projectedAngle(*planner), w));
    waitForCommand(i + 1);
    if (last.linear_x != 0.0f) {
      EXPECT_NEAR(last.linear_x, 0.309243, 1e-6);
      moved = true;
      break;
    }
    EXPECT_LT(last.front_steer_angle, 0.0f);
  }
  EXPECT_TRUE(moved);
}

TEST_F(FrontSteerRosInterface, IdleStopRestartsButMovingHistoryTimeoutStillFails)
{
  connect(); Access::receive(*planner, 0.0, 0.0);
  Access::prepareCommand(*planner, 0.0, 0.0, 0.1, 1.0, false);
  double v = 0.309243, w = -0.352808;
  ASSERT_NO_THROW(Access::project(*planner, v, w));
  EXPECT_EQ(v, 0.0); EXPECT_EQ(w, 0.0);
  EXPECT_LT(Access::projectedAngle(*planner), 0.0);
  EXPECT_TRUE(Access::publish(*planner, v, Access::projectedAngle(*planner), w));
  waitForCommand(1); EXPECT_LT(last.front_steer_angle, 0.0f);
  Access::prepareCommand(*planner, -0.1, -0.1, 0.1, 1.0, false);
  v = 0.3; w = -0.2;
  EXPECT_THROW(Access::project(*planner, v, w), nav2_core::PlannerException);
}

TEST_F(FrontSteerRosInterface, StationaryAlignmentPublishesAngleAndSafetyStopStillZeros)
{
  connect(); Access::receive(*planner, 0.0, 0.0);
  Access::prepareCommand(*planner, 0.0, 0.0, 0.1, 0.1, true);
  ASSERT_TRUE(Access::publish(*planner, 0.0, -0.01, 0.0));
  waitForCommand(1);
  EXPECT_EQ(last.linear_x, 0.0f); EXPECT_FLOAT_EQ(last.front_steer_angle, -0.01f);
  EXPECT_NEAR(Access::previousAngle(*planner), -0.01, 1e-7);
  Access::expire(*planner);
  Access::prepareCommand(*planner, -0.01, -0.1, 0.1, 0.1, true);
  EXPECT_FALSE(Access::publish(*planner, 0.0, -0.03, 0.0));
  waitForCommand(2); EXPECT_EQ(last.front_steer_angle, 0.0f);
  EXPECT_TRUE(Access::publish(*planner, 0.0, 0.0));
  waitForCommand(3); EXPECT_EQ(last.front_steer_angle, 0.0f);
}

TEST_F(FrontSteerRosInterface, FinalValidationUsesProjectionIntervalDespitePublicationDelay)
{
  connect(); Access::receive(*planner, 0.2, 0.1);
  Access::prepareCommand(*planner, 0.1, 0.4, 0.1, 0.2, false);
  const double delta = 0.13;
  EXPECT_TRUE(Access::publish(*planner, 0.2, delta, 0.2 * std::tan(delta) / 0.854));
  waitForCommand(1); EXPECT_FLOAT_EQ(last.front_steer_angle, delta);
  // The interval being frozen must never bypass a true command/feedback timeout.
  Access::prepareCommand(*planner, 0.13, 0.3, 0.1, 0.6, false);
  EXPECT_FALSE(Access::publish(*planner, 0.2, 0.15, 0.2 * std::tan(0.15) / 0.854));
  waitForCommand(2); EXPECT_EQ(last.linear_x, 0.0f);
}

TEST_F(FrontSteerRosInterface, LinearRampCrossesLowSpeedAndRestartsFromStop)
{
  connect(); Access::robot(*planner).acc_lim_x = 0.3;
  for (int i = 0; i < 10; ++i) {
    Access::receive(*planner, last.linear_x, 0.0);
    Access::prepareCommand(*planner, 0.0, 0.0, 0.1, 0.1, false);
    ASSERT_TRUE(Access::publish(*planner, 0.3, 0.0));
    waitForCommand(i + 1);
    EXPECT_NEAR(last.linear_x, 0.03 * (i + 1), 1e-6);
    EXPECT_DOUBLE_EQ(Access::actualCommand(*planner).linear.x, last.linear_x);
  }
  ASSERT_TRUE(Access::publish(*planner, 0.0, 0.0));
  waitForCommand(11); EXPECT_EQ(last.linear_x, 0.0f);
  Access::receive(*planner, 0.0, 0.0);
  Access::prepareCommand(*planner, 0.0, 0.0, 0.1, 0.1, false);
  ASSERT_TRUE(Access::publish(*planner, 0.3, 0.0));
  waitForCommand(12); EXPECT_NEAR(last.linear_x, 0.03, 1e-6);
}

TEST_F(FrontSteerRosInterface, LinearRampRetainsSteeringAndConsistentTwistBelowMinSpeed)
{
  connect(); Access::robot(*planner).acc_lim_x = 0.3;
  Access::receive(*planner, 0.0, 0.0);
  Access::prepareCommand(*planner, 0.0, 0.0, 0.1, 0.1, false);
  const double angle = 0.01;
  ASSERT_TRUE(Access::publish(*planner, 0.3, angle, 0.3 * std::tan(angle) / 0.854));
  waitForCommand(1);
  EXPECT_NEAR(last.linear_x, 0.03, 1e-6);
  EXPECT_FLOAT_EQ(last.front_steer_angle, angle);
  const auto actual = Access::actualCommand(*planner);
  EXPECT_DOUBLE_EQ(actual.linear.x, last.linear_x);
  EXPECT_NEAR(actual.angular.z,
    double(last.linear_x) * std::tan(double(last.front_steer_angle)) / 0.854, 1e-12);
  // A larger projection interval cannot allocate extra acceleration when the
  // actual messages are only 0.1 s apart (0.03 -> about 0.06, not 0.09).
  Access::prepareCommand(*planner, angle, 0.1, 0.2, 0.1, false);
  ASSERT_TRUE(Access::publish(*planner, 0.3, 0.02, 0.3 * std::tan(0.02) / 0.854));
  waitForCommand(2);
  EXPECT_GE(last.linear_x, 0.059f); EXPECT_LT(last.linear_x, 0.075f);
}

TEST_F(FrontSteerRosInterface, YawLimitAndStaleFeedbackOverrideLinearDecelerationRamp)
{
  connect(); Access::robot(*planner).acc_lim_x = 0.3;
  Access::robot(*planner).max_vel_theta = 0.4;
  Access::receive(*planner, 0.2, 1.0);
  Access::prepareCommand(*planner, 1.0, 0.0, 0.1, 0.1, false);
  Access::linearHistory(*planner, 0.5);
  ASSERT_TRUE(Access::publish(*planner, 0.2, 1.0, 0.2 * std::tan(1.0) / 0.854));
  waitForCommand(1);
  EXPECT_LE(last.linear_x, 0.4 * 0.854 / std::tan(1.0) + 1e-6);
  EXPECT_LE(std::abs(Access::actualCommand(*planner).angular.z), 0.4 + 1e-6);
  Access::expire(*planner);
  EXPECT_FALSE(Access::publish(*planner, 0.2, 1.0, 0.2 * std::tan(1.0) / 0.854));
  waitForCommand(2); EXPECT_EQ(last.linear_x, 0.0f);
}

TEST_F(FrontSteerRosInterface, LagScalingKeepsWireSpeedNonzeroAcrossChangingAvoidanceTargets)
{
  connect(); Access::enableLagScaling(*planner);
  Access::robot(*planner).acc_lim_x = 0.3; Access::robot(*planner).max_vel_theta = 0.4;
  double measured = 0.0;
  for (int i = 0; i < 30; ++i) {
    measured += clampDouble(double(last.front_steer_angle) - measured, -0.01, 0.01);
    Access::receive(*planner, last.linear_x, measured);
    if (i > 0) {Access::advanceControlTime(*planner, 0.1);}
    double v = 0.3, w = (i / 10) % 2 ? -0.3 : 0.3;
    ASSERT_NO_THROW(Access::project(*planner, v, w));
    const double cap = Access::lagCap(*planner);
    ASSERT_TRUE(Access::publish(*planner, v, Access::projectedAngle(*planner), w));
    waitForCommand(i + 1);
    EXPECT_GT(last.linear_x, 0.0f);
    EXPECT_LE(last.linear_x, cap + 1e-6);
  }
  Access::expire(*planner);
  Access::prepareCommand(*planner, last.front_steer_angle, 0.0, 0.1, 0.1, false);
  EXPECT_FALSE(Access::publish(*planner, 0.2, last.front_steer_angle,
    0.2 * std::tan(last.front_steer_angle) / 0.854));
  waitForCommand(31); EXPECT_EQ(last.linear_x, 0.0f);
}

TEST_F(FrontSteerRosInterface, MissingExpiredAndInvalidFeedbackStopThenRecover)
{
  connect();
  EXPECT_FALSE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(1); EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
  Access::receive(*planner, 0.1, 0.0);
  EXPECT_TRUE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(2); EXPECT_FLOAT_EQ(last.linear_x, 0.2f);
  Access::expire(*planner);
  EXPECT_FALSE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(3); EXPECT_EQ(last.linear_x, 0.0f);
  Access::receive(*planner, std::numeric_limits<float>::quiet_NaN(), 0);
  EXPECT_FALSE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(4); EXPECT_EQ(last.linear_x, 0.0f);
  Access::receive(*planner, 0.1, 0.0);
  EXPECT_TRUE(Access::publish(*planner, -0.2, 0.0));
  waitForCommand(5); EXPECT_FLOAT_EQ(last.linear_x, -0.2f);
}

TEST_F(FrontSteerRosInterface, MapsProjectedAngleAndResetsHistoryOnStop)
{
  connect(); Access::receive(*planner, 0.1, 0.0);
  const double delta = 0.02;
  EXPECT_TRUE(Access::publish(*planner, -0.2, delta, -0.2 * std::tan(delta) / 0.854));
  waitForCommand(1); EXPECT_FLOAT_EQ(last.front_steer_angle, delta);
  EXPECT_TRUE(Access::publish(*planner, 0.0, 0.0));
  waitForCommand(2); EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
  EXPECT_EQ(Access::previousAngle(*planner), 0.0);
  EXPECT_FALSE(Access::publish(*planner, 0.2, std::numeric_limits<double>::infinity()));
  waitForCommand(3); EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
}

TEST_F(FrontSteerRosInterface, LegacyModeSubscribesWithoutPublishingAndFeedbackCanRecover)
{
  Access::initialize(*planner, node, "cmd_vel");
  EXPECT_FALSE(Access::hasPublisher(*planner));
  geometry_msgs::msg::Twist v;
  EXPECT_FALSE(Access::fresh(*planner, v));
  geometry_msgs::msg::Twist nav2_velocity;
  nav2_velocity.linear.x = 0.4;
  EXPECT_DOUBLE_EQ(Access::select(*planner, nav2_velocity).linear.x, 0.4);
  Access::receive(*planner, -0.2, 0.3);
  EXPECT_NEAR(Access::select(*planner, nav2_velocity).linear.x, -0.2, 1e-7);
  ASSERT_TRUE(Access::fresh(*planner, v)); EXPECT_LT(v.angular.z, 0.0);
  Access::expire(*planner); EXPECT_FALSE(Access::fresh(*planner, v));
  EXPECT_DOUBLE_EQ(Access::select(*planner, nav2_velocity).linear.x, 0.4);
  Access::receive(*planner, 0.2, -0.3); EXPECT_TRUE(Access::fresh(*planner, v));
}


TEST_F(FrontSteerRosInterface, DirectModeNeverFallsBackAndCleanupPublishesStop)
{
  connect();
  geometry_msgs::msg::Twist nav2_velocity;
  nav2_velocity.linear.x = 0.4;
  EXPECT_THROW(Access::select(*planner, nav2_velocity), nav2_core::PlannerException);
  Access::receive(*planner, 0.2, 0.0);
  EXPECT_NEAR(Access::select(*planner, nav2_velocity).linear.x, 0.2, 1e-7);
  EXPECT_TRUE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(1);
  planner->cleanup();
  waitForCommand(2);
  EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
  EXPECT_FALSE(Access::hasPublisher(*planner));
}

TEST_F(FrontSteerRosInterface, RuntimeModeChangeIsRejected)
{
  node->declare_parameter("FollowPath.front_steer_drive_interface.mode", "cmd_vel");
  Access::initialize(*planner, node, "cmd_vel");
  const auto result = node->set_parameter(
    rclcpp::Parameter("FollowPath.front_steer_drive_interface.mode", "front_steer_cmd"));
  EXPECT_FALSE(result.successful);
  EXPECT_FALSE(Access::hasPublisher(*planner));
}

TEST_F(FrontSteerRosInterface, ReceivesStateOverRosAndCanonicalizesLowSpeed)
{
  connect();
  auto publisher = observer->create_publisher<capella_ros_msg::msg::FrontSteerDriveStu>(
    "/teb_interface_test/state", rclcpp::SystemDefaultsQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  capella_ros_msg::msg::FrontSteerDriveStu state;
  state.header.stamp = node->now();
  state.header.frame_id = "base_link";
  state.linear_x = 0.15; state.front_steer_angle = 0.2; state.front_wheel_speed = 100.0;
  geometry_msgs::msg::Twist v;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!Access::fresh(*planner, v) && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(state);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(Access::fresh(*planner, v));
  EXPECT_NEAR(v.linear.x, 0.15, 1e-7);
  EXPECT_TRUE(Access::publish(*planner, 0.01, 0.0));
  waitForCommand(1);
  EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
}

TEST_F(FrontSteerRosInterface, UninitializedControllerFailureStillPublishesStop)
{
  connect(); Access::receive(*planner, 0.1, 0.0);
  EXPECT_TRUE(Access::publish(*planner, 0.2, 0.0));
  waitForCommand(1);
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist velocity;
  EXPECT_THROW(planner->computeVelocityCommands(pose, velocity, nullptr),
    nav2_core::PlannerException);
  waitForCommand(2);
  EXPECT_EQ(last.linear_x, 0.0f); EXPECT_EQ(last.front_steer_angle, 0.0f);
}

TEST_F(FrontSteerRosInterface, CommandDebugMatchesActualReverseCommandAndSafetyStops)
{
  std_msgs::msg::Float64 linear, angle;
  geometry_msgs::msg::Twist twist;
  int linear_count = 0, angle_count = 0, twist_count = 0;
  auto linear_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/teb_interface_test/cmd/debug/linear_x", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {linear = *msg; ++linear_count;});
  auto angle_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/teb_interface_test/cmd/debug/front_steer_angle", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {angle = *msg; ++angle_count;});
  auto twist_sub = observer->create_subscription<geometry_msgs::msg::Twist>(
    "/teb_interface_test/cmd/debug/twist", rclcpp::SystemDefaultsQoS(),
    [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) {twist = *msg; ++twist_count;});
  connect();
  pumpUntil([&]() {
    return node->count_subscribers("/teb_interface_test/cmd/debug/linear_x") > 0 &&
      node->count_subscribers("/teb_interface_test/cmd/debug/front_steer_angle") > 0 &&
      node->count_subscribers("/teb_interface_test/cmd/debug/twist") > 0;
  });
  Access::receive(*planner, -0.2, 0.02);
  ASSERT_TRUE(Access::publish(*planner, -0.2, 0.02, -0.2 * std::tan(0.02) / 0.854));
  pumpUntil([&]() {return received >= 1 && linear_count >= 1 && angle_count >= 1 && twist_count >= 1;});
  EXPECT_DOUBLE_EQ(linear.data, last.linear_x);
  EXPECT_DOUBLE_EQ(angle.data, last.front_steer_angle);
  EXPECT_DOUBLE_EQ(twist.linear.x, last.linear_x);
  EXPECT_EQ(twist.linear.y, 0.0);
  EXPECT_NEAR(twist.angular.z,
    double(last.linear_x) * std::tan(double(last.front_steer_angle)) / 0.854, 1e-12);
  EXPECT_LT(twist.angular.z, 0.0);
  EXPECT_FALSE(Access::publish(*planner, 0.2, std::numeric_limits<double>::infinity()));
  pumpUntil([&]() {return received >= 2 && linear_count >= 2 && angle_count >= 2 && twist_count >= 2;});
  EXPECT_EQ(linear.data, 0.0); EXPECT_EQ(angle.data, 0.0);
  EXPECT_EQ(twist.linear.x, 0.0); EXPECT_EQ(twist.angular.z, 0.0);
  planner->cleanup();
  pumpUntil([&]() {return received >= 3 && linear_count >= 3 && angle_count >= 3 && twist_count >= 3;});
  EXPECT_EQ(linear.data, 0.0); EXPECT_EQ(angle.data, 0.0);
  EXPECT_EQ(twist.linear.x, 0.0); EXPECT_EQ(twist.angular.z, 0.0);
  EXPECT_FALSE(Access::hasCommandDebugPublishers(*planner));
}

TEST_F(FrontSteerRosInterface, FeedbackDebugMirrorsRawFieldsInLegacyMode)
{
  std_msgs::msg::Float64 linear, angle, wheel;
  int linear_count = 0, angle_count = 0, wheel_count = 0;
  auto linear_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/teb_interface_test/state/debug/linear_x", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {linear = *msg; ++linear_count;});
  auto angle_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/teb_interface_test/state/debug/front_steer_angle", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {angle = *msg; ++angle_count;});
  auto wheel_sub = observer->create_subscription<std_msgs::msg::Float64>(
    "/teb_interface_test/state/debug/front_wheel_speed", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {wheel = *msg; ++wheel_count;});
  Access::initialize(*planner, node, "cmd_vel");
  EXPECT_FALSE(Access::hasCommandDebugPublishers(*planner));
  pumpUntil([&]() {
    return node->count_subscribers("/teb_interface_test/state/debug/linear_x") > 0 &&
      node->count_subscribers("/teb_interface_test/state/debug/front_steer_angle") > 0 &&
      node->count_subscribers("/teb_interface_test/state/debug/front_wheel_speed") > 0;
  });
  Access::receive(*planner, -0.2, 0.3, -120.5);
  pumpUntil([&]() {return linear_count >= 1 && angle_count >= 1 && wheel_count >= 1;});
  EXPECT_DOUBLE_EQ(linear.data, double(float(-0.2)));
  EXPECT_DOUBLE_EQ(angle.data, double(float(0.3)));
  EXPECT_DOUBLE_EQ(wheel.data, -120.5);
  Access::receive(*planner, std::numeric_limits<double>::quiet_NaN(), 0.0, 100.0);
  pumpUntil([&]() {return linear_count >= 2 && angle_count >= 2 && wheel_count >= 2;});
  EXPECT_TRUE(std::isnan(linear.data));
  EXPECT_EQ(angle.data, 0.0); EXPECT_EQ(wheel.data, 100.0);
  geometry_msgs::msg::Twist v;
  EXPECT_FALSE(Access::fresh(*planner, v));
}

TEST_F(FrontSteerRosInterface, ConfigureRejectsUnknownModeAndInvalidModel)
{
  EXPECT_THROW(Access::initialize(*planner, node, "invalid"), std::invalid_argument);
  Access::robot(*planner).wheelbase = 0.0;
  EXPECT_THROW(Access::initialize(*planner, node, "front_steer_cmd"), std::invalid_argument);
}
}  // namespace
