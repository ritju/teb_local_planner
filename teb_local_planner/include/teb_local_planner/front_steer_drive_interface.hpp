#ifndef TEB_FRONT_STEER_DRIVE_INTERFACE_HPP_
#define TEB_FRONT_STEER_DRIVE_INTERFACE_HPP_

#include "teb_local_planner/front_steering_model.hpp"
#include "capella_ros_msg/msg/front_steer_drive_stu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <limits>

namespace teb_local_planner
{
// The odometry twist must describe the rear axle center in vehicle axes.
// A zero radius explicitly means wheel RPM is unavailable (diagnostic zero).
inline bool odomToFrontSteerState(
  const geometry_msgs::msg::Twist & twist, const TebConfig::Robot & robot,
  double wheel_radius, capella_ros_msg::msg::FrontSteerDriveStu & state,
  double angular_deadband = 0.01)
{
  state = capella_ros_msg::msg::FrontSteerDriveStu();
  const double v = twist.linear.x;
  const double w = twist.angular.z;
  const double limit = std::numeric_limits<float>::max();
  if (!std::isfinite(robot.wheelbase) || robot.wheelbase <= 0.0 ||
      !std::isfinite(robot.rear_track) || robot.rear_track <= 0.0 ||
      !std::isfinite(robot.max_steering_angle) || robot.max_steering_angle <= 0.0 ||
      robot.max_steering_angle >= 1.5 ||
      !std::isfinite(robot.steering_min_speed) || robot.steering_min_speed < 0.0 ||
      !std::isfinite(wheel_radius) || wheel_radius < 0.0 ||
      !std::isfinite(angular_deadband) || angular_deadband < 0.0 ||
      !std::isfinite(v) || !std::isfinite(w) ||
      !std::isfinite(twist.linear.y) || !std::isfinite(twist.linear.z) ||
      !std::isfinite(twist.angular.x) || !std::isfinite(twist.angular.y) ||
      std::abs(v) > limit) {
    return false;
  }
  // Publish fresh standstill feedback instead of timing out on odometry noise.
  // Check every component and all finite/configuration guards before zeroing.
  if (std::abs(v) <= robot.steering_min_speed &&
      std::abs(twist.linear.y) <= robot.steering_min_speed &&
      std::abs(twist.linear.z) <= robot.steering_min_speed &&
      std::abs(w) <= angular_deadband &&
      std::abs(twist.angular.x) <= angular_deadband &&
      std::abs(twist.angular.y) <= angular_deadband) {
    return true;  // motion fields were initialized to 0; caller fills header.
  }
  if (std::abs(twist.linear.y) > 1e-6 || std::abs(twist.linear.z) > 1e-6 ||
      std::abs(twist.angular.x) > 1e-6 || std::abs(twist.angular.y) > 1e-6) {
    return false;
  }
  // Do not disguise a differential-drive spin as valid Ackermann feedback.
  if (std::abs(v) <= robot.steering_min_speed) {return false;}
  const double delta = std::atan(robot.wheelbase * (w / v));
  const double left = v - w * robot.rear_track / 2.0;
  const double right = v + w * robot.rear_track / 2.0;
  const double rpm = wheel_radius == 0.0 ? 0.0 :
    (v / std::cos(delta)) / wheel_radius * (60.0 / (2.0 * std::acos(-1.0)));
  if (!std::isfinite(delta) || std::abs(delta) > robot.max_steering_angle ||
      !std::isfinite(left) || !std::isfinite(right) ||
      std::abs(left) > limit || std::abs(right) > limit ||
      !std::isfinite(rpm) || std::abs(rpm) > limit) {
    return false;
  }
  state.linear_x = static_cast<float>(v);
  state.front_steer_angle = static_cast<float>(delta);
  state.front_wheel_speed = static_cast<float>(rpm);
  return true;
}

inline bool validFrontSteerConfiguration(const TebConfig::Robot & robot)
{
  return std::isfinite(robot.wheelbase) && robot.wheelbase > 0.0 &&
    std::isfinite(robot.max_steering_angle) && robot.max_steering_angle > 0.0 &&
    robot.max_steering_angle < 1.5 &&
    std::isfinite(robot.max_steering_rate) && robot.max_steering_rate > 0.0 &&
    std::isfinite(robot.max_steering_acceleration) && robot.max_steering_acceleration > 0.0 &&
    std::isfinite(robot.steering_min_speed) && robot.steering_min_speed >= 0.0 &&
    std::isfinite(robot.max_vel_x) && robot.max_vel_x >= 0.0 &&
    std::isfinite(robot.max_vel_x_backwards) && robot.max_vel_x_backwards >= 0.0 &&
    std::isfinite(robot.max_vel_theta) && robot.max_vel_theta >= 0.0 &&
    std::isfinite(robot.acc_lim_x) && robot.acc_lim_x > 0.0;
}

// Nominal longitudinal slew. Stops and tighter safety limits take priority at
// the publisher. Direction reversal first ramps to zero, then starts backwards.
inline double limitFrontSteerLinearAcceleration(
  double target, double previous, double acceleration, double dt)
{
  if (!std::isfinite(target) || !std::isfinite(previous) ||
      !std::isfinite(acceleration) || acceleration <= 0.0 || !std::isfinite(dt) || dt <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (target * previous < 0.0) {target = 0.0;}
  const double step = acceleration * dt;
  return clampDouble(target, previous - step, previous + step);
}

// Static infeasibility remains a fault. Steering actuator lag is handled by
// commanding zero translation until both the command and measured angle can
// follow the requested yaw rate within the same projection tolerance.
inline const char * projectFrontSteerDriveCommand(
  double v, double vy, double omega, double dt, const TebConfig::Robot & robot,
  const FrontSteeringCommandState & previous, double measured_angle, double tolerance,
  FrontSteeringCommand & command, bool & aligning,
  bool scale_speed_for_lag = false, double * lag_speed_limit = nullptr)
{
  aligning = false;
  if (lag_speed_limit) {*lag_speed_limit = std::numeric_limits<double>::infinity();}
  command = FrontSteeringCommand();
  if (!validFrontSteerConfiguration(robot) || !std::isfinite(v) || !std::isfinite(vy) ||
      !std::isfinite(omega) || !std::isfinite(dt) || dt <= 1e-4 ||
      !std::isfinite(measured_angle) || std::abs(measured_angle) > robot.max_steering_angle + 1e-6 ||
      !std::isfinite(tolerance) || tolerance < 0.0 ||
      (previous.valid && (!std::isfinite(previous.steering_angle) ||
      !std::isfinite(previous.steering_rate)))) {
    return "Invalid front-steer projection input";
  }
  const auto attainable = projectFrontSteeringCommand(
    v, vy, omega, dt, robot, FrontSteeringCommandState());
  if (std::abs(omega - attainable.omega) > tolerance) {
    return "Front-steer projection exceeds max_projection_omega_error";
  }
  command = attainable;
  if (previous.valid) {
    const double angle_limit = robot.max_steering_angle;
    const double rate_limit = robot.max_steering_rate;
    const double acceleration = robot.max_steering_acceleration;
    const double angle = previous.steering_angle;
    if (std::abs(angle) > angle_limit + 1e-6 ||
        std::abs(previous.steering_rate) > rate_limit + 1e-6) {
      return "Invalid front-steer command history";
    }
    // Choose a rate that leaves enough angle to brake to zero afterwards:
    // rate*dt + rate^2/(2*a) <= remaining_angle. Rationalize the
    // quadratic root to avoid cancellation near the target/angle boundary.
    const auto braking_rate = [acceleration, dt](double distance) {
        distance = std::max(0.0, distance);
        const double step = acceleration * dt;
        return 2.0 * acceleration * distance /
          (std::sqrt(step * step + 2.0 * acceleration * distance) + step);
      };
    const double rate_lo = std::max(-rate_limit,
      std::max(previous.steering_rate - acceleration * dt,
        -braking_rate(angle_limit + angle)));
    const double rate_hi = std::min(rate_limit,
      std::min(previous.steering_rate + acceleration * dt,
        braking_rate(angle_limit - angle)));
    // A history already outside the braking envelope cannot satisfy all
    // constraints. Keep the fault/stop path rather than hide that violation.
    if (rate_lo > rate_hi + 1e-6) {
      return "Front-steer history cannot brake within steering limits";
    }
    const double error = attainable.steering_angle - angle;
    const double desired_rate = std::copysign(braking_rate(std::abs(error)), error);
    const double rate = rate_lo > rate_hi ? 0.5 * (rate_lo + rate_hi) :
      clampDouble(desired_rate, rate_lo, rate_hi);
    command.steering_angle = clampDouble(angle + rate * dt, -angle_limit, angle_limit);
    command.steering_rate = (command.steering_angle - angle) / dt;
  }
  command.vx = v;
  command.vy = 0.0;
  command.omega = std::abs(v) <= robot.steering_min_speed ? 0.0 :
    v * std::tan(command.steering_angle) / robot.wheelbase;
  // Do not re-clamp omega and invert it back into an abrupt steering step.
  // Preserve steering dynamics and lower translation to satisfy the yaw limit.
  if (std::abs(command.omega) > robot.max_vel_theta) {
    command.vx *= robot.max_vel_theta / std::abs(command.omega);
    command.omega = command.vx * std::tan(command.steering_angle) / robot.wheelbase;
  }
  command.rear_left = command.vx - 0.5 * robot.rear_track * command.omega;
  command.rear_right = command.vx + 0.5 * robot.rear_track * command.omega;
  const double measured_curvature = std::tan(measured_angle) / robot.wheelbase;
  const double measured_omega = v * measured_curvature;
  if (scale_speed_for_lag && std::abs(v) > robot.steering_min_speed) {
    const double requested_curvature = omega / v;
    const double command_curvature = std::tan(command.steering_angle) / robot.wheelbase;
    const double curvature_error = std::max(std::abs(requested_curvature - command_curvature),
      std::abs(requested_curvature - measured_curvature));
    const double cap = curvature_error > 1e-12 ? tolerance / curvature_error :
      std::numeric_limits<double>::infinity();
    if (lag_speed_limit) {*lag_speed_limit = cap;}
    command.vx = std::copysign(std::min(std::abs(command.vx), cap), command.vx);
    command.omega = command.vx * command_curvature;
    // Preserve the same yaw-error budget at the speed actually allowed to be
    // emitted. Only use a full standstill when even creep is below the gate.
    aligning = std::abs(command.vx) <= robot.steering_min_speed;
  } else if (std::abs(v) > robot.steering_min_speed &&
      (std::abs(omega - command.omega) > tolerance ||
      std::abs(omega - measured_omega) > tolerance ||
      std::abs(command.vx) <= robot.steering_min_speed)) {
    aligning = true;
  }
  if (aligning) {command.vx = command.vy = command.omega = 0.0;}
  command.rear_left = command.vx - 0.5 * robot.rear_track * command.omega;
  command.rear_right = command.vx + 0.5 * robot.rear_track * command.omega;
  return nullptr;
}

inline bool frontSteerStateToVelocity(
  const capella_ros_msg::msg::FrontSteerDriveStu & state, double age, double timeout,
  const TebConfig::Robot & robot, geometry_msgs::msg::Twist & velocity)
{
  velocity = geometry_msgs::msg::Twist();
  if (!validFrontSteerConfiguration(robot) || !std::isfinite(age) || age < 0.0 ||
      !std::isfinite(timeout) || timeout <= 0.0 || age > timeout ||
      !std::isfinite(state.linear_x) || !std::isfinite(state.front_steer_angle) ||
      !std::isfinite(state.front_wheel_speed) ||
      std::abs(state.front_steer_angle) > robot.max_steering_angle + 1e-6) {
    return false;
  }
  velocity.linear.x = state.linear_x;
  velocity.angular.z = std::abs(state.linear_x) <= robot.steering_min_speed ? 0.0 :
    state.linear_x * std::tan(state.front_steer_angle) / robot.wheelbase;
  return std::isfinite(velocity.angular.z);
}

// Return a stable diagnostic string, or nullptr for an acceptable command.
// Explicit stops override steering slew limits so every failure can send {0, 0}.
inline const char * frontSteerCommandError(
  double v, double delta, double omega, double dt, const TebConfig::Robot & robot,
  double previous_delta, double previous_rate, bool allow_low_speed_steering = false)
{
  constexpr double epsilon = 1e-6;
  if (!std::isfinite(v) || !std::isfinite(delta) || !std::isfinite(omega) ||
      std::abs(v) > std::numeric_limits<float>::max()) {return "non-finite command";}
  if (v == 0.0 && delta == 0.0 && omega == 0.0) {return nullptr;}
  if (!validFrontSteerConfiguration(robot)) {return "invalid robot configuration";}
  if (std::abs(v) <= robot.steering_min_speed) {
    if (delta == 0.0 && omega == 0.0) {return nullptr;}
    if (!allow_low_speed_steering || (v == 0.0 && omega != 0.0)) {
      return "steering or rotation at low speed";
    }
  }
  if (!std::isfinite(dt) || dt <= 1e-4 || !std::isfinite(previous_delta) ||
      !std::isfinite(previous_rate)) {return "invalid control interval or history";}
  if (v > robot.max_vel_x + epsilon || v < -robot.max_vel_x_backwards - epsilon) {
    return "linear speed limit";
  }
  if (std::abs(delta) > robot.max_steering_angle + epsilon) {return "steering angle limit";}
  const double rate = (delta - previous_delta) / dt;
  if (std::abs(rate) > robot.max_steering_rate + epsilon) {return "steering rate limit";}
  if (std::abs(rate - previous_rate) > robot.max_steering_acceleration * dt + epsilon) {
    return "steering acceleration limit";
  }
  const double curvature = std::tan(delta) / robot.wheelbase;
  if (!std::isfinite(curvature) || std::abs(v * curvature - omega) > epsilon) {
    return "v/omega/steering mismatch";
  }
  if (std::abs(omega) > robot.max_vel_theta + epsilon) {return "angular speed limit";}
  return nullptr;
}
}  // namespace teb_local_planner
#endif
