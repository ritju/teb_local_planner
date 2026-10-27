#ifndef EDGE_FRONT_STEERING_H_
#define EDGE_FRONT_STEERING_H_

#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"

namespace teb_local_planner
{

inline double steeringPenaltyEpsilon(double bound, double epsilon)
{
  const double width = std::abs(bound);
  if (!(epsilon > 0.0) || !(width > epsilon))
  {
    return 0.0;
  }
  return epsilon;
}

inline bool segmentFromVertices(
  const VertexPose * pose1, const VertexPose * pose2, const VertexTimeDiff * dt, const TebConfig * cfg,
  SegmentKinematics & segment)
{
  if (!pose1 || !pose2 || !dt || !cfg)
  {
    return false;
  }
  return computeSteeringSegment(
    pose1->pose(), pose2->pose(), dt->dt(), cfg->robot.wheelbase, cfg->robot.steering_min_speed,
    cfg->robot.steering_omega_epsilon, segment);
}

class EdgeSteeringKinematics : public BaseTebMultiEdge<1, double>
{
public:
  EdgeSteeringKinematics()
  {
    this->resize(3);
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringKinematics()");
    const VertexPose * pose1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose * pose2 = static_cast<const VertexPose *>(_vertices[1]);
    const VertexTimeDiff * dt = static_cast<const VertexTimeDiff *>(_vertices[2]);
    SegmentKinematics segment;
    if (!segmentFromVertices(pose1, pose2, dt, cfg_, segment))
    {
      _error[0] = 0.0;
      return;
    }

    const double omega_epsilon = std::max(0.0, cfg_->robot.steering_omega_epsilon);
    if (segment.stopped || segment.inplace_rotation)
    {
      _error[0] = penaltyBoundToInterval(
        segment.omega, -omega_epsilon, omega_epsilon,
        steeringPenaltyEpsilon(omega_epsilon, cfg_->optim.penalty_epsilon));
      return;
    }

    double omega_limit = std::abs(segment.velocity) * maxCurvature(
      cfg_->robot.wheelbase, cfg_->robot.max_steering_angle);
    if (cfg_->robot.max_vel_theta > 0.0)
    {
      omega_limit = std::min(omega_limit, cfg_->robot.max_vel_theta);
    }
    _error[0] = penaltyBoundToInterval(
      segment.omega, -omega_limit, omega_limit,
      steeringPenaltyEpsilon(omega_limit, cfg_->optim.penalty_epsilon));
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringKinematics::computeError() _error[0]=%f\n", _error[0]);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeSteeringRate : public BaseTebMultiEdge<1, double>
{
public:
  EdgeSteeringRate()
  {
    this->resize(5);
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringRate()");
    const VertexPose * pose1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose * pose2 = static_cast<const VertexPose *>(_vertices[1]);
    const VertexPose * pose3 = static_cast<const VertexPose *>(_vertices[2]);
    const VertexTimeDiff * dt1 = static_cast<const VertexTimeDiff *>(_vertices[3]);
    const VertexTimeDiff * dt2 = static_cast<const VertexTimeDiff *>(_vertices[4]);
    SegmentKinematics first;
    SegmentKinematics second;
    if (!segmentFromVertices(pose1, pose2, dt1, cfg_, first) ||
      !segmentFromVertices(pose2, pose3, dt2, cfg_, second))
    {
      _error[0] = 0.0;
      return;
    }
    if (first.inplace_rotation || second.inplace_rotation)
    {
      _error[0] = std::abs(first.omega) + std::abs(second.omega);
      return;
    }
    const double rate = steeringRate(first, second);
    _error[0] = penaltyBoundToInterval(
      rate, cfg_->robot.max_steering_rate,
      steeringPenaltyEpsilon(cfg_->robot.max_steering_rate, cfg_->optim.penalty_epsilon));
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRate::computeError() _error[0]=%f\n", _error[0]);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeSteeringAcceleration : public BaseTebMultiEdge<1, double>
{
public:
  EdgeSteeringAcceleration()
  {
    this->resize(7);
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringAcceleration()");
    const VertexPose * pose1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose * pose2 = static_cast<const VertexPose *>(_vertices[1]);
    const VertexPose * pose3 = static_cast<const VertexPose *>(_vertices[2]);
    const VertexPose * pose4 = static_cast<const VertexPose *>(_vertices[3]);
    const VertexTimeDiff * dt1 = static_cast<const VertexTimeDiff *>(_vertices[4]);
    const VertexTimeDiff * dt2 = static_cast<const VertexTimeDiff *>(_vertices[5]);
    const VertexTimeDiff * dt3 = static_cast<const VertexTimeDiff *>(_vertices[6]);
    SegmentKinematics first;
    SegmentKinematics second;
    SegmentKinematics third;
    if (!segmentFromVertices(pose1, pose2, dt1, cfg_, first) ||
      !segmentFromVertices(pose2, pose3, dt2, cfg_, second) ||
      !segmentFromVertices(pose3, pose4, dt3, cfg_, third))
    {
      _error[0] = 0.0;
      return;
    }
    if (first.inplace_rotation || second.inplace_rotation || third.inplace_rotation)
    {
      _error[0] = std::abs(first.omega) + std::abs(second.omega) + std::abs(third.omega);
      return;
    }
    const double acceleration = steeringAcceleration(first, second, third);
    _error[0] = penaltyBoundToInterval(
      acceleration, cfg_->robot.max_steering_acceleration,
      steeringPenaltyEpsilon(cfg_->robot.max_steering_acceleration, cfg_->optim.penalty_epsilon));
    TEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeSteeringAcceleration::computeError() _error[0]=%f\n", _error[0]);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace teb_local_planner

#endif
