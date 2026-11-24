#pragma once

#include "teb_local_planner/g2o_types/edge_obstacle.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"

namespace teb_local_planner
{

// 在两个既有 TEB 位姿之间固定比例 s 处约束实际多边形，不新增位姿顶点。
class EdgeSweptObstacle : public BaseTebBinaryEdge<1, const Obstacle*, VertexPose, VertexPose>
{
public:
  void setParameters(
    const TebConfig & cfg, const BaseRobotFootprintModel * model,
    const Obstacle * obstacle, double fraction)
  {
    cfg_ = &cfg;
    model_ = model;
    _measurement = obstacle;
    fraction_ = fraction;
  }

  void computeError() override
  {
    const auto * a = static_cast<const VertexPose *>(_vertices[0]);
    const auto * b = static_cast<const VertexPose *>(_vertices[1]);
    const double d = model_->optimizationDistance(interpolate(a->pose(), b->pose()), _measurement)
      .signed_distance;
    _error[0] = penaltyBoundFromBelow(
      d, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
  }

  void linearizeOplus() override
  {
    const auto * a = static_cast<const VertexPose *>(_vertices[0]);
    const auto * b = static_cast<const VertexPose *>(_vertices[1]);
    const PoseSE2 pose = interpolate(a->pose(), b->pose());
    const auto distance = model_->optimizationDistance(pose, _measurement);
    const auto gradient = obstacleDistanceGradient(*model_, pose, _measurement, distance);
    const double error = penaltyBoundFromBelow(
      distance.signed_distance, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
    const Eigen::RowVector3d jacobian =
      obstaclePenaltySlope(error, *cfg_) * gradient.transpose();
    this->_jacobianOplusXi.row(0) = (1.0 - fraction_) * jacobian;
    this->_jacobianOplusXj.row(0) = fraction_ * jacobian;
  }

private:
  PoseSE2 interpolate(const PoseSE2 & a, const PoseSE2 & b) const
  {
    const double dtheta = g2o::normalize_theta(b.theta() - a.theta());
    return PoseSE2(
      a.position() + fraction_ * (b.position() - a.position()),
      g2o::normalize_theta(a.theta() + fraction_ * dtheta));
  }

  const BaseRobotFootprintModel * model_{nullptr};
  double fraction_{0.5};
};

}  // namespace teb_local_planner
