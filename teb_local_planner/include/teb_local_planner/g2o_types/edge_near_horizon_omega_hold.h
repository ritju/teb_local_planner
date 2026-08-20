/*********************************************************************
 * Soft constraint: keep angular velocity near the current executed omega
 * on the near-horizon TEB segments (predictable motion under unreliable
 * dynamic-obstacle velocities). See 动态障碍物避让.md.
 *********************************************************************/

#ifndef EDGE_NEAR_HORIZON_OMEGA_HOLD_H
#define EDGE_NEAR_HORIZON_OMEGA_HOLD_H

#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/misc.h"

#include <cmath>

namespace teb_local_planner
{

/**
 * @class EdgeNearHorizonOmegaHold
 * @brief Penalize |omega - omega_ref| exceeding delta_omega_allow.
 *
 * Vertices: Pose_i, Pose_{i+1}, TimeDiff_i (same wiring as EdgeVelocity).
 * Soft interval penalty — does NOT hard-lock omega to omega_ref.
 */
class EdgeNearHorizonOmegaHold : public BaseTebMultiEdge<1, double>
{
public:
  EdgeNearHorizonOmegaHold()
  {
    this->resize(3);
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeNearHorizonOmegaHold()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    const double dt = deltaT->estimate();
    if (isTimeDiffDegenerate(dt))
    {
      _error[0] = 0.0;
      return;
    }

    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    const double omega = angle_diff / dt;
    const double diff = omega - omega_ref_;

    // Soft bound: zero inside [-delta, +delta], linear penalty outside
    _error[0] = penaltyBoundToInterval(diff, delta_omega_allow_, cfg_->optim.penalty_epsilon);
  }

  void setOmegaRef(double omega_ref) { omega_ref_ = omega_ref; }
  void setDeltaOmegaAllow(double delta) { delta_omega_allow_ = std::max(0.0, delta); }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double omega_ref_ = 0.0;
  double delta_omega_allow_ = 0.3;
};

} // namespace teb_local_planner

#endif
