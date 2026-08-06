/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 *********************************************************************/
#ifndef EDGE_WALL_SIDE_CLEARANCE_H_
#define EDGE_WALL_SIDE_CLEARANCE_H_

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/misc.h"

#include <cmath>
#include <limits>

namespace teb_local_planner
{

/**
 * @class EdgeWallSideClearance
 * @brief 矩形 footprint 到贴边参考障碍（墙线 guide）的侧隙代价（方案 A：二维 Pull/Push）
 *
 * \f$ g = \mathrm{dist}(F(s_i), W) \f$
 * - \c _error[0] = g - g*                     （Pull：始终拉向目标侧隙）
 * - \c _error[1] = max(g_push - g, 0)         （Push：仅过近时推远）
 *
 * 当 (g - g*) > attract_max 时将 Pull 误差置 0，避免过远乱拽。
 */
class EdgeWallSideClearance : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose>
{
public:
  EdgeWallSideClearance()
  {
    _measurement = nullptr;
    robot_model_ = nullptr;
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement && robot_model_,
                   "You must call setParameters() on EdgeWallSideClearance()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    const double g = robot_model_->calculateDistance(bandpt->pose(), _measurement);
    const double g_star = cfg_->wall_line.desired_side_clearance;
    const double g_push = cfg_->wall_line.side_clearance_push;
    const double attract_max = cfg_->wall_line.side_clearance_attract_max;

    double e_pull = g - g_star;
    if (attract_max > 0.0 && e_pull > attract_max)
      e_pull = 0.0;

    const double e_push = (g_push > g) ? (g_push - g) : 0.0;

    _error[0] = e_pull;
    _error[1] = e_push;

    TEB_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]),
                   "EdgeWallSideClearance::computeError() _error=[%f, %f] g=%f\n",
                   _error[0], _error[1], g);
  }

  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }

  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

  /** @brief footprint 到墙线 guide 的侧隙 g [m]（调试用） */
  double sideClearance() const
  {
    if (!robot_model_ || !_measurement || !_vertices[0])
      return std::numeric_limits<double>::quiet_NaN();
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    return robot_model_->calculateDistance(bandpt->pose(), _measurement);
  }

  /**
   * @brief 与 EdgeDistanceToWall 相同的轨迹前向权重缩放
   */
  static double distanceWeightScale(const TebConfig& cfg, double distance_from_robot_pose_m)
  {
    const double R = cfg.wall_line.wall_line_dist_robot_weight_radius;
    const double k_min = cfg.wall_line.wall_line_dist_weight_scale_at_robot;
    const double k_max = cfg.wall_line.wall_line_dist_weight_scale_far;
    if (R <= 1e-9)
      return k_max;
    double t = distance_from_robot_pose_m / R;
    if (t < 0.0)
      t = 0.0;
    else if (t > 1.0)
      t = 1.0;
    return k_min + (k_max - k_min) * t;
  }

protected:
  const BaseRobotFootprintModel* robot_model_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace teb_local_planner

#endif
