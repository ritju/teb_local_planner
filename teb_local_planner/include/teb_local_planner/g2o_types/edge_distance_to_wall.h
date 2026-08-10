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
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_DISTANCE_TO_WALL_H_
#define EDGE_DISTANCE_TO_WALL_H_

#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/misc.h"

#include "g2o/core/base_unary_edge.h"
#include <chrono>

using namespace std::chrono;
namespace teb_local_planner
{
  

/**
 * @class EdgeDistanceToWall
 * @brief 约束路径点到墙线的垂直距离；信息矩阵权重可按路径点与机器人（轨迹首点）距离线性缩放，见 distanceWeightScale().
 */
class EdgeDistanceToWall : public BaseTebUnaryEdge<1, const std::vector<Eigen::Vector2d>*, VertexPose>
{
public:
  /**
   * @brief 贴墙距离项权重系数：在距离首点 [0,R] 内从 k_min 线性增至 k_max，d>=R 为 k_max；R<=0 时恒为 k_max（关闭渐变）
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

  /**
   * @brief Construct edge.
   */    
  EdgeDistanceToWall() 
  {
    _measurement = NULL;
  }
  // std::chrono::seconds intervel_ = 2s;
  // std::chrono::_V2::steady_clock::time_point last_time_ = steady_clock::now();
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig(), setDistanceToWall() on EdgeDistanceToWall()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    if (_measurement && _measurement->size())
    {
      const double d = perpendicularDistance(bandpt->position(), *_measurement);
      // 超出匹配容差时不施加中心距吸引（勿用假测量值制造零误差）
      if (d > cfg_->wall_line.distance_tolerance)
        _error[0] = 0;
      else
        _error[0] = fabs(d - cfg_->wall_line.min_wall_dist);
    }
    else
    {
      _error[0] = 0;
    }

    // if (steady_clock::now() - last_time_ > intervel_)
    // {
    //   std::cout << "DistanceToWall::computeError() _error[0]: " << _error[0] << " !" << std::endl;
    //   last_time_ = steady_clock::now();
    // }
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "DistanceToWall::computeError() _error[0]=%f\n",_error[0]);
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param via_point 2D position vector containing the position of the via point
   */ 
  void setParameters(const TebConfig& cfg, const std::vector<Eigen::Vector2d>* wall_line)
  {
    cfg_ = &cfg;
    _measurement = wall_line;
  }

  // 计算点到由方向向量定义的线段（视为直线）的垂直距离
  double perpendicularDistance(const Eigen::Vector2d& point,
    const std::vector<Eigen::Vector2d> wall_direction) 
  {
    // 验证输入有效性
    if (wall_direction.size() != 2) {
      return cfg_->wall_line.min_wall_dist;
    }

    const Eigen::Vector2d& p1 = wall_direction[0];
    const Eigen::Vector2d& p2 = wall_direction[1];

    // 计算直线方向向量
    const Eigen::Vector2d dir = p2 - p1;

    // 构造直线方程：Ax + By + C = 0
    const double A = dir.y();  // y2 - y1
    const double B = -dir.x(); // x1 - x2
    const double C = p2.x()*p1.y() - p1.x()*p2.y(); // x2y1 - x1y2

    // 计算分子绝对值
    const double numerator = std::abs(A*point.x() + B*point.y() + C);

    // 计算分母（提前处理除零异常）
    const double denominator = std::hypot(A, B);

    if (denominator < 1e-9) { // 处理两点重合的情况
      return cfg_->wall_line.min_wall_dist;
    }
    // 始终返回真实垂距；远距是否施加代价由 computeError / 权重调度决定，禁止用 min_wall_dist 冒充测量值
    return numerator / denominator;
  }




  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
