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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>

#include "teb_local_planner/optimal_planner.h"

#include <map>
#include <sstream>
// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_near_horizon_omega_hold.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <limits>
#include "dwb_core/exceptions.hpp"

namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(nullptr), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points, const std::vector<Eigen::Vector2d>* wall_line)
{    
  initialize(node, cfg, obstacles, visual, via_points, wall_line);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points, const std::vector<Eigen::Vector2d>* wall_line)
{    
  node_ = node;
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  wall_line_ = wall_line;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;

  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;

  setVisualization(visual);
}

void TebOptimalPlanner::setVisualization(const TebVisualizationPtr& visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;

  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *cfg_->robot_model);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);

  if (cfg_->trajectory.publish_dynamic_obstacle_debug && obstacles_)
  {
    visualization_->publishDynamicObstacleDebug(
        teb_,
        dyn_obst_edge_build_snapshot_valid_ ? &dyn_obst_edge_build_snapshot_ : nullptr,
        *obstacles_,
        *cfg_->robot_model);
  }
}

void TebOptimalPlanner::captureSamplingDebug(
  std::vector<geometry_msgs::msg::Pose2D> & out) const
{
  out.clear();
  out.reserve(teb_.sizePoses());
  for (int i = 0; i < teb_.sizePoses(); ++i) {
    geometry_msgs::msg::Pose2D pose;
    teb_.Pose(i).toPoseMsg(pose);
    out.push_back(pose);
  }
}

void TebOptimalPlanner::publishSamplingDebug(const char * result) const
{
  if (!visualization_ || !cfg_ ||
      !cfg_->debug_mode.load(std::memory_order_relaxed)) {return;}
  std::vector<geometry_msgs::msg::Pose2D> optimized;
  captureSamplingDebug(optimized);
  visualization_->publishTrajectorySamplingStages(
    sampling_initial_, sampling_after_resize_, optimized,
    sampling_resize_outer_iteration_, cfg_->trajectory.teb_autosize,
    this, result);
}

/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType("VERTEX_TIMEDIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
  factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType("EDGE_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType("EDGE_ACCELERATION", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType("EDGE_ACCELERATION_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType("EDGE_ACCELERATION_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType("EDGE_KINEMATICS_CARLIKE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  factory->registerType("EDGE_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType("EDGE_INFLATED_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType("EDGE_NEAR_HORIZON_OMEGA_HOLD", std::make_shared<g2o::HyperGraphElementCreator<EdgeNearHorizonOmegaHold>>());
  factory->registerType("EDGE_VIA_POINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType("EDGE_WALL_LINE_DIST", std::make_shared<g2o::HyperGraphElementCreator<EdgeDistanceToWall>>());
  factory->registerType("EDGE_WALL_SIDE_CLEARANCE", std::make_shared<g2o::HyperGraphElementCreator<EdgeWallSideClearance>>());
  factory->registerType("EDGE_WALL_LINE_DIRECTION", std::make_shared<g2o::HyperGraphElementCreator<EdgeParallelToWall>>());
  factory->registerType("EDGE_PREFER_ROTDIR", std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this](){this->registerG2OTypes();});

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<TEBLinearSolver>(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  auto blockSolver = std::make_unique<TEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  swept_repair_obstacles_.clear();
  forced_swept_segment_ = -1;
  swept_repair_active_ = false;
  sampling_initial_.clear();
  sampling_after_resize_.clear();
  sampling_resize_outer_iteration_ = 0;
  if (visualization_ && cfg_->debug_mode.load(std::memory_order_relaxed)) {
    captureSamplingDebug(sampling_initial_);
  }
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  const double min_dt = std::max(0.01 * cfg_->trajectory.dt_ref, kTimeDiffEpsilon);
  teb_.setMinDt(min_dt);

  try
  {
  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
      teb_.clampAllTimeDiffs();
    }
    if (!sampling_initial_.empty()) {
      captureSamplingDebug(sampling_after_resize_);
      sampling_resize_outer_iteration_ = i + 1;
    }

    success = buildGraph(weight_multiplier);
    if (!success) 
    {
        publishSamplingDebug("建图失败");
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) 
    {
        publishSamplingDebug("优化失败");
        clearGraph();
        return false;
    }
    optimized_ = true;

    // 最后一轮外层迭代、清图前：按间隔打印贴边/障碍代价
    if (i == iterations_outerloop - 1)
      logWallAndObstacleCostsIfDue();
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
      
    clearGraph();
    
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }
  }
  catch (const TebAssertionFailureException& ex)
  {
    logTebNumericFailure("optimizeTEB", ex);
    publishSamplingDebug("数值断言失败");
    clearGraph();
    teb_.clearTimedElasticBand();
    optimized_ = false;
    return false;
  }

  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::msg::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::msg::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{    
  TEB_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


//bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
//{
//  PoseSE2 start_(start);
//  PoseSE2 goal_(goal);
//  return plan(start_, goal_, start_vel);
//}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{	
  TEB_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  
  // add TEB vertices
  AddTEBVertices();
  
  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);
  AddEdgesSweptObstacles(weight_multiplier);

  // Dynamic obstacle edges:
  // - safety predictable mode: do NOT use AddEdgesDynamicObstacles as the main path;
  //   optional NearHorizonOmegaHold is added instead (gated via cfg_->near_horizon_runtime).
  // - legacy mode: keep constant-velocity spatiotemporal edges.
  if (cfg_->obstacles.dynamic_safety_predictable_mode)
  {
    dyn_obst_edge_build_snapshot_.clear();
    dyn_obst_edge_build_snapshot_valid_ = false;
    AddEdgesNearHorizonOmegaHold();
  }
  else if (cfg_->obstacles.include_dynamic_obstacles)
  {
    AddEdgesDynamicObstacles();
  }
  else
  {
    dyn_obst_edge_build_snapshot_.clear();
    dyn_obst_edge_build_snapshot_valid_ = false;
  }
  
  AddEdgesViaPoints();
  
  AddEdgesDistanceToWall();

  // AddEdgesParallelToWall();
  
  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	

  AddEdgesShortestPath();
  
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();
    
  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
    RCLCPP_ERROR(node_->get_logger(), "optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  RCLCPP_DEBUG_EXPRESSION(node_->get_logger(), cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}


void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    
  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {    
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;
      
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // Legacy: skip dynamic obstacles here and handle them in AddEdgesDynamicObstacles.
        // Safety mode: optional switch dynamic_obstacles_as_static_edges builds static edges
        // from the obstacle's current pose (do not hard-code; controlled by parameter).
        if (cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic() &&
            !cfg_->obstacles.dynamic_obstacles_as_static_edges)
          continue;
        // Wall-guide line is handled by EdgeWallSideClearance
        if (obst->isWallGuide())
          continue;

          // calculate distance to robot model
          double dist = cfg_->robot_model->optimizationDistance(teb_.Pose(i), obst.get()).signed_distance;
          
          // force considering obstacle if really close to the current pose
        if (dist <= 0.0 || dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;
          
          // Determine side (left or right) using the obstacle position relative to this pose.
          // Must use (centroid - pose), not absolute centroid — otherwise map-frame quadrant
          // dominates and left/right association is wrong (upstream teb PR #415).
          if (cross2d(pose_orient, obst->getCentroid() - teb_.Pose(i).position()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }   
      
      if (left_obstacle)
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle)
        create_edge(i, obst.get());
      ++iter_obstacle;
  }
}


namespace
{
struct SweptAabb
{
  double xmin, ymin, xmax, ymax;
};

bool sweptAabbForObstacle(const Obstacle * obst, SweptAabb & box)
{
  double radius = 0.0;
  std::vector<Eigen::Vector2d> points;
  if (const auto * p = dynamic_cast<const PointObstacle *>(obst)) {
    points.push_back(p->position());
  } else if (const auto * c = dynamic_cast<const CircularObstacle *>(obst)) {
    points.push_back(c->position()); radius = c->radius();
  } else if (const auto * l = dynamic_cast<const LineObstacle *>(obst)) {
    points = {l->start(), l->end()};
  } else if (const auto * pill = dynamic_cast<const PillObstacle *>(obst)) {
    points = {pill->start(), pill->end()};
    radius = std::max(0.0, -pill->getMinimumDistance(pill->start()));
  } else if (const auto * poly = dynamic_cast<const PolygonObstacle *>(obst)) {
    points.assign(poly->vertices().begin(), poly->vertices().end());
  } else {
    return false;  // 未知障碍不能被包围盒过滤
  }
  if (points.empty() || !std::isfinite(radius)) {return false;}
  box = {points.front().x(), points.front().y(), points.front().x(), points.front().y()};
  for (const auto & point : points) {
    if (!point.allFinite()) {return false;}
    box.xmin = std::min(box.xmin, point.x());
    box.ymin = std::min(box.ymin, point.y());
    box.xmax = std::max(box.xmax, point.x());
    box.ymax = std::max(box.ymax, point.y());
  }
  box.xmin -= radius; box.ymin -= radius;
  box.xmax += radius; box.ymax += radius;
  return true;
}

bool sweptAabbOverlap(const SweptAabb & a, const SweptAabb & b)
{
  return a.xmin <= b.xmax && b.xmin <= a.xmax &&
         a.ymin <= b.ymax && b.ymin <= a.ymax;
}

PoseSE2 sweptInterpolate(const PoseSE2 & a, const PoseSE2 & b, double fraction)
{
  return PoseSE2(
    a.position() + fraction * (b.position() - a.position()),
    g2o::normalize_theta(a.theta() + fraction * g2o::normalize_theta(b.theta() - a.theta())));
}
}  // namespace

void TebOptimalPlanner::AddEdgesSweptObstacles(double weight_multiplier)
{
  const auto started = std::chrono::steady_clock::now();
  swept_danger_segments_.clear();
  swept_sample_poses_.clear();
  swept_candidate_obstacles_.clear();
  if (!cfg_->trajectory.swept_obstacle_enable || !obstacles_ ||
      cfg_->optim.weight_obstacle <= 0.0 || teb_.sizePoses() < 2) {return;}
  const auto * polygon = dynamic_cast<const PolygonRobotFootprint *>(cfg_->robot_model.get());
  if (!polygon || polygon->vertices().size() < 3) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[TEB 扫掠约束] 优化 footprint 不是有效多边形，无法约束前角；请与代价图 footprint 对齐");
    return;
  }
  double radius = 0.0;
  for (const auto & vertex : polygon->vertices()) {radius = std::max(radius, vertex.norm());}
  const double pad = radius + std::max(cfg_->obstacles.inflation_dist,
    cfg_->obstacles.min_obstacle_dist) + std::max(0.0, cfg_->trajectory.swept_obstacle_margin);
  const int limit = std::max(1, cfg_->trajectory.swept_obstacle_max_edges_per_segment);
  const double trigger = std::max(cfg_->obstacles.min_obstacle_dist,
    cfg_->trajectory.swept_obstacle_trigger_dist);
  const double max_check = cfg_->trajectory.feasibility_check_lookahead_distance;
  size_t broadphase_pairs = 0, edge_count = 0, truncated = 0;
  double min_signed = std::numeric_limits<double>::infinity();
  for (int i = 0; i < teb_.sizePoses() - 1; ++i) {
    const auto & a = teb_.Pose(i);
    const auto & b = teb_.Pose(i + 1);
    if (max_check > 0.0 && (a.position() - teb_.Pose(0).position()).norm() > max_check) {
      break;
    }
    const SweptAabb sweep{
      std::min(a.x(), b.x()) - pad, std::min(a.y(), b.y()) - pad,
      std::max(a.x(), b.x()) + pad, std::max(a.y(), b.y()) + pad};
    struct Candidate {const Obstacle * obstacle; double fraction; double distance; bool repair_cell;};
    std::vector<Candidate> candidates;
    auto consider = [&](const ObstaclePtr & obst, bool repair_cell) {
      if (!obst || obst->isWallGuide() ||
          (obst->isDynamic() && cfg_->obstacles.include_dynamic_obstacles &&
           !cfg_->obstacles.dynamic_obstacles_as_static_edges)) {return;}
      SweptAabb obstacle_box;
      if (sweptAabbForObstacle(obst.get(), obstacle_box) &&
          !sweptAabbOverlap(sweep, obstacle_box)) {return;}
      ++broadphase_pairs;
      const bool forced = i == forced_swept_segment_;
      std::array<double, 3> fractions{{0.25, 0.5, 0.75}};
      double best_distance = std::numeric_limits<double>::infinity();
      double best_fraction = 0.5;
      for (const double fraction : fractions) {
        const double distance = cfg_->robot_model->optimizationDistance(
          sweptInterpolate(a, b, fraction), obst.get()).signed_distance;
        if (distance < best_distance) {
          best_distance = distance;
          best_fraction = fraction;
        }
      }
      if (forced) {
        // 修复边必须落在硬检查实际失败的位置，不能被其他试探点取代。
        best_fraction = forced_swept_fraction_;
        best_distance = cfg_->robot_model->optimizationDistance(
          sweptInterpolate(a, b, best_fraction), obst.get()).signed_distance;
      }
      if (best_distance <= trigger || forced) {
        min_signed = std::min(min_signed, best_distance);
        candidates.push_back({obst.get(), best_fraction, best_distance, repair_cell});
      }
    };
    for (const auto & obst : *obstacles_) {consider(obst, false);}
    if (i == forced_swept_segment_) {
      for (const auto & obst : swept_repair_obstacles_) {consider(obst, true);}
    }
    if (candidates.empty()) {continue;}
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.distance < rhs.distance;
    });
    swept_danger_segments_.push_back(i);
    // 穿透项全部保留。预算只限制额外的接近项；最终硬检查仍负责拒绝未解决的碰撞。
    std::vector<int> selected_indices;
    for (int k = 0; k < static_cast<int>(candidates.size()); ++k) {
      if (candidates[k].distance <= 0.0) {selected_indices.push_back(k);}
    }
    // 代价图独有障碍至少补入一个；否则普通障碍可能耗尽该段预算。
    if (!swept_repair_obstacles_.empty() && i == forced_swept_segment_ &&
        std::none_of(selected_indices.begin(), selected_indices.end(), [&](int k) {
          return candidates[k].repair_cell;
        })) {
      const auto cell = std::find_if(candidates.begin(), candidates.end(),
        [](const Candidate & candidate) {return candidate.repair_cell;});
      if (cell != candidates.end()) {
        selected_indices.push_back(static_cast<int>(cell - candidates.begin()));
      }
    }
    for (int k = 0; k < static_cast<int>(candidates.size()) &&
         static_cast<int>(selected_indices.size()) < limit; ++k) {
      if (std::find(selected_indices.begin(), selected_indices.end(), k) == selected_indices.end()) {
        selected_indices.push_back(k);
      }
    }
    if (selected_indices.size() < candidates.size()) {
      truncated += candidates.size() - selected_indices.size();
    }
    for (const int k : selected_indices) {
      auto * edge = new EdgeSweptObstacle;
      edge->setVertex(0, teb_.PoseVertex(i));
      edge->setVertex(1, teb_.PoseVertex(i + 1));
      Eigen::Matrix<double, 1, 1> information;
      information[0] = cfg_->optim.weight_obstacle * weight_multiplier / selected_indices.size();
      edge->setInformation(information);
      edge->setParameters(*cfg_, cfg_->robot_model.get(), candidates[k].obstacle,
        candidates[k].fraction);
      optimizer_->addEdge(edge);
      swept_sample_poses_.push_back(sweptInterpolate(a, b, candidates[k].fraction));
      geometry_msgs::msg::Point centroid;
      centroid.x = candidates[k].obstacle->getCentroid().x();
      centroid.y = candidates[k].obstacle->getCentroid().y();
      swept_candidate_obstacles_.push_back(centroid);
      ++edge_count;
    }
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[TEB 扫掠建边] 候选=%p 包围盒命中=%zu 危险段=%zu 新增边=%zu 截断=%zu 最小距离=%.3f 半径=%.3f 裕度=%.3f 耗时=%.1f ms",
    static_cast<const void*>(this), broadphase_pairs, swept_danger_segments_.size(),
    edge_count, truncated, std::isfinite(min_signed) ? min_signed : -1.0,
    radius, pad - radius, elapsed_ms);
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    // Same skip policy as AddEdgesObstacles (see dynamic_obstacles_as_static_edges).
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic() &&
        !cfg_->obstacles.dynamic_obstacles_as_static_edges)
      continue; 
    if ((*obst)->isWallGuide())
      continue;
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}


void TebOptimalPlanner::AddEdgesNearHorizonOmegaHold()
{
  // Soft constraint on near-horizon angular velocity: |omega - omega_ref| <= scale*max_vel_theta
  if (!cfg_->obstacles.dynamic_safety_predictable_mode)
  {
    return;
  }
  if (!cfg_->near_horizon_runtime.omega_hold_enable)
  {
    if (cfg_->debug_mode.load(std::memory_order_relaxed)) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "[NearHorizon] OmegaHold skipped: gate disabled this cycle");
    }
    return;
  }
  if (cfg_->optim.weight_near_horizon_omega_hold <= 0.0 || teb_.sizePoses() < 2)
  {
    return;
  }

  const double omega_ref = cfg_->near_horizon_runtime.omega_ref;
  const double delta_allow =
      std::max(0.0, cfg_->obstacles.delta_omega_scale * cfg_->robot.max_vel_theta);
  const double T_near = cfg_->obstacles.dyn_gate_T_near;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_near_horizon_omega_hold);

  double t_acc = 0.0;
  int edges_added = 0;
  for (int i = 0; i < teb_.sizePoses() - 1; ++i)
  {
    if (t_acc > T_near)
    {
      break;
    }

    EdgeNearHorizonOmegaHold* edge = new EdgeNearHorizonOmegaHold;
    edge->setVertex(0, teb_.PoseVertex(i));
    edge->setVertex(1, teb_.PoseVertex(i + 1));
    edge->setVertex(2, teb_.TimeDiffVertex(i));
    edge->setInformation(information);
    edge->setTebConfig(*cfg_);
    edge->setOmegaRef(omega_ref);
    edge->setDeltaOmegaAllow(delta_allow);
    optimizer_->addEdge(edge);
    ++edges_added;

    t_acc += teb_.TimeDiff(i);
  }

  if (node_)
  {
    if (cfg_->debug_mode.load(std::memory_order_relaxed)) {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[NearHorizon] AddEdgesNearHorizonOmegaHold: edges=%d omega_ref=%.3f delta_allow=%.3f T_near=%.2f",
          edges_added, omega_ref, delta_allow, T_near);
    }
  }
}

void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
  {
    dyn_obst_edge_build_snapshot_.clear();
    dyn_obst_edge_build_snapshot_valid_ = false;
    return; // if weight equals zero skip adding edges!
  }

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  // Capture TEB pose/time used while constructing dynamic-obstacle edges (pre-optimize snapshot for this outer loop)
  dyn_obst_edge_build_snapshot_.clear();
  dyn_obst_edge_build_snapshot_valid_ = false;
  if (cfg_->trajectory.publish_dynamic_obstacle_debug && teb_.sizePoses() >= 3)
  {
    double snap_time = teb_.TimeDiff(0);
    for (int i = 1; i < teb_.sizePoses() - 1; ++i)
    {
      TebPoseTimeSample sample;
      sample.pose_idx = i;
      sample.t = snap_time;
      sample.pose = teb_.Pose(i);
      dyn_obst_edge_build_snapshot_.push_back(sample);
      snap_time += teb_.TimeDiff(i);
    }
    dyn_obst_edge_build_snapshot_valid_ = !dyn_obst_edge_build_snapshot_.empty();
  }
  
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        RCLCPP_DEBUG(node_->get_logger(), "TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0) 
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void TebOptimalPlanner::AddEdgesDistanceToWall()
{
  if (!wall_line_ || wall_line_->size() < 2)
    return;

  const Eigen::Vector2d p0 = teb_.PoseVertex(0)->position();

  // --- Scheme A: footprint side clearance (Pull/Push) ---
  if (cfg_->wall_line.wall_side_clearance_mode)
  {
    double w_pull = cfg_->optim.weight_wall_side_pull;
    if (w_pull <= 0.0)
      w_pull = cfg_->optim.weight_wall_line_dist;
    const double w_push = cfg_->optim.weight_wall_side_push;
    if (w_pull <= 0.0 && w_push <= 0.0)
      return;

    const Obstacle* wall_guide = nullptr;
    if (obstacles_)
    {
      for (const ObstaclePtr& obst : *obstacles_)
      {
        if (obst && obst->isWallGuide())
        {
          wall_guide = obst.get();
          break;
        }
      }
    }
    if (!wall_guide)
    {
      if (!wall_side_clearance_fallback_)
        wall_side_clearance_fallback_ = std::make_shared<LineObstacle>();
      wall_side_clearance_fallback_->setStart((*wall_line_)[0]);
      wall_side_clearance_fallback_->setEnd((*wall_line_)[1]);
      wall_side_clearance_fallback_->setWallGuide(true);
      wall_guide = wall_side_clearance_fallback_.get();
    }

    const BaseRobotFootprintModel* robot_model =
      cfg_->robot_model ? cfg_->robot_model.get() : robot_model_.get();
    if (!robot_model)
      return;

    Eigen::Matrix<double,2,2> information;
    information.setZero();
    for (int i = 0; i < teb_.sizePoses() - 1; ++i)
    {
      const double dist_from_robot = (teb_.PoseVertex(i)->position() - p0).norm();
      const double k = EdgeWallSideClearance::distanceWeightScale(*cfg_, dist_from_robot);
      information(0, 0) = w_pull * k;
      information(1, 1) = w_push * k;
      EdgeWallSideClearance* edge = new EdgeWallSideClearance;
      edge->setVertex(0, teb_.PoseVertex(i));
      edge->setInformation(information);
      edge->setParameters(*cfg_, robot_model, wall_guide);
      optimizer_->addEdge(edge);
    }
    return;
  }

  // --- Legacy: center-to-wall distance ---
  if (cfg_->optim.weight_wall_line_dist == 0)
    return;
  Eigen::Matrix<double,1,1> information;
  for (int i = 0; i < teb_.sizePoses() - 1; ++i)
  {
    const double dist_from_robot = (teb_.PoseVertex(i)->position() - p0).norm();
    const double k = EdgeDistanceToWall::distanceWeightScale(*cfg_, dist_from_robot);
    information.fill(cfg_->optim.weight_wall_line_dist * k);
    EdgeDistanceToWall* wall_line_dist_edge = new EdgeDistanceToWall;
    wall_line_dist_edge->setVertex(0, teb_.PoseVertex(i));
    wall_line_dist_edge->setInformation(information);
    wall_line_dist_edge->setParameters(*cfg_, wall_line_);
    optimizer_->addEdge(wall_line_dist_edge);
  }
}

void TebOptimalPlanner::AddEdgesParallelToWall()
{
  if (cfg_->optim.weight_wall_line_direction==0)
    return; // if weight equals zero skip adding edges!
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_wall_line_dist);
  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeParallelToWall* wall_line_direction_edge = new EdgeParallelToWall;
    wall_line_direction_edge->setVertex(0,teb_.PoseVertex(i));
    wall_line_direction_edge->setInformation(information);
    wall_line_direction_edge->setParameters(*cfg_, wall_line_);
    optimizer_->addEdge(wall_line_direction_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    RCLCPP_WARN(node_->get_logger(), "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      if (obstacle && obstacle->isWallGuide())
        continue;
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, cfg_->robot_model.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::logTebNumericFailure(const char* where, const std::exception& ex) const
{
  double min_dt = std::numeric_limits<double>::infinity();
  int min_dt_index = -1;
  for (int i = 0; i < teb_.sizeTimeDiffs(); ++i)
  {
    const double dt = teb_.TimeDiff(i);
    if (!std::isfinite(dt) || dt < min_dt)
    {
      min_dt = dt;
      min_dt_index = i;
    }
  }

  double segment_distance = std::numeric_limits<double>::quiet_NaN();
  double yaw0 = std::numeric_limits<double>::quiet_NaN();
  double yaw1 = std::numeric_limits<double>::quiet_NaN();
  if (min_dt_index >= 0 && min_dt_index + 1 < teb_.sizePoses())
  {
    const PoseSE2& pose0 = teb_.Pose(min_dt_index);
    const PoseSE2& pose1 = teb_.Pose(min_dt_index + 1);
    segment_distance = (pose1.position() - pose0.position()).norm();
    yaw0 = pose0.theta();
    yaw1 = pose1.theta();
  }

  auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("teb_local_planner");
  RCLCPP_ERROR(
      logger,
      "TEB numeric failure at %s: %s poses=%d timediffs=%d min_dt=%.6g at i=%d "
      "seg_dist=%.4f yaw=(%.3f,%.3f)",
      where,
      ex.what(),
      teb_.sizePoses(),
      teb_.sizeTimeDiffs(),
      min_dt,
      min_dt_index,
      segment_distance,
      yaw0,
      yaw1);
}

void TebOptimalPlanner::logWallAndObstacleCostsIfDue()
{
  if (!cfg_ || !cfg_->debug_mode.load(std::memory_order_relaxed) ||
      !optimizer_ || !node_ || !enable_wall_obstacle_cost_log_)
    return;

  const double interval = cfg_->wall_line.debug_wall_obstacle_cost_interval;
  if (interval <= 0.0)
    return;

  const rclcpp::Time now = node_->now();
  if (last_wall_obstacle_cost_log_time_.nanoseconds() != 0 &&
      (now - last_wall_obstacle_cost_log_time_).seconds() < interval)
    return;
  last_wall_obstacle_cost_log_time_ = now;

  double wall_side_chi2 = 0.0;
  double wall_side_pull = 0.0;
  double wall_side_push = 0.0;
  double wall_legacy_chi2 = 0.0;
  double obst_chi2 = 0.0;
  double obst_hard = 0.0;
  double obst_inflation = 0.0;
  double dyn_obst_chi2 = 0.0;
  int n_wall_side = 0;
  int n_wall_legacy = 0;
  int n_obst = 0;
  int n_inflated = 0;
  int n_dyn = 0;
  double g_min = std::numeric_limits<double>::infinity();
  double g_max = -std::numeric_limits<double>::infinity();
  double g_sum = 0.0;
  int g_count = 0;
  double g0 = std::numeric_limits<double>::quiet_NaN();

  for (g2o::OptimizableGraph::Edge* edge : optimizer_->activeEdges())
  {
    if (auto* e = dynamic_cast<EdgeWallSideClearance*>(edge))
    {
      const auto& err = e->getError();
      const auto& info = e->information();
      const double pull = err[0] * err[0] * info(0, 0);
      const double push = err[1] * err[1] * info(1, 1);
      wall_side_pull += pull;
      wall_side_push += push;
      wall_side_chi2 += e->chi2();
      ++n_wall_side;

      const double g = e->sideClearance();
      if (std::isfinite(g))
      {
        if (g_count == 0)
          g0 = g;
        g_min = std::min(g_min, g);
        g_max = std::max(g_max, g);
        g_sum += g;
        ++g_count;
      }
    }
    else if (dynamic_cast<EdgeDistanceToWall*>(edge))
    {
      wall_legacy_chi2 += edge->chi2();
      ++n_wall_legacy;
    }
    else if (auto* e = dynamic_cast<EdgeInflatedObstacle*>(edge))
    {
      const auto& err = e->getError();
      const auto& info = e->information();
      obst_hard += err[0] * err[0] * info(0, 0);
      obst_inflation += err[1] * err[1] * info(1, 1);
      obst_chi2 += e->chi2();
      ++n_inflated;
    }
    else if (dynamic_cast<EdgeObstacle*>(edge))
    {
      const double c = edge->chi2();
      obst_hard += c;
      obst_chi2 += c;
      ++n_obst;
    }
    else if (dynamic_cast<EdgeDynamicObstacle*>(edge))
    {
      dyn_obst_chi2 += edge->chi2();
      ++n_dyn;
    }
  }

  const double wall_total = wall_side_chi2 + wall_legacy_chi2;
  const double obst_total = obst_chi2 + dyn_obst_chi2;

  // n=0：本轮未添加贴边边（wall_line 为空/未贴边），g 无样本，不是计算错误
  if (g_count <= 0)
  {
    const char * wall_state = "null";
    std::size_t wall_size = 0u;
    if (wall_line_)
    {
      wall_size = wall_line_->size();
      wall_state = (wall_size >= 2u) ? "ok" : "empty";
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "[TEB cost dbg] no side-clearance samples (n_wall_side=%d n_legacy=%d wall_line=%s size=%zu) "
        "obst_total=%.3f (hard=%.3f infl=%.3f n_pt=%d n_inf=%d | dyn=%.3f n=%d) "
        "g*=%.3f g_push=%.3f — g0/g_avg/g_min/g_max N/A",
        n_wall_side, n_wall_legacy,
        wall_state, wall_size,
        obst_total,
        obst_hard, obst_inflation, n_obst, n_inflated,
        dyn_obst_chi2, n_dyn,
        cfg_->wall_line.desired_side_clearance,
        cfg_->wall_line.side_clearance_push);
    return;
  }

  const double g_avg = g_sum / static_cast<double>(g_count);

  RCLCPP_INFO(
      node_->get_logger(),
      "[TEB cost dbg] wall_total=%.3f (side=%.3f pull=%.3f push=%.3f n=%d | legacy=%.3f n=%d) "
      "obst_total=%.3f (hard=%.3f infl=%.3f n_pt=%d n_inf=%d | dyn=%.3f n=%d) "
      "g0=%.3f g_avg=%.3f g_min=%.3f g_max=%.3f g*=%.3f g_push=%.3f",
      wall_total,
      wall_side_chi2, wall_side_pull, wall_side_push, n_wall_side,
      wall_legacy_chi2, n_wall_legacy,
      obst_total,
      obst_hard, obst_inflation, n_obst, n_inflated,
      dyn_obst_chi2, n_dyn,
      g0, g_avg, g_min, g_max,
      cfg_->wall_line.desired_side_clearance,
      cfg_->wall_line.side_clearance_push);
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (isTimeDiffDegenerate(dt))
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    RCLCPP_ERROR(node_->get_logger(), "TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    RCLCPP_ERROR(node_->get_logger(), "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::msg::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<teb_msgs::msg::TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  teb_msgs::msg::TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start = durationFromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    teb_msgs::msg::TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start = durationFromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  teb_msgs::msg::TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start = durationFromSec(curr_time);
}


void TebOptimalPlanner::logTrajectoryClearanceIfDue(
  bool feasible, const char * reason, int checked_until,
  const std::vector<geometry_msgs::msg::Point> & costmap_footprint)
{
  if (!cfg_ || !node_) {return;}
  const auto now = node_->now();
  const std::string current_reason = reason ? reason : "unknown";
  const char * reason_cn = current_reason == "checks_passed" ? "检查通过" :
    current_reason == "costmap_pose_rejected" ? "代价地图位姿碰撞" :
    current_reason == "costmap_interpolation_rejected" ? "代价地图段内插值碰撞" :
    current_reason == "contour_rejected" ? "障碍轮廓检查拒绝" : "其他原因";
  const bool changed = current_reason != last_feasibility_event_reason_ ||
    (!feasible && checked_until != last_feasibility_event_index_);
  const bool summary_due = now < last_feasibility_event_time_ ||
    (now - last_feasibility_event_time_).seconds() >= 2.0;
  if (changed || (!feasible && summary_due)) {
    if (feasible) {
      if (!last_feasibility_event_reason_.empty()) {
        RCLCPP_INFO(node_->get_logger(),
          "[TEB 可行性事件] 候选=%p 已恢复通过，前次原因=%s，前次重复省略=%zu次",
          static_cast<const void *>(this), last_feasibility_event_reason_.c_str(),
          suppressed_feasibility_events_);
      }
    } else {
      const PoseSE2 * pose = checked_until >= 0 && checked_until < teb_.sizePoses() ?
        &teb_.Pose(checked_until) : nullptr;
      std::ostringstream footprint_text;
      for (const auto & vertex : costmap_footprint) {
        footprint_text << "(" << vertex.x << "," << vertex.y << ")";
      }
      RCLCPP_WARN(node_->get_logger(),
        "[TEB 可行性事件] 候选=%p 轨迹拒绝，原因=%s，段/位姿=%d，参考位姿=(%.3f,%.3f,%.3f)，"
        "障碍数=%zu，车体轮廓=%s，前次重复省略=%zu次",
        static_cast<const void *>(this), reason_cn, checked_until,
        pose ? pose->x() : std::numeric_limits<double>::quiet_NaN(),
        pose ? pose->y() : std::numeric_limits<double>::quiet_NaN(),
        pose ? pose->theta() : std::numeric_limits<double>::quiet_NaN(),
        obstacles_ ? obstacles_->size() : 0, footprint_text.str().c_str(),
        suppressed_feasibility_events_);
    }
    suppressed_feasibility_events_ = 0;
    last_feasibility_event_time_ = now;
  } else if (!feasible) {
    ++suppressed_feasibility_events_;
  }
  last_feasibility_event_reason_ = current_reason;
  last_feasibility_event_index_ = checked_until;

  if (!obstacles_ || !cfg_->robot_model ||
    !cfg_->debug_mode.load(std::memory_order_relaxed)) {return;}
  // 节流判定必须在全轨迹 × 全障碍轮廓扫描之前。
  if (current_reason == last_clearance_detail_reason_ && now >= last_clearance_log_time_ &&
    (now-last_clearance_log_time_).seconds() < 1.0) {return;}
  double minimum = std::numeric_limits<double>::infinity();
  double minimum_free = minimum;
  int worst_pose = -1, worst_obstacle = -1;
  size_t penetrating = 0, inside_margin = 0, invalid = 0, guides = 0;
  size_t associated_pairs = 0;
  for (const auto & obstacle : *obstacles_) {if (obstacle && obstacle->isWallGuide()) {++guides;}}
  for (int i = 0; i < teb_.sizePoses(); ++i) {
    for (size_t j = 0; j < obstacles_->size(); ++j) {
      const auto & obstacle = obstacles_->at(j);
      if (!obstacle || obstacle->isWallGuide()) {continue;}
      double distance;
      try {
        distance = cfg_->robot_model->optimizationDistance(teb_.Pose(i), obstacle.get()).signed_distance;
      } catch (...) {++invalid; continue;}
      if (!std::isfinite(distance)) {++invalid; continue;}
      if (distance < 0.0) {++penetrating;}
      if (distance < cfg_->obstacles.min_obstacle_dist) {++inside_margin;}
      if (!teb_.PoseVertex(i)->fixed()) {minimum_free = std::min(minimum_free, distance);}
      if (distance < minimum) {minimum = distance; worst_pose = i; worst_obstacle = j;}
      if (i > 0 && i+1 < teb_.sizePoses() && static_cast<size_t>(i) < obstacles_per_vertex_.size() &&
        std::find(obstacles_per_vertex_[i].begin(), obstacles_per_vertex_[i].end(), obstacle) !=
        obstacles_per_vertex_[i].end()) {++associated_pairs;}
    }
  }
  last_clearance_log_time_ = now;
  last_clearance_detail_reason_ = current_reason;
  const bool pass_with_penetration = feasible && minimum < 0.0;
  const bool pass_with_invalid = feasible && !pass_with_penetration && invalid != 0;
  std::ostringstream summary;
  summary << "[TEB Clearance] 轨迹检查 候选=" << this
          << " ROS时刻=" << now.seconds() << "秒 坐标系=" << cfg_->map_frame
          << " 结果=" << (feasible ? "通过" : "拒绝")
          << " 原因=" << reason_cn << "(" << reason << ")"
          << " 检查至位姿/段=" << checked_until << " 全轨迹位姿=" << teb_.sizePoses()
          << " 优化障碍=" << obstacles_->size() << " 排除墙线引导=" << guides
          << " 轮廓硬检查=" << (cfg_->trajectory.feasibility_check_obstacle_contours ? "开启" : "关闭");
  std::ostringstream clearance;
  clearance << "[TEB Clearance] 全轨迹轮廓诊断 候选=" << this
            << " 最小有符号距离=" << minimum << "米 自由位姿最小距离=" << minimum_free << "米"
            << " 侵入对=" << penetrating << " 小于最小间距的对=" << inside_margin
            << " 距离无效对=" << invalid << " 静态关联缓存对=" << associated_pairs
            << " 最小间距=" << cfg_->obstacles.min_obstacle_dist << "米"
            << " 代价缓冲=" << cfg_->optim.penalty_epsilon << "米"
            << " 障碍权重=" << cfg_->optim.weight_obstacle
            << " 障碍模型内切半径=" << cfg_->robot_model->getInscribedRadius() << "米"
            << " 内/外迭代=" << cfg_->optim.no_inner_iterations << "/" << cfg_->optim.no_outer_iterations
            << " 旧版关联=" << (cfg_->obstacles.legacy_obstacle_association ? "是" : "否");
  std::ostringstream geometry;
  geometry << "[TEB Clearance] 最危险几何 候选=" << this;
  if (worst_pose >= 0) {
    const auto & pose = teb_.Pose(worst_pose);
    const auto & obstacle = obstacles_->at(worst_obstacle);
    const bool associated = worst_pose > 0 && worst_pose+1 < teb_.sizePoses() &&
      static_cast<size_t>(worst_pose) < obstacles_per_vertex_.size() &&
      std::find(obstacles_per_vertex_[worst_pose].begin(), obstacles_per_vertex_[worst_pose].end(),
        obstacle) != obstacles_per_vertex_[worst_pose].end();
    const char * type = dynamic_cast<PointObstacle *>(obstacle.get()) ? "点" :
      dynamic_cast<CircularObstacle *>(obstacle.get()) ? "圆" :
      dynamic_cast<LineObstacle *>(obstacle.get()) ? "线段" :
      dynamic_cast<PillObstacle *>(obstacle.get()) ? "胶囊" : "多边形";
    geometry << " 位姿序号=" << worst_pose
             << " 固定=" << (teb_.PoseVertex(worst_pose)->fixed() ? "是" : "否")
             << " 属于本次检查范围=" << (worst_pose <= checked_until ? "是" : "否")
             << " 位姿=(" << pose.x() << "," << pose.y() << "," << pose.theta() << "弧度)"
             << " 障碍容器序号=" << worst_obstacle << " 类型=" << type
             << " 动态标记=" << (obstacle->isDynamic() ? "是" : "否")
             << " 静态关联缓存=" << (associated ? "已关联" : "未关联")
             << " 障碍质心=(" << obstacle->getCentroid().x() << "," << obstacle->getCentroid().y() << ")";
    if (worst_pose > checked_until) {
      geometry << "（全轨迹诊断点，非本次拒绝位置）";
    }
    geometry_msgs::msg::Polygon polygon;
    obstacle->toPolygonMsg(polygon);
    geometry << " 障碍顶点数=" << polygon.points.size() << " 顶点=[";
    for (size_t j = 0; j < std::min<size_t>(polygon.points.size(),16); ++j) {
      geometry << "(" << polygon.points[j].x << "," << polygon.points[j].y << ")";
    }
    geometry << "]";
  } else {
    geometry << " 无可计算的位姿/障碍对";
  }
  geometry << " 代价地图车体轮廓=[";
  for (const auto & p : costmap_footprint) {geometry << "(" << p.x << "," << p.y << ")";}
  geometry << "] 优化车体模型=" <<
    (dynamic_cast<PolygonRobotFootprint *>(cfg_->robot_model.get()) ? "多边形" : "其他");
  if (pass_with_penetration || pass_with_invalid) {
    const char * warning = pass_with_penetration ?
      "检查虽通过，但全轨迹诊断发现轮廓侵入；未改变原检查结果" :
      "检查虽通过，但全轨迹诊断存在无效距离；未改变原检查结果";
    RCLCPP_WARN(node_->get_logger(), "%s 警告=%s", summary.str().c_str(), warning);
    RCLCPP_WARN(node_->get_logger(), "%s", clearance.str().c_str());
    RCLCPP_WARN(node_->get_logger(), "%s", geometry.str().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "%s", summary.str().c_str());
    RCLCPP_INFO(node_->get_logger(), "%s", clearance.str().c_str());
    RCLCPP_INFO(node_->get_logger(), "%s", geometry.str().c_str());
  }
}

void TebOptimalPlanner::recordSweptFailure(
  int segment, double fraction, const PoseSE2 & pose, const char * reason)
{
  swept_failure_.segment = segment;
  swept_failure_.fraction = fraction;
  swept_failure_.pose = pose;
  swept_failure_.reason = reason;
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "[TEB 扫掠失败] 段=%d s=%.3f 位姿=(%.3f,%.3f,%.3f) 原因=%s",
    segment, fraction, pose.x(), pose.y(), pose.theta(), reason);
  if (visualization_) {
    visualization_->publishSweptObstacleDebug(teb_, swept_danger_segments_,
      swept_sample_poses_, swept_candidate_obstacles_, *cfg_->robot_model,
      segment, fraction, &pose, reason);
  }
}

bool TebOptimalPlanner::repairSweptFailure(const nav2_costmap_2d::Costmap2D * costmap)
{
  if (cfg_->swept_repair_tokens.fetch_sub(1, std::memory_order_relaxed) <= 0) {
    cfg_->swept_repair_tokens.store(0, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[TEB 扫掠修复] 本控制周期修复次数已用完，拒绝候选");
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  const int segment = swept_failure_.segment;
  if (segment < 0 || segment >= teb_.sizePoses() - 1) {return false;}
  forced_swept_segment_ = segment;
  forced_swept_fraction_ = swept_failure_.fraction;
  swept_repair_obstacles_.clear();
  const auto * polygon = dynamic_cast<const PolygonRobotFootprint *>(cfg_->robot_model.get());
  if (!polygon || polygon->vertices().size() < 3) {return false;}

  // costmap 检查可能发现 converter 尚未写入 obstacles_ 的障碍；只取失败区域的单元。
  if (costmap && swept_failure_.reason.find("costmap_") == 0) {
    double radius = 0.0;
    for (const auto & vertex : polygon->vertices()) {radius = std::max(radius, vertex.norm());}
    const auto & center = swept_failure_.pose.position();
    const double reach = radius + costmap->getResolution() +
      cfg_->obstacles.min_obstacle_dist;
    int mx0, my0, mx1, my1;
    costmap->worldToMapEnforceBounds(center.x() - reach, center.y() - reach, mx0, my0);
    costmap->worldToMapEnforceBounds(center.x() + reach, center.y() + reach, mx1, my1);
    struct Cell {double distance; double x; double y;};
    std::vector<Cell> cells;
    for (int my = my0; my <= my1; ++my) {
      for (int mx = mx0; mx <= mx1; ++mx) {
        const unsigned char cost = costmap->getCost(mx, my);
        if (cost < 253 || cost == 255) {continue;}
        double x, y;
        costmap->mapToWorld(mx, my, x, y);
        CircularObstacle cell(x, y, costmap->getResolution() * 0.707107);
        cells.push_back({cfg_->robot_model->optimizationDistance(
          swept_failure_.pose, &cell).signed_distance, x, y});
      }
    }
    std::sort(cells.begin(), cells.end(), [](const Cell & a, const Cell & b) {
      return a.distance < b.distance;
    });
    for (size_t i = 0; i < std::min<size_t>(12, cells.size()); ++i) {
      swept_repair_obstacles_.push_back(std::make_shared<CircularObstacle>(
        cells[i].x, cells[i].y, costmap->getResolution() * 0.707107));
    }
  }
  size_t nearby = 0;
  for (const auto & obst : *obstacles_) {
    if (obst && !obst->isWallGuide() && !obst->isDynamic() &&
        cfg_->robot_model->optimizationDistance(swept_failure_.pose, obst.get()).signed_distance <
        cfg_->trajectory.swept_obstacle_trigger_dist + cfg_->trajectory.swept_obstacle_margin)
    {
      ++nearby;
    }
  }
  if (nearby == 0 && swept_repair_obstacles_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[TEB 扫掠修复] 失败区域没有可用障碍几何，拒绝无效重试");
    return false;
  }
  swept_repair_active_ = true;
  const bool built = buildGraph();
  const bool optimized = built && optimizeGraph(
    std::max(1, cfg_->trajectory.swept_repair_inner_iterations), false);
  clearGraph();
  swept_repair_active_ = false;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  RCLCPP_WARN(node_->get_logger(),
    "[TEB 扫掠修复] 段=%d s=%.3f 原有近邻=%zu 代价图补点=%zu 建图=%d 优化=%d 耗时=%.1f ms 预算=%.1f ms",
    segment, forced_swept_fraction_, nearby, swept_repair_obstacles_.size(), built, optimized,
    elapsed_ms, cfg_->trajectory.swept_repair_budget_ms);
  return optimized && elapsed_ms <= cfg_->trajectory.swept_repair_budget_ms;
}

bool TebOptimalPlanner::isTrajectoryFeasible(dwb_critics::ObstacleFootprintCritic* costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance,
                                             const nav2_costmap_2d::Costmap2D* repair_costmap)
{
  auto reject = [&](int segment, double fraction, const PoseSE2 & pose, const char * reason) {
    recordSweptFailure(segment, fraction, pose, reason);
    if (!cfg_->trajectory.swept_obstacle_enable || swept_recheck_ ||
        cfg_->trajectory.swept_repair_max_attempts < 1 || segment < 0 ||
        std::string(reason) == "swept_check_budget_exhausted" ||
        !repairSweptFailure(repair_costmap)) {return false;}
    swept_recheck_ = true;
    const bool repaired = isTrajectoryFeasible(
      costmap_model, footprint_spec, inscribed_radius, circumscribed_radius,
      look_ahead_idx, feasibility_check_lookahead_distance, repair_costmap);
    swept_recheck_ = false;
    RCLCPP_WARN(node_->get_logger(), "[TEB 扫掠修复] 段=%d s=%.3f 复查=%s",
      segment, fraction, repaired ? "通过" : "仍碰撞");
    return repaired;
  };
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  if (feasibility_check_lookahead_distance > 0){
    for (int i=1; i < teb().sizePoses(); ++i){
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance){
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  geometry_msgs::msg::Pose2D pose2d;
  for (int i=0; i <= look_ahead_idx; ++i)
  {
    teb().Pose(i).toPoseMsg(pose2d);
    if (!isPoseValid(pose2d, costmap_model, footprint_spec)){
      logTrajectoryClearanceIfDue(false, "costmap_pose_rejected", i, footprint_spec);
      publishSamplingDebug("离散位姿碰撞");
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model);
      }
      return reject(i > 0 ? i - 1 : -1, i > 0 ? 1.0 : 0.0, teb().Pose(i), "costmap_pose_rejected");
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          intermediate_pose.toPoseMsg(pose2d);

          if (!isPoseValid(pose2d, costmap_model, footprint_spec)){
            logTrajectoryClearanceIfDue(false, "costmap_interpolation_rejected", i, footprint_spec);
            publishSamplingDebug("段内插值碰撞");
            if (visualization_)
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model);
            }
            return reject(i, static_cast<double>(step + 1) / (n_additional_samples + 1),
              intermediate_pose, "costmap_interpolation_rejected");
          }
        }
      }
    }
  }

  // 轮廓段内检查：只处理扫掠包围盒相交的静态障碍，避免全路径全障碍密采样。
  if (cfg_->trajectory.swept_obstacle_enable && obstacles_ && cfg_->robot_model &&
      dynamic_cast<const PolygonRobotFootprint *>(cfg_->robot_model.get()))
  {
    const auto * polygon = static_cast<const PolygonRobotFootprint *>(cfg_->robot_model.get());
    double radius = 0.0;
    for (const auto & v : polygon->vertices()) {radius = std::max(radius, v.norm());}
    const double pad = radius + std::max(0.0, cfg_->trajectory.feas_contour_margin);
    for (int i = 0; i < look_ahead_idx; ++i) {
      const auto & a = teb().Pose(i);
      const auto & b = teb().Pose(i + 1);
      const SweptAabb box{std::min(a.x(), b.x()) - pad, std::min(a.y(), b.y()) - pad,
        std::max(a.x(), b.x()) + pad, std::max(a.y(), b.y()) + pad};
      for (const auto & obstacle : *obstacles_) {
        if (!obstacle || obstacle->isWallGuide() || obstacle->isDynamic()) {continue;}
        SweptAabb obstacle_box;
        if (sweptAabbForObstacle(obstacle.get(), obstacle_box) &&
            !sweptAabbOverlap(box, obstacle_box)) {continue;}
        // 距离函数对刚体平移/旋转的变化不超过 L*ds；中点距离足够大时可证明整段安全。
        const double motion_bound = (b.position() - a.position()).norm() +
          radius * std::abs(g2o::normalize_theta(b.theta() - a.theta()));
        int evaluations = 0;
        double failed_fraction = 0.5;
        const char * failed_reason = "swept_contour_rejected";
        std::function<bool(double, double)> verify = [&](double left, double right) {
          const double mid = 0.5 * (left + right);
          failed_fraction = mid;
          if (++evaluations > std::max(1, cfg_->trajectory.swept_check_max_evals)) {
            failed_reason = "swept_check_budget_exhausted";
            return false;
          }
          const auto pose = sweptInterpolate(a, b, mid);
          const double distance = cfg_->robot_model->optimizationDistance(
            pose, obstacle.get()).signed_distance;
          if (distance <= cfg_->trajectory.feas_contour_margin) {
            failed_reason = "swept_contour_rejected";
            return false;
          }
          if (distance > cfg_->trajectory.feas_contour_margin +
              0.5 * motion_bound * (right - left)) {return true;}
          return verify(left, mid) && verify(mid, right);
        };
        if (!verify(0.0, 1.0)) {
          const auto pose = sweptInterpolate(a, b, failed_fraction);
          RCLCPP_ERROR(node_->get_logger(),
            "[TEB 扫掠复查] 段=%d s=%.4f 原因=%s 检查次数=%d",
            i, failed_fraction, failed_reason, evaluations);
          return reject(i, failed_fraction, pose, failed_reason);
        }
      }
    }
  }

  // Contour hard veto against obstacles_ (static + dynamic current; gated prediction)
  if (!isTrajectoryContourFeasible(inscribed_radius))
  {
    logTrajectoryClearanceIfDue(false, "contour_rejected", look_ahead_idx, footprint_spec);
    publishSamplingDebug("轮廓碰撞");
    return false;
  }
  logTrajectoryClearanceIfDue(true, "checks_passed", look_ahead_idx, footprint_spec);
  publishSamplingDebug("检查通过");
  if (visualization_ && cfg_->trajectory.swept_obstacle_enable) {
    visualization_->publishSweptObstacleDebug(teb_, swept_danger_segments_,
      swept_sample_poses_, swept_candidate_obstacles_, *cfg_->robot_model,
      -1, 0.0, nullptr, "检查通过");
  }
  return true;
}

bool TebOptimalPlanner::isTrajectoryContourFeasible(double inscribed_radius)
{
  if (!cfg_ || !obstacles_ || !cfg_->robot_model)
  {
    return true;
  }
  if (!cfg_->trajectory.feasibility_check_obstacle_contours &&
      !cfg_->trajectory.feasibility_check_dynamic_prediction)
  {
    return true;
  }
  if (teb_.sizePoses() < 1)
  {
    return true;
  }

  const double margin = std::max(0.0, cfg_->trajectory.feas_contour_margin);
  const double pip_radius = std::max(0.0, inscribed_radius);

  auto collidesCurrent = [&](const PoseSE2& pose, const Obstacle* obst) -> bool {
    if (cfg_->robot_model->calculateDistance(pose, obst) <= margin)
    {
      return true;
    }
    // Catch deep interior of large polygons (unsigned boundary distance can stay large)
    if (obst->checkCollision(pose.position(), std::max(margin, pip_radius)))
    {
      return true;
    }
    return false;
  };

  // --- A: full path, static contours + dynamic at current pose ---
  if (cfg_->trajectory.feasibility_check_obstacle_contours)
  {
    for (int i = 0; i < teb_.sizePoses(); ++i)
    {
      const PoseSE2& pose = teb_.Pose(i);
      for (const ObstaclePtr& obst : *obstacles_)
      {
        if (!obst)
        {
          continue;
        }
        if (collidesCurrent(pose, obst.get()))
        {
          if (visualization_)
          {
            visualization_->publishInfeasibleRobotPose(pose, *cfg_->robot_model);
          }
          if (node_)
          {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[FeasContour] current-contour collision at pose %d (dynamic=%d)",
                i, obst->isDynamic() ? 1 : 0);
          }
          return false;
        }
      }
    }
  }

  // --- B: gated short-horizon dynamic prediction ---
  if (!cfg_->trajectory.feasibility_check_dynamic_prediction || teb_.sizePoses() < 2)
  {
    return true;
  }

  const double T_pred = std::max(0.0, cfg_->trajectory.feas_pred_T);
  const double R_check = std::max(0.0, cfg_->trajectory.feas_pred_R);
  const double v_min = std::max(0.0, cfg_->trajectory.feas_pred_v_min);
  const double v_x_max = std::max(1e-3, cfg_->obstacles.dyn_gate_v_x_max);
  const double omega_ref = cfg_->near_horizon_runtime.omega_ref;
  double delta_rel = cfg_->trajectory.feas_pred_delta_omega_rel;
  if (delta_rel <= 0.0)
  {
    delta_rel = cfg_->obstacles.delta_omega_scale * cfg_->robot.max_vel_theta;
  }
  const double delta_abs = std::max(0.0, cfg_->trajectory.feas_pred_delta_omega_abs);
  const std::string& mode = cfg_->trajectory.feas_pred_gate_mode;

  // Omega gate over segments with cumulative time <= T_pred
  double max_abs_omega = 0.0;
  double max_rel_omega = 0.0;
  double t_acc = 0.0;
  for (int i = 0; i < teb_.sizePoses() - 1; ++i)
  {
    const double dt = teb_.TimeDiff(i);
    if (t_acc > T_pred + 1e-9)
    {
      break;
    }
    if (dt >= 1e-6)
    {
      const double angle_diff =
          g2o::normalize_theta(teb_.Pose(i + 1).theta() - teb_.Pose(i).theta());
      const double omega = angle_diff / dt;
      max_abs_omega = std::max(max_abs_omega, std::abs(omega));
      max_rel_omega = std::max(max_rel_omega, std::abs(omega - omega_ref));
    }
    t_acc += dt;
  }

  const bool gate_abs = (mode == "abs" || mode == "both_or") && (max_abs_omega > delta_abs);
  const bool gate_rel = (mode == "rel" || mode == "both_or") && (max_rel_omega > delta_rel);
  if (mode != "abs" && mode != "rel" && mode != "both_or")
  {
    // Unknown mode: treat as both_or
    if (!(max_abs_omega > delta_abs || max_rel_omega > delta_rel))
    {
      return true;
    }
  }
  else if (!gate_abs && !gate_rel)
  {
    return true;
  }

  const char* gate_reason = gate_abs && gate_rel ? "abs|rel" : (gate_abs ? "abs" : "rel");
  const Eigen::Vector2d robot0 = teb_.Pose(0).position();

  for (const ObstaclePtr& obst : *obstacles_)
  {
    if (!obst || !obst->isDynamic())
    {
      continue;
    }
    const Eigen::Vector2d v = obst->getCentroidVelocity();
    const double vn = v.norm();
    if (vn < v_min)
    {
      continue;
    }
    if ((obst->getCentroid() - robot0).norm() > R_check)
    {
      continue;
    }

    // Clamp velocity via effective time: x + v*t_eff ≈ x + v_clamped*t
    const double t_scale = (vn > v_x_max) ? (v_x_max / vn) : 1.0;
    Eigen::Vector2d v_clamped = v * t_scale;

    t_acc = 0.0;
    for (int i = 0; i < teb_.sizePoses(); ++i)
    {
      if (t_acc > T_pred + 1e-9)
      {
        break;
      }
      const PoseSE2& pose = teb_.Pose(i);
      const double t_eff = t_acc * t_scale;

      bool hit = false;
      if (cfg_->robot_model->estimateSpatioTemporalDistance(pose, obst.get(), t_eff) <= margin)
      {
        hit = true;
      }
      else
      {
        // Interior test in obstacle frame at t=0 equivalent to predicted obstacle at t
        const Eigen::Vector2d p_rel = pose.position() - v_clamped * t_acc;
        if (obst->checkCollision(p_rel, std::max(margin, pip_radius)))
        {
          hit = true;
        }
      }

      if (hit)
      {
        if (visualization_)
        {
          visualization_->publishInfeasibleRobotPose(pose, *cfg_->robot_model);
        }
        if (node_)
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[FeasContour] predicted-dynamic collision pose=%d t=%.2f gate=%s "
              "|w|=%.3f |w-wref|=%.3f (thr abs=%.3f rel=%.3f)",
              i, t_acc, gate_reason, max_abs_omega, max_rel_omega, delta_abs, delta_rel);
        }
        return false;
      }

      if (i < teb_.sizePoses() - 1)
      {
        t_acc += teb_.TimeDiff(i);
      }
    }
  }

  return true;
}

bool TebOptimalPlanner::isPoseValid(geometry_msgs::msg::Pose2D pose2d, dwb_critics::ObstacleFootprintCritic* costmap_model,
                           const std::vector<geometry_msgs::msg::Point>& footprint_spec)
{
  try {
    if ( costmap_model->scorePose(pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec)) < 0 ) {
      return false;
    }
  } catch(const dwb_core::IllegalTrajectoryException& e){
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[TEB 栅格碰撞检查] 非法轨迹异常，拒绝候选；位姿=(%.3f,%.3f,%.3f)，异常=%s",
      pose2d.x, pose2d.y, pose2d.theta, e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[TEB 栅格碰撞检查] 检查异常，拒绝候选；位姿=(%.3f,%.3f,%.3f)，异常=%s",
      pose2d.x, pose2d.y, pose2d.theta, e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[TEB 栅格碰撞检查] 未知异常，拒绝候选；位姿=(%.3f,%.3f,%.3f)",
      pose2d.x, pose2d.y, pose2d.theta);
    return false;
  }
  return true;
}

} // namespace teb_local_planner
