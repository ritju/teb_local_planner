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

 #include "teb_local_planner/teb_local_planner_ros.h"

 //#include <tf_conversions/tf_eigen.h>
 #include <boost/algorithm/string.hpp>
 
 #include <nav2_costmap_2d/cost_values.hpp>
 #include <string>
 
 // pluginlib macros
 #include <pluginlib/class_list_macros.hpp>
 
 #include "g2o/core/sparse_optimizer.h"
 #include "g2o/core/block_solver.h"
 #include "g2o/core/factory.h"
 #include "g2o/core/optimization_algorithm_gauss_newton.h"
 #include "g2o/core/optimization_algorithm_levenberg.h"
 #include "g2o/solvers/csparse/linear_solver_csparse.h"
 #include "g2o/solvers/cholmod/linear_solver_cholmod.h"
 
 #include <nav2_core/exceptions.hpp>
 #include <nav2_costmap_2d/footprint.hpp>
 #include <nav_2d_utils/tf_help.hpp>
 
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <tf2_eigen/tf2_eigen.hpp>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer_interface.h>
 #include "dwb_core/exceptions.hpp"
 #include "nav2_util/robot_utils.hpp"
 #include <cmath>
 
 using nav2_util::declare_parameter_if_not_declared;
 
 namespace teb_local_planner
 {
 
   const char* use_curb_or_wall = std::getenv("USE_CURB_OR_WALL");
   
 
 TebLocalPlannerROS::TebLocalPlannerROS() 
     : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), intra_proc_node_(nullptr),
                                            costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                            custom_via_points_active_(false), no_infeasible_plans_(0),
                                            last_preferred_rotdir_(RotType::none), initialized_(false),
                                            launch_max_vel_x_(0), launch_max_global_plan_lookahead_dist_(0),weight_via_point_(1.0),
                                            cfg_max_angular_vel_(0.6), cfg_max_angular_acc_(0.6), wall_line_update_time_(0),
                                            curb_line_update_time_(0), min_obstacle_dist_(0.5), wall_line_ptr_(nullptr), curb_line_subscriber_(nullptr),
                                            normal_weight_optimaltime_(2.0), normal_min_obstacle_dist_(0.2),
                                            normal_footprint_vertices_("[[1.25, 0.55], [1.25, -0.55], [-0.65, -0.55], [-0.65, 0.55]]"),
                                            edge_weight_optimaltime_(10.0), edge_min_obstacle_dist_(0.05),
                                            edge_footprint_vertices_("[[1.25, 0.5], [1.25, -0.5], [-0.65, -0.5], [-0.65, 0.5]]"),
                                            is_edge_following_mode_(false)
 {
   // Initialize edge mode parameters from config defaults (will be overridden by parameters if available)
   edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
   edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
   edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
 }
 
 
 TebLocalPlannerROS::~TebLocalPlannerROS()
 {
 }
 
 void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
 {
   // check if the plugin is already initialized
   if(!initialized_)
   {	
     // declare parameters (ros2-dashing)
     intra_proc_node_.reset( 
             new rclcpp::Node("costmap_converter", node->get_namespace(), 
               rclcpp::NodeOptions()));
     cfg_->declareParameters(node, name_);
 
     // get parameters of TebConfig via the nodehandle and override the default config
     cfg_->loadRosParamFromNodeHandle(node, name_);
     // 获取默认值
     launch_max_vel_x_ = cfg_->robot.max_vel_x;
     launch_max_global_plan_lookahead_dist_ = cfg_->trajectory.max_global_plan_lookahead_dist;
     weight_wall_line_direction_ = cfg_->optim.weight_wall_line_direction;
     weight_wall_line_dist_ = cfg_->optim.weight_wall_line_dist;
     weight_via_point_ = cfg_->optim.weight_viapoint;
     cfg_max_angular_vel_ = cfg_->robot.max_vel_theta;
     cfg_max_angular_acc_= cfg_->robot.acc_lim_theta;
     min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
     
     // Save normal mode parameters
     normal_weight_optimaltime_ = cfg_->optim.weight_optimaltime;
     normal_min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
     // Try to get footprint vertices from parameter server
     std::string footprint_string;
     if (node->get_parameter(name_ + "." + "footprint_model.vertices", footprint_string)) {
       normal_footprint_vertices_ = footprint_string;
     }
     
     // Load edge-following mode parameters from config
     edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
     edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
     edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
     // via_sep_ = cfg_->trajectory.global_plan_viapoint_sep;
     RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %f, max_vel_x: %f! In initialize!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);
 
     // reserve some memory for obstacles
     obstacles_.reserve(500);
         
     // create the planner instance
     if (cfg_->hcp.enable_homotopy_class_planning)
     {
       planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
       RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies enabled.");
     }
     else
     {
       planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
       RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");
     }
     
     // init other variables
     costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
     
     costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
     std::string costmap_model_name("costmap_model");
     costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);
 
     cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO
 
     //Initialize a costmap to polygon converter
     if (!cfg_->obstacles.costmap_converter_plugin.empty())
     {
       try
       {
         costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
         std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
         RCLCPP_INFO(logger_, "library path : %s", costmap_converter_loader_.getClassLibraryPath(cfg_->obstacles.costmap_converter_plugin).c_str());
         // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
         boost::replace_all(converter_name, "::", "/");
 
         costmap_converter_->setOdomTopic(cfg_->odom_topic);
         
         // Pass the main (lifecycle) node directly to costmap_converter so it can read parameters from controller node
         // Parameters will be read directly from the node's namespace (e.g., controller_server.FollowPath.cluster_max_distance)
         costmap_converter_->initialize(node);
         costmap_converter_->setCostmap2D(costmap_);
         const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
         costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
         RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", cfg_->obstacles.costmap_converter_plugin.c_str());
       }
       catch(pluginlib::PluginlibException& ex)
       {
         RCLCPP_INFO(logger_,
                     "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
         costmap_converter_.reset();
       }
     }
     else {
       RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
     }
   
     
     // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
     footprint_spec_ = costmap_ros_->getRobotFootprint();
     nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
 
     // Add callback for dynamic parameters
     dyn_params_handler = node->add_on_set_parameters_callback(
       std::bind(&TebConfig::dynamicParametersCallback, std::ref(cfg_), std::placeholders::_1));
 
     // validate optimization footprint and costmap footprint
     validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);
         
     // setup callback for custom obstacles
     custom_obst_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
                 "obstacles", 
                 rclcpp::SystemDefaultsQoS(),
                 std::bind(&TebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));
 
     // setup callback for custom via-points
     via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                 "via_points", 
                 rclcpp::SystemDefaultsQoS(),
                 std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));
     // setup callback for line center points
     if (cfg_->optim.weight_wall_line_dist > 0.0)
     {
       if (use_curb_or_wall != nullptr && std::strcmp(use_curb_or_wall, "WALL") == 0)
       {
         wall_line_ptr_ = std::make_shared<line_path_compare::LinePathCompare>(node);
       }
       else if (use_curb_or_wall != nullptr && std::strcmp(use_curb_or_wall, "CURB") == 0)
       {
         curb_line_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
                 "camera1/extracted_line_path", 
                 rclcpp::QoS{5}.best_effort(),
                 std::bind(&TebLocalPlannerROS::curb_line_callback, this, std::placeholders::_1));
       }
     
     wall_line_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("teb_selected_wall_line", 1);
     edge_distance_publisher_  = node->create_publisher<std_msgs::msg::Float32>("edge_distance", 1);
     // initialize failure detector
     //rclcpp::Node::SharedPtr nh_move_base("~");
     double controller_frequency = 5;
     node->get_parameter("controller_frequency", controller_frequency);
     failure_detector_.setBufferLength(std::round(cfg_->recovery.oscillation_filter_duration*controller_frequency));
     
     // set initialized flag
     initialized_ = true;
 
     // This should be called since to prevent different time sources exception
     time_last_infeasible_plan_ = clock_->now();
     time_last_oscillation_ = clock_->now();
     RCLCPP_DEBUG(logger_, "teb_local_planner plugin initialized.");
     
     transformed_path = node->create_publisher<nav_msgs::msg::Path>("teb_transformed_path", 1);
     global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("teb_global_plan", 1);
   }
   else
   {
     RCLCPP_INFO(logger_, "teb_local_planner has already been initialized, doing nothing.");
   }
 }}
 
 void TebLocalPlannerROS::configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name,
     std::shared_ptr<tf2_ros::Buffer> tf,
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
   nh_ = parent;
 
   auto node = nh_.lock();
   logger_ = node->get_logger();
   clock_ = node->get_clock();
 
   costmap_ros_ = costmap_ros;
   tf_ = tf;
   name_ = name;
 
   initialize(node);
   visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
   visualization_->on_configure();
   planner_->setVisualization(visualization_);
   
   return;
 }
 
 void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path & orig_global_plan)
 {
   // check if plugin is initialized
   if(!initialized_)
   {
     RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
     return;
   }
 
   // store the global plan
   global_plan_.clear();
   global_plan_.reserve(orig_global_plan.poses.size());
   origin_plan_.clear();
   origin_plan_.reserve(orig_global_plan.poses.size());
   for(const auto &in_pose :orig_global_plan.poses) {
     geometry_msgs::msg::PoseStamped out_pose;
     out_pose.pose = in_pose.pose;
     out_pose.header = orig_global_plan.header;
     global_plan_.push_back(out_pose);
     origin_plan_.push_back(out_pose);
   }
 
 
   // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
   // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
             
   return;
 }
 
 
 geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
   const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *goal_checker)
 {
   // check if plugin initialized
   if(!initialized_)
   {
     throw nav2_core::PlannerException(
       std::string("teb_local_planner has not been initialized, please call initialize() before using this planner")
     );
   }
 
   geometry_msgs::msg::TwistStamped cmd_vel;
   
   cmd_vel.header.stamp = clock_->now();
   cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
   cmd_vel.twist.linear.x = 0;
   cmd_vel.twist.linear.y = 0;
   cmd_vel.twist.angular.z = 0;
 
   // Update for the current goal checker's state
   geometry_msgs::msg::Pose pose_tolerance;
   geometry_msgs::msg::Twist vel_tolerance;
   if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
     RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
   } else {
     cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
   }
   
   // Get robot pose
   robot_pose_ = PoseSE2(pose.pose);
   geometry_msgs::msg::PoseStamped robot_pose;
   robot_pose.header = pose.header;
   robot_pose_.toPoseMsg(robot_pose.pose);
   
   // Get robot velocity
   robot_vel_ = velocity;
   
   // prune global plan to cut off parts of the past (spatially before the robot)
   pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance);
   // pruneGlobalPlan(robot_pose, origin_plan_, 3);
   geometry_msgs::msg::PoseStamped corner_pose_global, corner_pose_robot;
   bool corner_found = false;
   size_t corner_index = 0;  // Index of corner in global_plan_
   
   if (global_plan_.size() < 30)
   {
     cfg_->trajectory.max_global_plan_lookahead_dist = launch_max_global_plan_lookahead_dist_;
     cfg_->robot.max_vel_x = launch_max_vel_x_;
     // Initialize corner poses to avoid using uninitialized variables
     corner_pose_global = robot_pose;
     corner_pose_robot = robot_pose;
     // RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %f, max_vel_x: %f! Set succed !", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);
   }
   else
   {
     double theta = cfg_->trajectory.theta_threshold;
     const double min_segment_length = 1e-6;  // Minimum segment length to avoid division by zero
     const double position_tolerance = 0.01;  // Tolerance for position comparison
     
     // Find the nearest corner point that satisfies the condition
     for (size_t i = 2; i < global_plan_.size() - 2 && i < cfg_->trajectory.pose_num_threshold; ++i)
     {
       double x0 = global_plan_.at(i).pose.position.x;
       double y0 = global_plan_.at(i).pose.position.y;
       double x1 = global_plan_.at(i - 2).pose.position.x;
       double y1 = global_plan_.at(i - 2).pose.position.y;
       double x2 = global_plan_.at(i + 2).pose.position.x;
       double y2 = global_plan_.at(i + 2).pose.position.y;
 
       // Calculate triangle side lengths for geometric cosine theorem
       // Three points form a triangle: x1 -> x0 -> x2
       // Calculate the angle at x0 using cosine theorem: cos(θ) = (a² + b² - c²) / (2ab)
       double dx_a = x0 - x1;  // Side a: from x1 to x0
       double dy_a = y0 - y1;
       double dx_b = x2 - x0;  // Side b: from x0 to x2
       double dy_b = y2 - y0;
       double dx_c = x2 - x1;  // Side c: from x1 to x2 (opposite to angle at x0)
       double dy_c = y2 - y1;
       
       double length_a = std::sqrt(dx_a * dx_a + dy_a * dy_a);  // |x1 - x0|
       double length_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);  // |x0 - x2|
       double length_c = std::sqrt(dx_c * dx_c + dy_c * dy_c);  // |x1 - x2|
       
       // Check for zero-length segments to avoid division by zero
       if (length_a < min_segment_length || length_b < min_segment_length)
       {
         continue;
       }
       
       // Geometric cosine theorem: cos(θ) = (a² + b² - c²) / (2ab)
       // The result is naturally constrained to [0, 180] degrees
       double a_sq = length_a * length_a;
       double b_sq = length_b * length_b;
       double c_sq = length_c * length_c;
       double cos_theta = (a_sq + b_sq - c_sq) / (2.0 * length_a * length_b);
       
       // Clamp cos_theta to valid range [-1, 1] to avoid NaN from acos due to numerical errors
       cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
       
       // Calculate the turning angle using arccosine (0-180 degrees)
       // For sharp corners, we want small angles (acute)
       double temp_theta = std::acos(cos_theta) * 180.0 / M_PI;      
       // Check if this point is different from last_corner_pose_ (using distance tolerance)
       bool is_different_from_last = true;
       if (last_corner_pose_.header.frame_id == global_plan_.at(0).header.frame_id)
       {
         double dx = global_plan_.at(i).pose.position.x - last_corner_pose_.pose.position.x;
         double dy = global_plan_.at(i).pose.position.y - last_corner_pose_.pose.position.y;
         double dist_sq = dx * dx + dy * dy;
         if (dist_sq < position_tolerance * position_tolerance)
         {
           is_different_from_last = false;
         }
       }
       
       if (temp_theta < theta && is_different_from_last)
       {
         theta = temp_theta;
         corner_pose_global = global_plan_.at(i);
         corner_index = i;
         corner_found = true;
         break;  // Find the nearest corner that satisfies the condition
       }
     }
     
     if (corner_found)
     {
       corner_pose_global.header.frame_id = global_plan_.at(0).header.frame_id;
       // Use TimePointZero to get the latest available transform, avoiding extrapolation errors
       try 
       {
         geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
           corner_pose_global.header.frame_id, 
           costmap_ros_->getBaseFrameID(), 
           tf2::TimePointZero,
           tf2::durationFromSec(0.5));
         corner_pose_global.header.stamp = transform.header.stamp;
       } 
       catch (const tf2::ExtrapolationException& ex) 
       {
         // If TimePointZero fails with extrapolation, try to get latest common time
         try {
           rclcpp::Time latest_time;
           if (tf_->canTransform(corner_pose_global.header.frame_id, costmap_ros_->getBaseFrameID(), tf2::TimePointZero)) {
             geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
               corner_pose_global.header.frame_id, 
               costmap_ros_->getBaseFrameID(), 
               tf2::TimePointZero);
             corner_pose_global.header.stamp = transform.header.stamp;
           } else {
             RCLCPP_WARN(logger_, "TF 查询失败 (ExtrapolationException): %s, 尝试使用最新可用时间", ex.what());
             corner_found = false;
           }
         } catch (const tf2::TransformException& ex2) {
           RCLCPP_ERROR(logger_, "TF 查询失败: %s", ex2.what());
           corner_found = false;
         }
       }
       catch (const tf2::TransformException &ex) 
       {
         RCLCPP_ERROR(logger_, "TF 查询失败: %s", ex.what());
         corner_found = false;  // Mark corner as invalid if TF fails
       }
       
       if (corner_found)
       {
         // transformPoseInTargetFrame uses TimePointZero internally, which should be safe
         if (nav2_util::transformPoseInTargetFrame(corner_pose_global, corner_pose_robot, *tf_, costmap_ros_->getBaseFrameID()))
         {
           if (theta < cfg_->trajectory.theta_threshold && corner_pose_robot.pose.position.x > 0)
           {
             cfg_->trajectory.max_global_plan_lookahead_dist = cfg_->trajectory.min_global_plan_lookahead_dist_threshold;
             cfg_->robot.max_vel_x = cfg_->trajectory.min_vel_x_threshold;
             // RCLCPP_INFO(logger_, "Theta: %f! Set theta succed !", theta);
             // RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %f, max_vel_x: %f! In small theta!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);
             //设置 max_global_plan_lookahead_dist、max_vel_x 为较小值
           }
           else
           {
             cfg_->trajectory.max_global_plan_lookahead_dist = launch_max_global_plan_lookahead_dist_;
             cfg_->robot.max_vel_x = launch_max_vel_x_;
             // RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %f, max_vel_x: %f! In big theta!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);
           }
         }
         else
         {
           corner_found = false;  // Mark corner as invalid if transform fails
           cfg_->trajectory.max_global_plan_lookahead_dist = launch_max_global_plan_lookahead_dist_;
           cfg_->robot.max_vel_x = launch_max_vel_x_;
         }
       }
     }
     else
     {
       // No corner found, initialize corner poses
       corner_pose_global = robot_pose;
       corner_pose_robot = robot_pose;
       cfg_->trajectory.max_global_plan_lookahead_dist = launch_max_global_plan_lookahead_dist_;
       cfg_->robot.max_vel_x = launch_max_vel_x_;
     }
   }
   
   // RCLCPP_INFO(logger_, "max_global_plan_lookahead_dist %f, max_vel_x: %f!", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);
   
   // Record corner pose if it's close enough
   if (corner_found)
   {
     double corner_pose_dist = corner_pose_robot.pose.position.x * corner_pose_robot.pose.position.x + 
            corner_pose_robot.pose.position.y * corner_pose_robot.pose.position.y;
     if (corner_pose_robot.pose.position.x > 0 && corner_pose_robot.pose.position.x < 0.3 && corner_pose_dist < 0.25)
     {
       last_corner_pose_ = corner_pose_global;
     }
   }
 
   // Prune path before last corner using distance tolerance
   if (last_corner_pose_.header.frame_id == global_plan_.at(0).header.frame_id)
   {
     const double prune_position_tolerance = 0.05;  // Tolerance for finding corner in path
     for (auto prune_before_last_corner = global_plan_.begin(); 
          prune_before_last_corner != global_plan_.end() && 
          prune_before_last_corner != global_plan_.begin() + 40; 
          ++prune_before_last_corner)
     {
       double dx = prune_before_last_corner->pose.position.x - last_corner_pose_.pose.position.x;
       double dy = prune_before_last_corner->pose.position.y - last_corner_pose_.pose.position.y;
       double dist_sq = dx * dx + dy * dy;
       
       if (dist_sq < prune_position_tolerance * prune_position_tolerance)
       {
         if (global_plan_.end() - prune_before_last_corner > 10)
         {
           global_plan_.erase(global_plan_.begin(), prune_before_last_corner);
         }
         break;
       }
     }
   }
 
 
   // Publish global_plan_ for visualization/debugging
   if (global_plan_pub_ && !global_plan_.empty())
   {
     nav_msgs::msg::Path global_plan_msg;
     global_plan_msg.header.stamp = clock_->now();
     global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
     global_plan_msg.poses = global_plan_;
     global_plan_pub_->publish(global_plan_msg);
   }
 
   // Transform global plan to the frame of interest (w.r.t. the local costmap)
   std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
   int goal_idx;
   geometry_msgs::msg::TransformStamped tf_plan_to_global;
   if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                            transformed_plan, &goal_idx, &tf_plan_to_global))
   {
     throw nav2_core::PlannerException(
       std::string("Could not transform the global plan to the frame of the controller")
     );
   }
   // Check if corner has obstacle and prune path before corner if needed
   if (corner_found && corner_pose_robot.pose.position.x < cfg_->trajectory.max_global_plan_lookahead_dist && corner_pose_robot.pose.position.x > 0.3)
   {
     geometry_msgs::msg::PoseStamped local_corner_check_pose2d;
     tf2::doTransform(corner_pose_global, local_corner_check_pose2d, tf_plan_to_global);
     
     // Get robot pose in local costmap frame (from transformed_plan, which starts with robot pose)
     geometry_msgs::msg::Pose2D robot_pose_local;
     if (!transformed_plan.empty())
     {
       robot_pose_local.x = transformed_plan.front().pose.position.x;
       robot_pose_local.y = transformed_plan.front().pose.position.y;
       robot_pose_local.theta = tf2::getYaw(transformed_plan.front().pose.orientation);
     }
     else
     {
       // Fallback: robot is at origin in local costmap frame
       robot_pose_local.x = 0.0;
       robot_pose_local.y = 0.0;
       robot_pose_local.theta = 0.0;
     }
     
     // Calculate direction from robot to corner
     double dx = local_corner_check_pose2d.pose.position.x - robot_pose_local.x;
     double dy = local_corner_check_pose2d.pose.position.y - robot_pose_local.y;
     double direction_to_corner = std::atan2(dy, dx);
     
     geometry_msgs::msg::Pose2D corner_check_pose2d;
     corner_check_pose2d.x = local_corner_check_pose2d.pose.position.x;
     corner_check_pose2d.y = local_corner_check_pose2d.pose.position.y;
     corner_check_pose2d.theta = direction_to_corner;  // Use direction from robot to corner
     
     bool corner_has_obstacle = false;
     try
     {
       unsigned char corner_check_cost = costmap_model_->scorePose(corner_check_pose2d, dwb_critics::getOrientedFootprint(corner_check_pose2d, footprint_spec_));
       if (corner_check_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
       {
         corner_has_obstacle = true;
       }
     }
     catch(const dwb_core::IllegalTrajectoryException& e)
     {
       if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
       {
         corner_has_obstacle = true;
       }
     }
     
     // If corner has obstacle, delete all path points before the corner in global_plan_
     if (corner_has_obstacle)
     {
       if (corner_index > 0 && corner_index < global_plan_.size() && corner_pose_robot.pose.position.x < std::max(cfg_->trajectory.max_global_plan_lookahead_dist / 2.0, 2.0))
       {
         // Erase all points before the corner (keep the corner point itself)
         global_plan_.erase(global_plan_.begin(), global_plan_.begin() + corner_index);
       }
       goto jump_prune_transformed_plan;
     }
     
     // If no obstacle, find corner in transformed_plan and prune after corner if needed
     const double corner_position_tolerance = 0.05;  // Tolerance for finding corner in transformed plan
     for (auto check_it = transformed_plan.begin(); 
          check_it != transformed_plan.end() && check_it != transformed_plan.begin() + 80; 
          ++check_it)
     {
       double dx = check_it->pose.position.x - local_corner_check_pose2d.pose.position.x;
       double dy = check_it->pose.position.y - local_corner_check_pose2d.pose.position.y;
       double dist_sq = dx * dx + dy * dy;
       
       if (dist_sq < corner_position_tolerance * corner_position_tolerance)
       {
         auto front_differance = check_it - transformed_plan.begin();
         auto end_differance = transformed_plan.end() - check_it;
         // Use a reasonable threshold (10 points) instead of cfg_->max_samples which may not exist
         if (front_differance > cfg_->trajectory.min_samples && end_differance > cfg_->trajectory.min_samples)
         {
           transformed_plan.erase(check_it, transformed_plan.end());
         }
         break;
       }
     }
   }
 
   jump_prune_transformed_plan:
   if (use_curb_or_wall != nullptr && std::strcmp(use_curb_or_wall, "WALL") == 0)
   {
     nav_msgs::msg::Path input_path;
     input_path.poses = (transformed_plan.size() > 40 ? std::vector<geometry_msgs::msg::PoseStamped>(transformed_plan.begin(), transformed_plan.begin() + 40) : transformed_plan);
     
     if (transformed_plan.size() > 0)
     {
       input_path.header = transformed_plan.at(0).header;
     }
     transformed_path->publish(input_path);
     std::vector<nav_msgs::msg::Path> path_from_wall_line;
     path_from_wall_line = wall_line_ptr_->get_compare_result(input_path);
     updateWallLineVec(path_from_wall_line, input_path, cfg_->wall_line.parallel_tolerance, cfg_->wall_line.distance_tolerance, robot_pose);
   }
   else if (use_curb_or_wall != nullptr && std::strcmp(use_curb_or_wall, "CURB") == 0)
   {
     // 检查curb_line_path_的时间戳，如果超过2秒则清空
     {
       std::lock_guard<std::mutex> l(curb_line_mutex_);
       if (curb_line_path_.poses.size() > 0)
       {
         auto time_diff = clock_->now() - curb_line_update_time_;
         if (time_diff.seconds() > 2.0)
         {
           curb_line_path_.poses.clear();
         }
       }
     }
     
     nav_msgs::msg::Path input_path;
     input_path.poses = (transformed_plan.size() > 40 ? std::vector<geometry_msgs::msg::PoseStamped>(transformed_plan.begin(), transformed_plan.begin() + 40) : transformed_plan);
     
     if (transformed_plan.size() > 0)
     {
       input_path.header = transformed_plan.at(0).header;
     }
     transformed_path->publish(input_path);
     updateCurbLineVec(curb_line_path_, input_path, cfg_->wall_line.parallel_tolerance, cfg_->wall_line.distance_tolerance, robot_pose);
   }
   
   // if (wall_line_points_.size() == 0)
   // {
   //   RCLCPP_INFO(logger_, "Can not found useful wall_line_points_ !");
   // }
 
   // update via-points container
   if (!custom_via_points_active_)
     updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);
   // check if we should enter any backup mode and apply settings
   configureBackupModes(transformed_plan, goal_idx);
     
   // Return false if the transformed global plan is empty
   if (transformed_plan.empty())
   {
     throw nav2_core::PlannerException(
       std::string("Transformed plan is empty. Cannot determine a local plan.")
     );
   }
               
   // Get current goal point (last point of the transformed plan)
   const geometry_msgs::msg::PoseStamped &goal_point = transformed_plan.back();
   robot_goal_.x() = goal_point.pose.position.x;
   robot_goal_.y() = goal_point.pose.position.y;
   if (cfg_->trajectory.global_plan_overwrite_orientation)
   {
     robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
     // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
     //transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
     tf2::Quaternion q;
     q.setRPY(0, 0, robot_goal_.theta());
     transformed_plan.back().pose.orientation = tf2::toMsg(q);
   }  
   else
   {
     robot_goal_.theta() = tf2::getYaw(goal_point.pose.orientation);
   }
 
   // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
   if (transformed_plan.size()==1) // plan only contains the goal
   {
     transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
   }
   //tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;
   transformed_plan.front().pose = robot_pose.pose;
     
   // clear currently existing obstacles
   obstacles_.clear();
   
   // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
   if (costmap_converter_)
     updateObstacleContainerWithCostmapConverter();
   else
     updateObstacleContainerWithCostmap();
   
   // also consider custom obstacles (must be called after other updates, since the container is not cleared)
   updateObstacleContainerWithCustomObstacles();
   
     
   // Do not allow config changes during the following optimization step
   std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
     
   // Now perform the actual planning
 //   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_->goal_tolerance.free_goal_vel); // straight line init
   // RCLCPP_INFO(logger_, "Weight_via_point: %f !", cfg_->optim.weight_viapoint);
   bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
   if (!success)
   {
     planner_->clearPlanner(); // force reinitialization for next time
     
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("teb_local_planner was not able to obtain a local plan for the current setting.")
     );
   }
 
   
 
   // Check for divergence
   if (planner_->hasDiverged())
   {
     cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
 
     // Reset everything to start again with the initialization of new trajectories.
     planner_->clearPlanner();
     RCLCPP_WARN_THROTTLE(logger_, *(clock_), 1, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");
 
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: velocity command invalid (hasDiverged). Resetting planner...")
     );
   }
          
   // Check feasibility (but within the first few states only)
   if(cfg_->robot.is_footprint_dynamic)
   {
     // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
     std::vector<geometry_msgs::msg::Point> updated_footprint_spec_ = costmap_ros_->getRobotFootprint();
     if (updated_footprint_spec_ != footprint_spec_) {
       updated_footprint_spec_ = footprint_spec_;
       nav2_costmap_2d::calculateMinAndMaxDistances(updated_footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
     }
   }
 
   bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_->trajectory.feasibility_check_no_poses, cfg_->trajectory.feasibility_check_lookahead_distance);
   if (!feasible)
   {
     cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    
     // now we reset everything to start again with the initialization of new trajectories.
     planner_->clearPlanner();
 
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
     );
   }
 
   // Get the velocity command for this sampling interval
   if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
   {
     planner_->clearPlanner();
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
     );
   }
   
   // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
   saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                    cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);
 
   // convert rot-vel to steering angle if desired (carlike robot).
   // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
   // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
   if (cfg_->robot.cmd_angle_instead_rotvel)
   {
     cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
     if (!std::isfinite(cmd_vel.twist.angular.z))
     {
       cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
       last_cmd_ = cmd_vel.twist;
       planner_->clearPlanner();
 
       ++no_infeasible_plans_; // increase number of infeasible solutions in a row
       time_last_infeasible_plan_ = clock_->now();
       
       throw nav2_core::PlannerException(
         std::string("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...")
       );
     }
   }
   
   // a feasible solution should be found, reset counter
   no_infeasible_plans_ = 0;
   
   // store last command (for recovery analysis etc.)
   last_cmd_ = cmd_vel.twist;
   
   // Now visualize everything    
   planner_->visualize();
   visualization_->publishObstacles(obstacles_);
   visualization_->publishViaPoints(via_points_);
   visualization_->publishGlobalPlan(global_plan_);
   
   return cmd_vel;
 }
 
 void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
 {  
   // Add costmap obstacles if desired
   if (cfg_->obstacles.include_costmap_obstacles)
   {
     std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());
 
     Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
     
     for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
     {
       for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
       {
         if (costmap_->getCost(i,j) == nav2_costmap_2d::LETHAL_OBSTACLE)
         {
           Eigen::Vector2d obs;
           costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
             
           // check if obstacle is interesting (e.g. not far behind the robot)
           Eigen::Vector2d obs_dir = obs-robot_pose_.position();
           if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist  )
             continue;
             
           obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
         }
       }
     }
   }
 }
 
 void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
 {
   if (!costmap_converter_)
     return;
     
   //Get obstacles from costmap converter
   costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
   if (!obstacles)
     return;
 
   for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
   {
     const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
     const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;
 
     if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
     {
       obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
     }
     else if (polygon->points.size()==1) // Point
     {
       obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
     }
     else if (polygon->points.size()==2) // Line
     {
       obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                         polygon->points[1].x, polygon->points[1].y )));
     }
     else if (polygon->points.size()>2) // Real polygon
     {
         PolygonObstacle* polyobst = new PolygonObstacle;
         for (std::size_t j=0; j<polygon->points.size(); ++j)
         {
             polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
         }
         polyobst->finalizePolygon();
         obstacles_.push_back(ObstaclePtr(polyobst));
     }
 
     // Set velocity, if obstacle is moving
     if(!obstacles_.empty())
       obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
   }
 }
 
 
 void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
 {
   // Add custom obstacles obtained via message
   std::lock_guard<std::mutex> l(custom_obst_mutex_);
 
   if (!custom_obstacle_msg_.obstacles.empty())
   {
     // We only use the global header to specify the obstacle coordinate system instead of individual ones
     Eigen::Affine3d obstacle_to_map_eig;
     try 
     {
       // Use TimePointZero to get the latest available transform, avoiding extrapolation errors
       geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(
                   cfg_->map_frame,
                   custom_obstacle_msg_.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
       //tf2::fromMsg(obstacle_to_map.transform, obstacle_to_map_eig);
     }
     catch (const tf2::ExtrapolationException& ex)
     {
       RCLCPP_WARN(logger_, "updateObstacleContainerWithCustomObstacles: ExtrapolationException: %s, using identity transform", ex.what());
       obstacle_to_map_eig.setIdentity();
     }
     catch (const tf2::TransformException& ex)
     {
       RCLCPP_ERROR(logger_, "updateObstacleContainerWithCustomObstacles: TransformException: %s, using identity transform", ex.what());
       obstacle_to_map_eig.setIdentity();
     }
     
     for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
     {
       if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
       {
         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
       {
         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
       {
         Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
         Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
         obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                            (obstacle_to_map_eig * line_end).head(2) )));
       }
       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
       {
         RCLCPP_INFO(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
         continue;
       }
       else // polygon
       {
         PolygonObstacle* polyobst = new PolygonObstacle;
         for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
         {
           Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
           polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
         }
         polyobst->finalizePolygon();
         obstacles_.push_back(ObstaclePtr(polyobst));
       }
 
       // Set velocity, if obstacle is moving
       if(!obstacles_.empty())
         obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
     }
   }
 }
 
 void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
 {
   via_points_.clear();
   
   if (min_separation<=0)
     return;
   ViaPointContainer occupied_points;
   std::size_t prev_idx = 0;
   for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
   {
     // check separation to the previous via-point inserted
     if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
       continue;
     unsigned int mx = 0;
     unsigned int my = 0;
     if (costmap_->worldToMap(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, mx, my) && costmap_->getCost(mx, my) >= nav2_costmap_2d::MAX_NON_OBSTACLE)
     {
       // add via-point
       via_points_.clear();
       break;
     }
     via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
     prev_idx = i;
   } 
 }
 
 void TebLocalPlannerROS::updateWallLineVec(
   const std::vector<nav_msgs::msg::Path>& wall_line, 
   nav_msgs::msg::Path& input_path,
   const double parallel_tolerance,
   const double distance_tolerance,
   const geometry_msgs::msg::PoseStamped& robot_pose)
 {
   std::lock_guard<std::mutex> l(update_wall_line_mutex_);
   if (wall_line.size() == 0 || input_path.poses.size() < 2)
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }
   if (input_path.poses.size() >= 2) {
     // 提取机器人当前朝向（从四元数转换为偏航角）
     double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
     // 提取输入路径的首尾点
     const auto& start_pose = input_path.poses.front().pose.position;
     const auto& end_pose = input_path.poses.back().pose.position;
     
     // 计算路径方向向量（从起点指向终点）
     const double dx_path = end_pose.x - start_pose.x;
     const double dy_path = end_pose.y - start_pose.y;
     const double path_length = std::hypot(dx_path, dy_path);
     
     if (path_length > 1e-6) {
       // 计算路径方向角度（相对于世界坐标系）
       const double path_yaw = std::atan2(dy_path, dx_path);
       
       // 计算机器人朝向与路径方向的夹角
       double angle_diff = path_yaw - robot_yaw;
       
       // 归一化角度到[-π, π]范围
       while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
       while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
       
       // 检查夹角是否超出±90度范围
       if (std::fabs(angle_diff) > M_PI / 4) {
         if(wall_line_points_.size() > 0) wall_line_points_.clear();
         switchParameterMode(false);
         return;
       }
     }
   }
 
 
   if (wall_line.size() == 0 || input_path.poses.size() < 2)
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }  
   // 1. 提取输入路径的首尾点
   const auto& start_pose = input_path.poses.front().pose.position;
   const auto& end_pose = input_path.poses.back().pose.position;
   
   // 2. 计算输入路径方向向量
   const double dx_input = end_pose.x - start_pose.x;
   const double dy_input = end_pose.y - start_pose.y;
   const double input_length = std::hypot(dx_input, dy_input);
   if (input_length < 1e-6)
   {
     if(wall_line_points_.size() > 0)
     {
       auto time_diff  = clock_->now() - wall_line_update_time_;
       if (time_diff.seconds() > 1.0)
       {
         wall_line_points_.clear();
       }
     }
     switchParameterMode(false);
     return;
   }  
   
   const double dir_input_x = dx_input / input_length;
   const double dir_input_y = dy_input / input_length;
 
   // 3. 遍历所有墙线
   bool found_valid_wall = false;
   double min_distance = std::numeric_limits<double>::max();
   nav_msgs::msg::Path best_wall_path;
   geometry_msgs::msg::Point best_wall_start, best_wall_end;
   double robot_to_edge_distance = 2.0;
   for(const auto& wall_path : wall_line) 
   {
     if(wall_path.poses.size() < 2) continue; // 无效墙线
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Wall_path.poses.size(): %ld !", wall_path.poses.size());
     
     // 4. 提取墙线端点
     const auto& wall_start = wall_path.poses.front().pose.position;
     const auto& wall_end = wall_path.poses.back().pose.position;
     
     // 5. 计算墙线方向向量
     const double dx_wall = wall_end.x - wall_start.x;
     const double dy_wall = wall_end.y - wall_start.y;
     const double wall_length = std::hypot(dx_wall, dy_wall);
     if(wall_length < 1e-6) continue; // 无效墙线
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Wall_length: %f !", wall_length);
     
     const double dir_wall_x = dx_wall / wall_length;
     const double dir_wall_y = dy_wall / wall_length;
     
     // 计算平行度（余弦值）
     const double dot_product = dir_input_x * dir_wall_x + dir_input_y * dir_wall_y;
     const double cos_theta = std::fabs(dot_product);
     const double parallel_threshold = std::cos(parallel_tolerance / 180 * M_PI);
     
     // 检查平行度是否满足要求
     if(cos_theta < parallel_threshold) continue;
     
     // 计算垂直距离
     const double A = dy_wall;
     const double B = -dx_wall;
     const double C = wall_end.x * wall_start.y - wall_start.x * wall_end.y;
     
     const double numerator = std::fabs(A * start_pose.x + B * start_pose.y + C);
     const double denominator = std::hypot(A, B);
     const double avg_distance = numerator / denominator;
     robot_to_edge_distance = std::fabs(A * robot_pose.pose.position.x + B * robot_pose.pose.position.y + C) / std::hypot(A, B);
     
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Wall path - Parallelism: %f, Distance: %f", cos_theta, avg_distance);
     
     // 记录满足平行度且距离最小的墙线
     if (avg_distance <= distance_tolerance && avg_distance < min_distance) 
     {
       found_valid_wall = true;
       min_distance = avg_distance;
       best_wall_path = wall_path;
       best_wall_start = wall_start;
       best_wall_end = wall_end;
     }
   }
 
   // 5. 如果找到最佳匹配的墙线，更新配置
   if (found_valid_wall) 
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
     cfg_->optim.weight_wall_line_dist = weight_wall_line_dist_;
     if (fabs(cfg_->wall_line.distance_tolerance - cfg_->wall_line.min_wall_dist) > 1e-5){
       cfg_->optim.weight_wall_line_dist = std::min(weight_wall_line_dist_, std::max(1.0, std::fabs((1.0 - weight_wall_line_dist_) / (cfg_->wall_line.distance_tolerance - cfg_->wall_line.min_wall_dist) * (min_distance - cfg_->wall_line.min_wall_dist) + weight_wall_line_dist_)));
     }
     wall_line_update_time_ = clock_->now();
     std_msgs::msg::Float32 distance;
     distance.data = robot_to_edge_distance;
     edge_distance_publisher_->publish(distance);
     
     // Switch to edge-following mode parameters
     switchParameterMode(true);
     
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Selected best wall line - Distance: %f", min_distance);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Min obstacle distance: %f, Weight optimal time: %f", cfg_->obstacles.min_obstacle_dist, cfg_->optim.weight_optimaltime);
     
     // 发布可视化标记
     visualization_msgs::msg::Marker marker_msg;
     marker_msg.ns = "teb_local_planner";
     marker_msg.id = 0;
     marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
     marker_msg.action = visualization_msgs::msg::Marker::ADD;
     marker_msg.scale.x = 0.1;
     marker_msg.color.r = 1.0;
     marker_msg.color.g = 1.0;
     marker_msg.color.b = 0.0;
     marker_msg.color.a = 1.0;
     
     geometry_msgs::msg::Point start_point, end_point;
     start_point.x = best_wall_start.x;
     start_point.y = best_wall_start.y;
     start_point.z = 0.0;
     
     end_point.x = best_wall_end.x;
     end_point.y = best_wall_end.y;
     end_point.z = 0.0;
     
     marker_msg.points.push_back(start_point);
     marker_msg.points.push_back(end_point);
     
     marker_msg.header.stamp = clock_->now();
     marker_msg.header.frame_id = input_path.header.frame_id;
     
     wall_line_marker_publisher_->publish(marker_msg);
   }
   else 
   {
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "No valid wall line found, clearing configuration");
     // 6. 如果没有找到合适的墙线，检查是否需要清除现有配置
     if(wall_line_points_.size() > 0)
     {
       auto time_diff = clock_->now() - wall_line_update_time_;
       if (time_diff.seconds() > 1.0)
       {
         wall_line_points_.clear();
         
         // Switch back to normal mode parameters
         switchParameterMode(false);
       }
     }
   }
 }
 
 void TebLocalPlannerROS::updateCurbLineVec(
   const nav_msgs::msg::Path curb_line, 
   nav_msgs::msg::Path& input_path,
   const double parallel_tolerance,
   const double distance_tolerance,
   const geometry_msgs::msg::PoseStamped& robot_pose)
 {
   std::lock_guard<std::mutex> l(update_curb_line_mutex_);
   if (curb_line.poses.size() == 0) {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }
   if (input_path.poses.size() >= 2) {
     // 提取机器人当前朝向（从四元数转换为偏航角）
     double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
     
     // 提取输入路径的首尾点
     const auto& start_pose = input_path.poses.front().pose.position;
     const auto& end_pose = input_path.poses.back().pose.position;
     
     // 计算路径方向向量（从起点指向终点）
     const double dx_path = end_pose.x - start_pose.x;
     const double dy_path = end_pose.y - start_pose.y;
     const double path_length = std::hypot(dx_path, dy_path);
     
     if (path_length > 1e-6) {
       // 计算路径方向角度（相对于世界坐标系）
       const double path_yaw = std::atan2(dy_path, dx_path);
       
       // 计算机器人朝向与路径方向的夹角
       double angle_diff = path_yaw - robot_yaw;
       
       // 归一化角度到[-π, π]范围
       while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
       while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
       
       // 检查夹角是否超出±90度范围
       if (std::fabs(angle_diff) > M_PI / 4) {
         if(wall_line_points_.size() > 0) wall_line_points_.clear();
         switchParameterMode(false);
         return;
       }
     }
   }
 
 
   if (curb_line.poses.size() == 0 || input_path.poses.size() < 2)
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     switchParameterMode(false);
     return;
   }  
   // 1. 提取输入路径的首尾点
   const auto& start_pose = input_path.poses.front().pose.position;
   const auto& end_pose = input_path.poses.back().pose.position;
   
   // 2. 计算输入路径方向向量
   const double dx_input = end_pose.x - start_pose.x;
   const double dy_input = end_pose.y - start_pose.y;
   const double input_length = std::hypot(dx_input, dy_input);
   if (input_length < 1e-6)
   {
     if(wall_line_points_.size() > 0)
     {
       auto time_diff  = clock_->now() - wall_line_update_time_;
       if (time_diff.seconds() > 1.0)
       {
         wall_line_points_.clear();
       }
     }
     switchParameterMode(false);
     return;
   }  
   
   const double dir_input_x = dx_input / input_length;
   const double dir_input_y = dy_input / input_length;
 
   // 3. 遍历所有墙线
   bool found_valid_wall = false;
   double min_distance = std::numeric_limits<double>::max();
   nav_msgs::msg::Path best_wall_path;
   geometry_msgs::msg::Point best_wall_start, best_wall_end;
   if(curb_line.poses.size() == 2)
   {
     // 4. 提取墙线端点
     const auto& wall_start = curb_line.poses.front().pose.position;
     const auto& wall_end = curb_line.poses.back().pose.position;
     
     // 5. 计算墙线方向向量
     const double dx_wall = wall_end.x - wall_start.x;
     const double dy_wall = wall_end.y - wall_start.y;
     const double wall_length = std::hypot(dx_wall, dy_wall);
     if(wall_length > 1e-6)
     {
       RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Wall_length: %f !", wall_length);
       const double dir_wall_x = dx_wall / wall_length;
       const double dir_wall_y = dy_wall / wall_length;
       
       // 计算平行度（余弦值）
       const double dot_product = dir_input_x * dir_wall_x + dir_input_y * dir_wall_y;
       const double cos_theta = std::fabs(dot_product);
       const double parallel_threshold = std::abs(std::cos(parallel_tolerance / 180 * M_PI));
       if (cos_theta <= 1.0)
       {
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "马路边沿与路径夹角: %f !", std::acos(cos_theta) / M_PI * 180);
       }
       // 检查平行度是否满足要求
       if(cos_theta > parallel_threshold)
       {
         // 计算垂直距离
         const double A = dy_wall;
         const double B = -dx_wall;
         const double C = wall_end.x * wall_start.y - wall_start.x * wall_end.y;
         
         const double numerator = std::fabs(A * start_pose.x + B * start_pose.y + C);
         const double denominator = std::hypot(A, B);
         const double avg_distance = numerator / denominator;
         
         RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Wall path - Parallelism: %f, Distance: %f", cos_theta, avg_distance);
         
         // 记录满足平行度且距离最小的墙线
         if (avg_distance <= distance_tolerance && avg_distance < min_distance) 
         {
           found_valid_wall = true;
           min_distance = avg_distance;
           best_wall_path = curb_line;
           best_wall_start = wall_start;
           best_wall_end = wall_end;
         }
       }
     }
   }
   
   
 
   // 5. 如果找到最佳匹配的墙线，更新配置
   if (found_valid_wall) 
   {
     if(wall_line_points_.size() > 0) wall_line_points_.clear();
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_start.x, best_wall_start.y));
     wall_line_points_.emplace_back(Eigen::Vector2d(best_wall_end.x, best_wall_end.y));
     
     cfg_->optim.weight_wall_line_dist = weight_wall_line_dist_;
     if (fabs(cfg_->wall_line.distance_tolerance - cfg_->wall_line.min_wall_dist) > 1e-5){
       cfg_->optim.weight_wall_line_dist = std::min(weight_wall_line_dist_, std::max(1.0, std::fabs((1.0 - weight_wall_line_dist_) / (cfg_->wall_line.distance_tolerance - cfg_->wall_line.min_wall_dist) * (min_distance - cfg_->wall_line.min_wall_dist) + weight_wall_line_dist_)));
     }
     wall_line_update_time_ = clock_->now();
     
     // Switch to edge-following mode parameters
     switchParameterMode(true);
     
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Selected best wall line - Distance: %f", min_distance);
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Min obstacle distance: %f, Weight optimal time: %f", cfg_->obstacles.min_obstacle_dist, cfg_->optim.weight_optimaltime);
     
     // 发布可视化标记
     visualization_msgs::msg::Marker marker_msg;
     marker_msg.ns = "teb_local_planner";
     marker_msg.id = 0;
     marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
     marker_msg.action = visualization_msgs::msg::Marker::ADD;
     marker_msg.scale.x = 0.1;
     marker_msg.color.r = 1.0;
     marker_msg.color.g = 1.0;
     marker_msg.color.b = 0.0;
     marker_msg.color.a = 1.0;
     
     geometry_msgs::msg::Point start_point, end_point;
     start_point.x = best_wall_start.x;
     start_point.y = best_wall_start.y;
     start_point.z = 0.0;
     
     end_point.x = best_wall_end.x;
     end_point.y = best_wall_end.y;
     end_point.z = 0.0;
     
     marker_msg.points.push_back(start_point);
     marker_msg.points.push_back(end_point);
     
     marker_msg.header.stamp = clock_->now();
     marker_msg.header.frame_id = input_path.header.frame_id;
     
     wall_line_marker_publisher_->publish(marker_msg);
   }
   else 
   {
     RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "No valid wall line found, clearing configuration");
     // 6. 如果没有找到合适的墙线，检查是否需要清除现有配置
     if(wall_line_points_.size() > 0)
     {
       auto time_diff = clock_->now() - wall_line_update_time_;
       if (time_diff.seconds() > 1.0)
       {
         wall_line_points_.clear();
         
         // Switch back to normal mode parameters
         switchParameterMode(false);
       }
     }
   }
 }
 
 
 void TebLocalPlannerROS::curb_line_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
 {
   std::lock_guard<std::mutex> l(curb_line_mutex_);
 
   curb_line_path_.poses.clear();
   if (msg->poses.size() == 0) {return;}
   // 获取当前时间戳
   rclcpp::Time current_time = clock_->now();
   // 更新curb_line更新时间戳
   curb_line_update_time_ = current_time;
 
   if (msg->poses.size() > 1)
   {
     try {
       // 转换路径的起点和终点到map坐标系
       geometry_msgs::msg::PoseStamped start_pose_transformed;
       geometry_msgs::msg::PoseStamped end_pose_transformed;
       
       // 转换起点
       if (msg->header.frame_id != cfg_->map_frame) {
         try {
           geometry_msgs::msg::TransformStamped transform_start = tf_->lookupTransform(
             cfg_->map_frame, 
             msg->header.frame_id,
             tf2::TimePointZero,
             tf2::durationFromSec(0.5));
           tf2::doTransform(msg->poses.front(), start_pose_transformed, transform_start);
         } catch (const tf2::ExtrapolationException& ex) {
           RCLCPP_WARN(logger_, "curbLinePathCallback: ExtrapolationException when transforming start pose: %s, retrying", ex.what());
           try {
             geometry_msgs::msg::TransformStamped transform_start = tf_->lookupTransform(
               cfg_->map_frame, 
               msg->header.frame_id,
               tf2::TimePointZero);
             tf2::doTransform(msg->poses.front(), start_pose_transformed, transform_start);
           } catch (const tf2::TransformException& ex2) {
             RCLCPP_ERROR(logger_, "curbLinePathCallback: Failed to transform start pose: %s", ex2.what());
             return; // Skip this message if transform fails
           }
         } catch (const tf2::TransformException& ex) {
           RCLCPP_ERROR(logger_, "curbLinePathCallback: TransformException when transforming start pose: %s", ex.what());
           return; // Skip this message if transform fails
         }
       } else {
         start_pose_transformed = msg->poses.front();
       }
       
       // 转换终点
       if (msg->header.frame_id != cfg_->map_frame) {
         try {
           geometry_msgs::msg::TransformStamped transform_end = tf_->lookupTransform(
             cfg_->map_frame, 
             msg->header.frame_id,
             tf2::TimePointZero,
             tf2::durationFromSec(0.5));
           tf2::doTransform(msg->poses.back(), end_pose_transformed, transform_end);
         } catch (const tf2::ExtrapolationException& ex) {
           RCLCPP_WARN(logger_, "curbLinePathCallback: ExtrapolationException when transforming end pose: %s, retrying", ex.what());
           try {
             geometry_msgs::msg::TransformStamped transform_end = tf_->lookupTransform(
               cfg_->map_frame, 
               msg->header.frame_id,
               tf2::TimePointZero);
             tf2::doTransform(msg->poses.back(), end_pose_transformed, transform_end);
           } catch (const tf2::TransformException& ex2) {
             RCLCPP_ERROR(logger_, "curbLinePathCallback: Failed to transform end pose: %s", ex2.what());
             return; // Skip this message if transform fails
           }
         } catch (const tf2::TransformException& ex) {
           RCLCPP_ERROR(logger_, "curbLinePathCallback: TransformException when transforming end pose: %s", ex.what());
           return; // Skip this message if transform fails
         }
       } else {
         end_pose_transformed = msg->poses.back();
       }
       
       // 更新转换后的位姿的时间戳和坐标系
       start_pose_transformed.header.stamp = current_time;
       start_pose_transformed.header.frame_id = cfg_->map_frame;
       end_pose_transformed.header.stamp = current_time;
       end_pose_transformed.header.frame_id = cfg_->map_frame;
       
       // 添加到路径中
       curb_line_path_.poses.emplace_back(start_pose_transformed);
       curb_line_path_.poses.emplace_back(end_pose_transformed);
     }
     catch (tf2::TransformException &ex) {
       RCLCPP_WARN(logger_, 
                   "坐标变换失败: %s, 使用原始坐标但frame_id设为map", ex.what());
       
       // 变换失败时的备选方案
       geometry_msgs::msg::PoseStamped fallback_start = msg->poses.front();
       geometry_msgs::msg::PoseStamped fallback_end = msg->poses.back();
       fallback_start.header.frame_id = cfg_->map_frame;
       fallback_end.header.frame_id = cfg_->map_frame;
       fallback_start.header.stamp = current_time;
       fallback_end.header.stamp = current_time;
       
       curb_line_path_.poses.emplace_back(fallback_start);
       curb_line_path_.poses.emplace_back(fallback_end);
     }
   }
 }
       
 //Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
 //{
 //  Eigen::Vector2d vel;
 //  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
 //  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
 //  return vel;
 //}
       
       
 bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot, double max_prune_dist)
 {
   if (global_plan.empty())
     return true;
   
   try
   {
     // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
     // Use TimePointZero to get the latest available transform to avoid extrapolation errors
     // when global_pose.header.stamp is in the future (especially important for low-frequency transforms like map->odom at 10Hz)
     geometry_msgs::msg::PoseStamped robot;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   global_plan.front().header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot, global_to_plan_transform);
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "pruneGlobalPlan: ExtrapolationException in transform: %s, skipping pruning this cycle", ex.what());
       // If TimePointZero fails with extrapolation, skip pruning this cycle rather than using future time
       // This prevents further extrapolation errors
       return true;
     } catch (const tf2::TransformException& ex) {
       RCLCPP_WARN(logger_, "pruneGlobalPlan: TransformException: %s, skipping pruning this cycle", ex.what());
       return true;
     }
     
     double dist_thresh_sq = dist_behind_robot*dist_behind_robot;    
     // iterate plan until a pose close the robot is found
     std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
     std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
     double accum_dist = 0;
     double min_dist_threshold = std::numeric_limits<double>::max();
     while (it != global_plan.end() && accum_dist < max_prune_dist)
     {
       double dx = robot.pose.position.x - it->pose.position.x;
       double dy = robot.pose.position.y - it->pose.position.y;
       double dist_sq = dx * dx + dy * dy;
       if (it != global_plan.begin())
       {
         double ddx = it->pose.position.x - (it-1)->pose.position.x;
         double ddy = it->pose.position.y - (it-1)->pose.position.y;
         accum_dist += std::sqrt(ddx*ddx + ddy*ddy);
       }
       if (dist_sq < min_dist_threshold)
       {
         min_dist_threshold = dist_sq;
         erase_end = it;
       }
       if (dist_sq < dist_thresh_sq)
       {
          erase_end = it;
          break;
       }
       ++it;
       // ++count_it;
     }
 
     if (erase_end == global_plan.end())
       return false;
     
     if (erase_end != global_plan.begin())
       global_plan.erase(global_plan.begin(), erase_end);
   }
   catch (const tf2::TransformException& ex)
   {
     RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
     return false;
   }
   return true;
 }
       
 
 bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                   const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                   std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
 {
   // this method is a slightly modified version of base_local_planner/goal_functions.h
 
   const geometry_msgs::msg::PoseStamped& plan_pose = global_plan[0];
 
   transformed_plan.clear();
 
   try 
   {
     if (global_plan.empty())
     {
       RCLCPP_ERROR(logger_, "Received plan with zero length");
       *current_goal_idx = 0;
       return false;
     }
     // get plan_to_global_transform from plan frame to global_frame
     // Use tf2::TimePointZero to get the latest available transform
     // This avoids extrapolation errors when plan_pose.header.stamp is in the future
     // (especially important for low-frequency transforms like map->odom at 10Hz)
     // Direct lookup without fixed_frame to avoid extrapolation issues
     // RCLCPP_INFO(logger_, "transformGlobalPlan: Looking up transform from %s to %s", 
     //              plan_pose.header.frame_id.c_str(), global_frame.c_str());
     // RCLCPP_INFO(logger_, "transformGlobalPlan: plan_pose.header.stamp = %.6f, global_pose.header.stamp = %.6f", 
     //              rclcpp::Time(plan_pose.header.stamp).seconds(),
     //              rclcpp::Time(global_pose.header.stamp).seconds());
     
     geometry_msgs::msg::TransformStamped plan_to_global_transform;
     try {
       plan_to_global_transform = tf_->lookupTransform(
                   global_frame,
                   plan_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       // RCLCPP_INFO(logger_, "transformGlobalPlan: Successfully got transform, transform.header.stamp = %.6f", 
       //              rclcpp::Time(plan_to_global_transform.header.stamp).seconds());
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "transformGlobalPlan: ExtrapolationException in lookupTransform: %s", ex.what());
       RCLCPP_WARN(logger_, "transformGlobalPlan: Requested from %s to %s with TimePointZero, retrying with latest available time", 
                    plan_pose.header.frame_id.c_str(), global_frame.c_str());
       // Retry with TimePointZero but without specifying source time (uses latest available)
       try {
         plan_to_global_transform = tf_->lookupTransform(
                     global_frame,
                     plan_pose.header.frame_id,
                     tf2::TimePointZero);
       } catch (const tf2::TransformException& ex2) {
         RCLCPP_ERROR(logger_, "transformGlobalPlan: Failed to get transform after retry: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
       RCLCPP_ERROR(logger_, "transformGlobalPlan: TransformException: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
 
 //    tf_->waitForTransform(global_frame, ros::Time::now(),
 //    plan_pose.header.frame_id, plan_pose.header.stamp,
 //    plan_pose.header.frame_id, ros::Duration(0.5));
 //    tf_->lookupTransform(global_frame, ros::Time(),
 //    plan_pose.header.frame_id, plan_pose.header.stamp,
 //    plan_pose.header.frame_id, plan_to_global_transform);
 
     //let's get the pose of the robot in the frame of the plan
     // Use TimePointZero to avoid extrapolation errors when global_pose.header.stamp is in the future
     // RCLCPP_INFO(logger_, "transformGlobalPlan: Transforming robot pose from %s to %s", 
     //              global_pose.header.frame_id.c_str(), plan_pose.header.frame_id.c_str());
     // RCLCPP_INFO(logger_, "transformGlobalPlan: global_pose.header.stamp = %.6f", 
     //              rclcpp::Time(global_pose.header.stamp).seconds());
     
     geometry_msgs::msg::PoseStamped robot_pose;
     try {
       // Get transform using TimePointZero to avoid extrapolation
       geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                   plan_pose.header.frame_id,
                   global_pose.header.frame_id,
                   tf2::TimePointZero,
                   tf2::durationFromSec(0.5));
       // RCLCPP_INFO(logger_, "transformGlobalPlan: Got transform from %s to %s, transform.header.stamp = %.6f", 
       //              global_pose.header.frame_id.c_str(), plan_pose.header.frame_id.c_str(),
       //              rclcpp::Time(global_to_plan_transform.header.stamp).seconds());
       
       // Apply transform manually to avoid using global_pose.header.stamp
       tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
       // RCLCPP_INFO(logger_, "transformGlobalPlan: Successfully transformed robot pose");
     } catch (const tf2::ExtrapolationException& ex) {
       RCLCPP_WARN(logger_, "transformGlobalPlan: ExtrapolationException when transforming robot pose: %s, retrying", ex.what());
       // Retry with TimePointZero but without specifying source time
       try {
         geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
                     plan_pose.header.frame_id,
                     global_pose.header.frame_id,
                     tf2::TimePointZero);
         tf2::doTransform(global_pose, robot_pose, global_to_plan_transform);
       } catch (const tf2::TransformException& ex2) {
         RCLCPP_ERROR(logger_, "transformGlobalPlan: Failed to transform robot pose after retry: %s", ex2.what());
         throw nav2_core::PlannerException(
           std::string("Could not transform the global plan to the frame of the controller: ") + ex2.what()
         );
       }
     } catch (const tf2::TransformException& ex) {
       RCLCPP_ERROR(logger_, "transformGlobalPlan: TransformException in robot pose transform: %s", ex.what());
       throw nav2_core::PlannerException(
         std::string("Could not transform the global plan to the frame of the controller: ") + ex.what()
       );
     }
 
     //we'll discard points on the plan that are outside the local costmap
     double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                      costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
     dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                            // located on the border of the local costmap
     
 
     int i = 0;
     double sq_dist_threshold = dist_threshold * dist_threshold;
     double sq_dist = 1e10;
     
     //we need to loop to a point on the plan that is within a certain distance of the robot
     bool robot_reached = false;
     for(int j=0; j < (int)global_plan.size() && j < 40; ++j)
     {
       double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
       double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
       double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
       if (new_sq_dist > sq_dist_threshold)
         break;  // force stop if we have reached the costmap border
 
       if (robot_reached && new_sq_dist > sq_dist)
         break;
 
       if (new_sq_dist < sq_dist) // find closest distance
       {
         sq_dist = new_sq_dist;
         i = j;
         if (sq_dist < 0.25)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
           robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
       }
     }
 
     geometry_msgs::msg::PoseStamped newer_pose;
     
     double plan_length = 0; // check cumulative Euclidean distance along the plan
     
     //now we'll transform until points are outside of our distance threshold
     cfg_->optim.weight_viapoint = weight_via_point_;
     while(i < (int)global_plan.size() && (max_plan_length<=0 || plan_length <= max_plan_length))
     {
       //const geometry_msgs::msg::PoseStamped& pose = global_plan[i];
       //tf::poseStampedMsgToTF(pose, tf_pose);
       //tf_pose.setData(plan_to_global_transform * tf_pose);
       tf2::doTransform(global_plan[i], newer_pose, plan_to_global_transform);
 
 //      tf_pose.stamp_ = plan_to_global_transform.stamp_;
 //      tf_pose.frame_id_ = global_frame;
 //      tf::poseStampedTFToMsg(tf_pose, newer_pose);
 
       transformed_plan.push_back(newer_pose);
 
       geometry_msgs::msg::Pose2D check_pose2d;
       check_pose2d.x = newer_pose.pose.position.x;
       check_pose2d.y = newer_pose.pose.position.y;
       check_pose2d.theta = tf2::getYaw(newer_pose.pose.orientation);
       double check_cost = 0;
       try
       {
         check_cost =  costmap_model_->scorePose(check_pose2d, dwb_critics::getOrientedFootprint(check_pose2d, footprint_spec_));
         if (check_cost > 0)
         {
           cfg_->optim.weight_viapoint = 1.0;
         }
       }
       catch(const dwb_core::IllegalTrajectoryException& e)
       {
         if ((int)global_plan.size() > 0)
         {
           if (i == ((int)global_plan.size() - 1))
           {
             if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
             {
               throw nav2_core::PlannerException(
                 std::string("Teb cannot find a free goal, goals are occupied ! ") + e.what()
               );
             }
           }
           if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
           {
             max_plan_length = costmap_->getSizeInMetersX() / 2.0 + 1.0;
           }
           cfg_->optim.weight_viapoint = 1.0;
         }
       }
       
       // caclulate distance to previous pose
       if (i>0 && max_plan_length>0)
         plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);
       ++i;
     }
         
     // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
     // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
     if (transformed_plan.empty())
     {
       //如果最后一个目标点不可达，抛出异常
       geometry_msgs::msg::PoseStamped plan_local_pose;
       tf2::doTransform(global_plan[i], plan_local_pose, plan_to_global_transform);
       geometry_msgs::msg::Pose2D pose2d;
       pose2d.x = plan_local_pose.pose.position.x;
       pose2d.y = plan_local_pose.pose.position.y;
       pose2d.theta = tf2::getYaw(plan_local_pose.pose.orientation);
       double cost = 0;
       try
       {
         cost =  costmap_model_->scorePose(pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec_));
       }
       catch(const dwb_core::IllegalTrajectoryException& e)
       {
         if (!std::strcmp(e.what(), "Trajectory Hits Obstacle."))
         {
           throw nav2_core::PlannerException(
             std::string("Teb cannot find a free goal, goals are occupied ! ") + e.what()
           );
         }
       }
       //如果最后一个目标点不可达，抛出异常
 
       tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);
 
       transformed_plan.push_back(newer_pose);
       
       // Return the index of the current goal point (inside the distance threshold)
       if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
     }
     else
     {
       // Return the index of the current goal point (inside the distance threshold)
       if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
     }
     
     // Return the transformation from the global plan to the global planning frame if desired
     if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
   }
   catch(tf2::LookupException& ex)
   {
     RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
     return false;
   }
   catch(tf2::ConnectivityException& ex)
   {
     RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
     return false;
   }
   catch(tf2::ExtrapolationException& ex)
   {
     RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
     if (global_plan.size() > 0)
       RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
 
     return false;
   }
 
   return true;
 }
 
 
     
       
       
 double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                     int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
 {
   int n = (int)global_plan.size();
   
   // check if we are near the global goal already
   if (current_goal_idx > n-moving_average_length-2)
   {
     if (current_goal_idx >= n-1) // we've exactly reached the goal
     {
       return tf2::getYaw(local_goal.pose.orientation);
     }
     else
     {
 //      tf::Quaternion global_orientation;
 //      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
 //      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );
 
         tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
         tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
         tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);
 
         return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
     }     
   }
   
   // reduce number of poses taken into account if the desired number of poses is not available
   moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
   
   std::vector<double> candidates;
   geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
   geometry_msgs::msg::PoseStamped tf_pose_kp1;
   
   int range_end = current_goal_idx + moving_average_length;
   for (int i = current_goal_idx; i < range_end; ++i)
   {
     // Transform pose of the global plan to the planning frame
     const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
     tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);
       
     // calculate yaw angle  
     candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
               tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
     
     if (i<range_end-1) 
       tf_pose_k = tf_pose_kp1;
   }
   return average_angles(candidates);
 }
       
       
 void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
 {
   double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
   // Limit translational velocity for forward driving
   if (vx > max_vel_x)
     ratio_x = max_vel_x / vx;
   
   // limit strafing velocity
   if (vy > max_vel_y || vy < -max_vel_y)
     ratio_y = std::abs(max_vel_y / vy);
   
   // Limit angular velocity
   if (omega > max_vel_theta || omega < -max_vel_theta)
     ratio_omega = std::abs(max_vel_theta / omega);
   
   // Limit backwards velocity
   if (max_vel_x_backwards<=0)
   {
     RCLCPP_WARN_ONCE(
                 logger_,
                 "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
   }
   else if (vx < -max_vel_x_backwards)
     ratio_x = - max_vel_x_backwards / vx;
 
   if (cfg_->robot.use_proportional_saturation)
   {
     double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
     vx *= ratio;
     vy *= ratio;
     omega *= ratio;
   }
   else
   {
     vx *= ratio_x;
     vy *= ratio_y;
     omega *= ratio_omega;
   }
 }
      
      
 double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
 {
   if (omega==0 || v==0)
     return 0;
     
   double radius = v/omega;
   
   if (fabs(radius) < min_turning_radius)
     radius = double(g2o::sign(radius)) * min_turning_radius; 
 
   return std::atan(wheelbase / radius);
 }
      
 
 void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
 {
     RCLCPP_WARN_EXPRESSION(
                 logger_, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                   "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                   "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                   "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
 }
    
    
    
 void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,  int& goal_idx)
 {
     rclcpp::Time current_time = clock_->now();
     
     // reduced horizon backup mode
     if (cfg_->recovery.shrink_horizon_backup && 
         goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
        (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
     {
         RCLCPP_INFO_EXPRESSION(
                     logger_,
                     no_infeasible_plans_==1,
                     "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);
 
 
         // Shorten horizon if requested
         // reduce to 50 percent:
         int horizon_reduction = goal_idx/2;
         
         if (no_infeasible_plans_ > 9)
         {
             RCLCPP_INFO_EXPRESSION(
                         logger_,
                         no_infeasible_plans_==10,
                         "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
             horizon_reduction /= 2;
         }
         
         // we have a small overhead here, since we already transformed 50% more of the trajectory.
         // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
         // and a reduced horizon should occur just rarely.
         int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
         goal_idx -= horizon_reduction;
         if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
             transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
         else
             goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
     }
     
     
     // detect and resolve oscillations
     if (cfg_->recovery.oscillation_recovery)
     {
         double max_vel_theta;
         double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
         if (cfg_->robot.min_turning_radius!=0 && max_vel_current>0)
             max_vel_theta = std::max( max_vel_current/std::abs(cfg_->robot.min_turning_radius),  cfg_->robot.max_vel_theta );
         else
             max_vel_theta = cfg_->robot.max_vel_theta;
         
         failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta,
                                cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps);
         
         bool oscillating = failure_detector_.isOscillating();
         bool recently_oscillated = (clock_->now()-time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
         
         if (oscillating)
         {
             if (!recently_oscillated)
             {
                 // save current turning direction
                 if (robot_vel_.angular.z > 0)
                     last_preferred_rotdir_ = RotType::left;
                 else
                     last_preferred_rotdir_ = RotType::right;
                 RCLCPP_INFO(logger_, "TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
             }
             time_last_oscillation_ = clock_->now();
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
         }
         else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
         {
             last_preferred_rotdir_ = RotType::none;
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
             RCLCPP_INFO(logger_, "TebLocalPlannerROS: oscillation recovery disabled/expired.");
         }
     }
 
 }
      
 
 void TebLocalPlannerROS::setSpeedLimit(
     const double & speed_limit, const bool & percentage)
 {
   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
     // Restore default value
     cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x;
     cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards;
     cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y;
     cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta;
   } else {
     if (percentage) {
       // Speed limit is expressed in % from maximum speed of robot
       cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * speed_limit / 100.0;
       cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * speed_limit / 100.0;
       cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * speed_limit / 100.0;
       cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * speed_limit / 100.0;
     } else {
       // Speed limit is expressed in absolute value
       double max_speed_xy = std::max(
             std::max(cfg_->robot.base_max_vel_x,cfg_->robot.base_max_vel_x_backwards),cfg_->robot.base_max_vel_y);
       if (speed_limit < max_speed_xy) {
         // Handling components and angular velocity changes:
         // Max velocities are being changed in the same proportion
         // as absolute linear speed changed in order to preserve
         // robot moving trajectories to be the same after speed change.
         // G. Doisy: not sure if that's applicable to base_max_vel_x_backwards.
         const double ratio = speed_limit / max_speed_xy;
         cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * ratio;
         cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * ratio;
         cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * ratio;
         cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * ratio;
       }
     }
   }
 }
 
 void TebLocalPlannerROS::switchParameterMode(bool enable_edge_mode)
 {
   if (!initialized_ || !nh_.lock()) {
     return;
   }
   
   auto node = nh_.lock();
   
   // Avoid redundant parameter updates
   if (enable_edge_mode == is_edge_following_mode_) {
     return;
   }
   
   try {
     if (enable_edge_mode) {
       // Switch to edge-following mode
       // Read latest edge mode parameters from config (they may have been updated dynamically)
       double current_edge_weight_optimaltime = cfg_->wall_line.edge_weight_optimaltime;
       double current_edge_min_obstacle_dist = cfg_->wall_line.edge_min_obstacle_dist;
       double current_edge_acc_lim_theta = cfg_->robot.acc_lim_theta;
       double current_edge_max_vel_theta = cfg_->robot.max_vel_theta;
       std::string current_edge_footprint_vertices = cfg_->wall_line.edge_footprint_vertices;
       
       RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Switching to edge-following mode parameters");
       
       // Set weight_optimaltime
       node->set_parameter(rclcpp::Parameter(name_ + "." + "weight_optimaltime", current_edge_weight_optimaltime));
       
       // Set min_obstacle_dist
       node->set_parameter(rclcpp::Parameter(name_ + "." + "min_obstacle_dist", current_edge_min_obstacle_dist));
       
       // Set footprint vertices
       node->set_parameter(rclcpp::Parameter(name_ + "." + "footprint_model.vertices", current_edge_footprint_vertices));
 
       // Set acc_lim_theta
       node->set_parameter(rclcpp::Parameter(name_ + "." + "acc_lim_theta", current_edge_acc_lim_theta));
       
       // Set max_vel_theta
       node->set_parameter(rclcpp::Parameter(name_ + "." + "max_vel_theta", current_edge_max_vel_theta));
       
       is_edge_following_mode_ = true;
     } else {
       // Switch to normal mode
       RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "Switching to normal mode parameters");
       
       // Set weight_optimaltime
       node->set_parameter(rclcpp::Parameter(name_ + "." + "weight_optimaltime", normal_weight_optimaltime_));
       
       // Set min_obstacle_dist
       node->set_parameter(rclcpp::Parameter(name_ + "." + "min_obstacle_dist", normal_min_obstacle_dist_));
       
       // Set footprint vertices
       node->set_parameter(rclcpp::Parameter(name_ + "." + "footprint_model.vertices", normal_footprint_vertices_));
 
       // Set acc_lim_theta
       node->set_parameter(rclcpp::Parameter(name_ + "." + "acc_lim_theta", cfg_max_angular_acc_));
       
       // Set max_vel_theta
       node->set_parameter(rclcpp::Parameter(name_ + "." + "max_vel_theta", cfg_max_angular_vel_));
       
       is_edge_following_mode_ = false;
 
     }
   } catch (const std::exception& ex) {
     RCLCPP_WARN(logger_, "Failed to switch parameter mode: %s", ex.what());
   }
 }
 
 void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
 {
   std::lock_guard<std::mutex> l(custom_obst_mutex_);
   custom_obstacle_msg_ = *obst_msg;  
 }
 
 void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
 {
   RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
   if (cfg_->trajectory.global_plan_viapoint_sep > 0)
   {
     RCLCPP_INFO(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
              "Ignoring custom via-points.");
     custom_via_points_active_ = false;
     return;
   }
 
   std::lock_guard<std::mutex> l(via_point_mutex_);
   via_points_.clear();
   for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
   {
     via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
   }
   custom_via_points_active_ = !via_points_.empty();
 }
 
 // void TebLocalPlannerROS::rotation_sigh_callback(const std_msgs::msg::Bool &msg)
 // {
 //   std::lock_guard<std::mutex> cfg_via_sep_lock(cfg_->configMutex());
 //   if (msg.data)
 //   {
 //     cfg_->trajectory.global_plan_viapoint_sep = via_sep_;
 //   }
 //   else
 //   {
 //     cfg_->trajectory.global_plan_viapoint_sep = -0.1;
 //   }
   
 // }
 
 void TebLocalPlannerROS::activate() {
   visualization_->on_activate();
 
   return;
 }
 void TebLocalPlannerROS::deactivate() {
   visualization_->on_deactivate();
 
   return;
 }
 void TebLocalPlannerROS::cleanup() {
   visualization_->on_cleanup();
   costmap_converter_->stopWorker();
   
   return;
 }
 
 } // end namespace teb_local_planner
 
 // register this planner as a nav2_core::Controller plugin
 PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
 