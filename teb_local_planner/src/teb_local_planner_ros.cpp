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
#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/front_steer_drive_interface.hpp"

 //#include <tf_conversions/tf_eigen.h>
 #include <boost/algorithm/string.hpp>

 #include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <future>
#include <thread>
#include <system_error>
#include <chrono>
#include <functional>
#include <mutex>
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
 #include "dwb_critics/obstacle_footprint.hpp"
 #include "dwb_critics/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "teb_local_planner/misc.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <exception>
#include <limits>
#include "teb_local_planner/detail/plan_corner_geometry.hpp"
#include "teb_local_planner/detail/async_wait.hpp"
#include "teb_local_planner/detail/edge_mode_env.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{

using detail::yawAbsDiff;
using detail::polylineVertexAngleDeg;
using detail::lastIdxWithinArc;
using detail::findFirstSharpCornerIdx;
using detail::findCurrentRowStartIdx;
using detail::planArcLength;
using detail::findEuclideanClosestPlanIdx;
using detail::clampClosestIdxBeforeUnprunedCorner;
using detail::lockedCornerIdx;

using detail::kBackgroundPollSlice;
using detail::joinThreadQuietly;
using detail::detachThreadQuietly;
using detail::waitForFutureInterruptible;

using detail::normalizeEdgeModeEnv;

 TebLocalPlannerROS::TebLocalPlannerROS() 
     : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), intra_proc_node_(nullptr),
                                            costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                            custom_via_points_active_(false), no_infeasible_plans_(0),
                                            last_preferred_rotdir_(RotType::none), initialized_(false),
                                            weight_wall_side_pull_(80.0),
                                            cfg_max_angular_vel_(0.6), cfg_max_vel_x_(0.5), cfg_max_angular_acc_(0.6), wall_line_update_time_(0),
                                            curb_line_update_time_(0), min_obstacle_dist_(0.5), wall_line_ptr_(nullptr), curb_line_subscriber_(nullptr),
                                            normal_weight_optimaltime_(2.0), normal_min_obstacle_dist_(0.2),
                                            normal_weight_inflation_(0.1),
                                            normal_footprint_vertices_("[[1.25, 0.55], [1.25, -0.55], [-0.65, -0.55], [-0.65, 0.55]]"),
                                            edge_weight_optimaltime_(10.0), edge_min_obstacle_dist_(0.05),
                                            edge_weight_inflation_(0.0), min_wall_line_length_(1.0),
                                            edge_footprint_vertices_("[[1.25, 0.5], [1.25, -0.5], [-0.65, -0.5], [-0.65, 0.5]]"),
                                            is_edge_following_mode_(false),
                                            control_duration_(0.2), 
                                            safe_linear_speed_limit_(2.0),
                                            keep_wall_line_time_(5.0),
                                            new_vehicle_distance_threshold_(5.0),
                                            erase_vehicle_distance_threshold_(10.0),
                                            has_speed_limit_(false)
 {
   // Initialize edge mode parameters from config defaults (will be overridden by parameters if available)
   edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
   edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
   edge_weight_inflation_ = cfg_->wall_line.edge_weight_inflation;
   edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
 }
 
 
 TebLocalPlannerROS::~TebLocalPlannerROS()
 {
   shutting_down_.store(true, std::memory_order_release);
   joinBackgroundTasks(std::chrono::milliseconds(1500));
   stopCostmapConverterWorker(std::chrono::milliseconds(800));
   wall_line_ptr_.reset();
   curb_line_subscriber_.reset();
 }
 
 void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
 {
   
   // check if the plugin is already initialized
   if(!initialized_)
   {	
    shutting_down_.store(false, std::memory_order_release);
    // declare parameters (ros2-dashing)
    intra_proc_node_.reset( 
            new rclcpp::Node("costmap_converter", node->get_namespace(), 
              rclcpp::NodeOptions()));
    cfg_->declareParameters(node, name_);

    // The interface parameters belong to the controller plugin namespace and
    // are deliberately fixed during configure.  Runtime switching would
    // allow two control chains to be active in the same cycle.
    rcl_interfaces::msg::ParameterDescriptor interface_descriptor;
    interface_descriptor.read_only = true;
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.mode",
      rclcpp::ParameterValue(front_steer_interface_mode_), interface_descriptor);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.legacy_command_topic",
      rclcpp::ParameterValue(legacy_command_topic_), interface_descriptor);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.command_topic",
      rclcpp::ParameterValue(front_steer_command_topic_), interface_descriptor);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.state_topic",
      rclcpp::ParameterValue(front_steer_state_topic_), interface_descriptor);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.state_timeout",
      rclcpp::ParameterValue(front_steer_state_timeout_), interface_descriptor);
    node->get_parameter(name_ + ".front_steer_drive_interface.mode", front_steer_interface_mode_);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.legacy_command_topic", legacy_command_topic_);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.command_topic", front_steer_command_topic_);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.state_topic", front_steer_state_topic_);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.state_timeout", front_steer_state_timeout_);

    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.max_projection_omega_error",
      rclcpp::ParameterValue(front_steer_projection_tolerance_), interface_descriptor);
    node->get_parameter(name_ + ".front_steer_drive_interface.max_projection_omega_error",
      front_steer_projection_tolerance_);

    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.odom_to_state.enabled",
      rclcpp::ParameterValue(front_steer_odom_enabled_), interface_descriptor);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.odom_to_state.enabled", front_steer_odom_enabled_);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.odom_to_state.topic",
      rclcpp::ParameterValue(front_steer_odom_topic_), interface_descriptor);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.odom_to_state.topic", front_steer_odom_topic_);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.odom_to_state.wheel_radius",
      rclcpp::ParameterValue(front_steer_odom_wheel_radius_), interface_descriptor);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.odom_to_state.wheel_radius", front_steer_odom_wheel_radius_);
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.odom_to_state.angular_deadband",
      rclcpp::ParameterValue(front_steer_odom_angular_deadband_), interface_descriptor);
    node->get_parameter(
      name_ + ".front_steer_drive_interface.odom_to_state.angular_deadband",
      front_steer_odom_angular_deadband_);

    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.steering_lag_speed_scaling",
      rclcpp::ParameterValue(front_steer_lag_speed_scaling_), interface_descriptor);
    node->get_parameter(name_ + ".front_steer_drive_interface.steering_lag_speed_scaling",
      front_steer_lag_speed_scaling_);

    // The Controller Server owns the Twist publisher. It reads this fixed
    // per-controller setting; keep returning the real Twist for Nav2 internals.
    declare_parameter_if_not_declared(
      node, name_ + ".front_steer_drive_interface.publish_twist",
      rclcpp::ParameterValue(true), interface_descriptor);
    bool publish_twist = true;
    node->get_parameter(name_ + ".front_steer_drive_interface.publish_twist", publish_twist);
    if (front_steer_interface_mode_ == "front_steer_cmd" && !publish_twist) {
      RCLCPP_INFO(logger_, "front_steer_cmd 模式关闭 Controller Server 的 Twist 下发");
    }

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_->loadRosParamFromNodeHandle(node, name_);
    initializeFrontSteerDriveInterface(node);
    weight_wall_line_direction_ = cfg_->optim.weight_wall_line_direction;
    weight_wall_line_dist_ = cfg_->optim.weight_wall_line_dist;
    weight_wall_side_pull_ = cfg_->optim.weight_wall_side_pull;
    cfg_max_angular_vel_ = cfg_->robot.max_vel_theta;
    cfg_max_vel_x_ = cfg_->robot.max_vel_x;
    cfg_max_angular_acc_= cfg_->robot.acc_lim_theta;
    min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
    
    // Save normal mode parameters
    normal_weight_optimaltime_ = cfg_->optim.weight_optimaltime;
    normal_min_obstacle_dist_ = cfg_->obstacles.min_obstacle_dist;
    normal_weight_inflation_ = cfg_->optim.weight_inflation;
    // Try to get footprint vertices from parameter server
    std::string footprint_string;
    if (node->get_parameter(name_ + "." + "footprint_model.vertices", footprint_string)) {
      normal_footprint_vertices_ = footprint_string;
    }
    
    // Load edge-following mode parameters from config
    edge_weight_optimaltime_ = cfg_->wall_line.edge_weight_optimaltime;
    edge_min_obstacle_dist_ = cfg_->wall_line.edge_min_obstacle_dist;
    edge_weight_inflation_ = cfg_->wall_line.edge_weight_inflation;
    edge_footprint_vertices_ = cfg_->wall_line.edge_footprint_vertices;
    // via_sep_ = cfg_->trajectory.global_plan_viapoint_sep;
    RCLCPP_INFO(logger_, "初始化: 全局路径前视距离 (max_global_plan_lookahead_dist) %.2f m, max_vel_x: %.2f", cfg_->trajectory.max_global_plan_lookahead_dist, cfg_->robot.max_vel_x);

    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create the planner instance
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
      RCLCPP_INFO(logger_, "已启用多拓扑并行规划 (distinctive topologies)");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_, &wall_line_points_));
      RCLCPP_INFO(logger_, "已关闭多拓扑并行规划 (distinctive topologies)");
    }
    
    // init other variables
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
    std::string costmap_model_name("costmap_model");
    costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);

    // Initialize rotation collision checker
    rotation_collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  
    // Get controller frequency for control_duration
    double controller_frequency = 5.0;
    node->get_parameter("controller_frequency", controller_frequency);
    control_duration_ = 1.0 / controller_frequency;

    cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO

    //Initialize a costmap to polygon converter
    if (!cfg_->obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
        RCLCPP_INFO(logger_, "代价地图转换插件库路径: %s", costmap_converter_loader_.getClassLibraryPath(cfg_->obstacles.costmap_converter_plugin).c_str());
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");

        costmap_converter_->setOdomTopic(cfg_->odom_topic);
        
        // Pass the main (lifecycle) node directly to costmap_converter so it can read parameters from controller node
        // Parameters will be read directly from the node's namespace (e.g., controller_server.FollowPath.cluster_max_distance)
        costmap_converter_->initialize(node);
        costmap_converter_->setCostmap2D(costmap_);
        const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
        costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
        RCLCPP_INFO(logger_, "已加载代价地图转换插件: %s", cfg_->obstacles.costmap_converter_plugin.c_str());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        RCLCPP_INFO(logger_,
                    "无法加载指定的代价地图转换插件，占用栅格将按点障碍处理。错误: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else {
      RCLCPP_INFO(logger_, "未指定代价地图转换插件，占用栅格将按点障碍处理");
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

    {
      rclcpp::QoS path_wait_qos(rclcpp::KeepLast(1));
      path_wait_qos.reliable();
      path_wait_obst_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
          cfg_->obstacles.path_wait_topic,
          path_wait_qos,
          std::bind(&TebLocalPlannerROS::pathWaitObstacleCB, this, std::placeholders::_1));
      RCLCPP_INFO(
          logger_,
          "TEB 独立订阅动态障碍话题 %s（PATH 停车门控，不并入 obstacles）",
          cfg_->obstacles.path_wait_topic.c_str());
    }

    // setup callback for custom via-points
    via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                "via_points", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

    // setup callback for surrounding vehicle poses
    {
      rclcpp::QoS vehicle_qos(rclcpp::KeepLast(1));
      vehicle_qos.transient_local();
      vehicle_qos.reliable();
      vehicle_poses_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/vehicle_poses_around",
        vehicle_qos,
        std::bind(&TebLocalPlannerROS::vehiclePosesCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /vehicle_poses_around 话题");
    }

    // setup callback for external speed limit (linear x)
    {
      rclcpp::QoS speed_limit_qos(rclcpp::KeepLast(1));
      speed_limit_qos.transient_local();
      speed_limit_qos.reliable();
      speed_limit_sub_ = node->create_subscription<std_msgs::msg::Float64>(
                "/nav/speed_limit",
                speed_limit_qos,
                std::bind(&TebLocalPlannerROS::speedLimitCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /nav/speed_limit 话题");
    }
    {
      // 狭窄通道多边形（与 Smac 共用 /narrow_passages）
      rclcpp::QoS narrow_qos(rclcpp::KeepLast(1));
      narrow_qos.transient_local();
      narrow_qos.reliable();
      narrow_passages_sub_ = node->create_subscription<garage_utils_msgs::msg::Polygons>(
        "/narrow_passages",
        narrow_qos,
        std::bind(&TebLocalPlannerROS::narrowPassagesCallback, this, std::placeholders::_1));
      narrow_passages_marker_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_narrow_passages_markers", 1);
      RCLCPP_INFO(
        logger_,
        "狭窄通道: TEB 已订阅 /narrow_passages，"
        "forward_drive 正常=%.2f，窄通道=%.2f，Marker 话题 teb_narrow_passages_markers",
        cfg_->optim.weight_kinematics_forward_drive,
        cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages);
    }
    {
      rclcpp::QoS enable_backward_qos(rclcpp::KeepLast(1));
      enable_backward_qos.transient_local();
      enable_backward_qos.reliable();
      enable_backward_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "/enable_backward",
        enable_backward_qos,
        std::bind(&TebLocalPlannerROS::enableBackwardCallback, this, std::placeholders::_1));
      RCLCPP_INFO(logger_, "TEB 已订阅 /enable_backward，true 时强制倒车友好参数");
    }
    {
      rclcpp::QoS backward_mode_qos(rclcpp::KeepLast(1));
      backward_mode_qos.transient_local();
      backward_mode_qos.reliable();
      backward_mode_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        "/backward_mode", backward_mode_qos);
      resetBackwardModePublicationState(true);
      RCLCPP_INFO(
        logger_,
        "TEB 已发布 /backward_mode（仅几何倒车检测），"
        "lookahead=%.2f arc_length_ratio=%.2f backward_check_duration=%.2f backward_check_num=%d",
        cfg_->trajectory.reverse_segment_lookahead_dist,
        cfg_->trajectory.reverse_segment_arc_length_ratio,
        cfg_->trajectory.backward_check_duration,
        cfg_->trajectory.backward_check_num);
    }
    // 备份正常模式参数，供狭窄通道 RAII 恢复
    normal_weight_kinematics_forward_drive_ = cfg_->optim.weight_kinematics_forward_drive;
    normal_delete_detours_backwards_ = cfg_->hcp.delete_detours_backwards;
    normal_allow_init_with_backwards_motion_ = cfg_->trajectory.allow_init_with_backwards_motion;
    {
      rclcpp::QoS vehicle_scan_qos(rclcpp::KeepLast(5));
      vehicle_scan_qos.best_effort();
      vehicle_scan_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_->obstacles.vehicle_scan_topic,
        vehicle_scan_qos,
        std::bind(&TebLocalPlannerROS::vehicleScanCloudCallback, this, std::placeholders::_1));
    }
    if (cfg_->wall_line.enable_front_scan_protrusion_check) {
      rclcpp::QoS front_scan_qos(rclcpp::KeepLast(5));
      front_scan_qos.best_effort();
      front_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        cfg_->wall_line.front_scan_topic,
        front_scan_qos,
        std::bind(&TebLocalPlannerROS::frontScanCallback, this, std::placeholders::_1));
      RCLCPP_INFO(
        logger_,
        "TEB 订阅前向 LaserScan 贴边 protrusion 检测，话题：%s",
        cfg_->wall_line.front_scan_topic.c_str());
    }
    {
      // volatile + reliable：与 RViz / 默认 echo 的 QoS 易匹配
      rclcpp::QoS grid_qos(rclcpp::KeepLast(1));
      grid_qos.reliable();
      vehicle_scan_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        cfg_->obstacles.vehicle_scan_grid_topic, grid_qos);
      RCLCPP_INFO(logger_, "TEB 创建 /teb_vehicle_scan_grid 话题");
    }
    // setup callback for line center points (WALL / CURB / Reference / fusion modes via USE_CURB_OR_WALL)
    if (cfg_->optim.weight_wall_line_dist > 0.0 && detail::useCurbOrWallEnv() != nullptr) {
      const std::string normalized_wall_or_curb_edge_mode_string =
        normalizeEdgeModeEnv(detail::useCurbOrWallEnv());
      if (normalized_wall_or_curb_edge_mode_string.find("wall") != std::string::npos) {
        wall_line_ptr_ = line_path_compare::LinePathCompare::create(node, tf_);
        RCLCPP_INFO(logger_, "TEB 使用2D激光雷达检测墙线贴边");
      }
      if (normalized_wall_or_curb_edge_mode_string == "curb" ||
          (normalized_wall_or_curb_edge_mode_string.find("curb") != std::string::npos &&
            normalized_wall_or_curb_edge_mode_string.find("reference") != std::string::npos)) {
        curb_line_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
          "camera1/extracted_line_path",
          rclcpp::QoS{5}.best_effort(),
          std::bind(&TebLocalPlannerROS::curb_line_callback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "TEB 使用深度相机提取墙线贴边，话题：/camera1/extracted_line_path");
      }
      if (normalized_wall_or_curb_edge_mode_string.find("reference") != std::string::npos) {
        rclcpp::QoS reference_paths_subscription_qos(rclcpp::KeepLast(10));
        reference_paths_subscription_qos.transient_local();
        reference_paths_subscription_qos.reliable();
        paired_mission_and_reference_path_sub_ =
          node->create_subscription<capella_ros_msg::msg::LaneCenterPaths>(
          cfg_->wall_line.paired_mission_and_reference_path_topic,
          reference_paths_subscription_qos,
          std::bind(
            &TebLocalPlannerROS::pairedMissionAndReferencePathCallback, this,
            std::placeholders::_1));
        RCLCPP_INFO(
          logger_, "TEB 订阅成对 mission/reference 路径，话题：%s (transient_local)",
          cfg_->wall_line.paired_mission_and_reference_path_topic.c_str());
        removed_plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
          cfg_->wall_line.removed_plan_topic,
          rclcpp::QoS(rclcpp::KeepLast(5)),
          std::bind(&TebLocalPlannerROS::removedPlanCallback, this, std::placeholders::_1));
        RCLCPP_INFO(
          logger_, "TEB 订阅 removed_plan，话题：%s",
          cfg_->wall_line.removed_plan_topic.c_str());
      }
    }

    
    // initialize failure detector (reuse controller_frequency from control_duration_ above)
    failure_detector_.setBufferLength(
      std::round(cfg_->recovery.oscillation_filter_duration * controller_frequency));

    // set initialized flag
    initialized_ = true;

    // This should be called since to prevent different time sources exception
    time_last_infeasible_plan_ = clock_->now();
    time_last_oscillation_ = clock_->now();
    RCLCPP_DEBUG(logger_, "teb_local_planner 插件已初始化");

    transformed_path = node->create_publisher<nav_msgs::msg::Path>("teb_transformed_path", 1);
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("teb_global_plan", 1);
    safe_terminal_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
      "teb_transformed_plan_safe_terminal", 1);
    occupied_goal_stop_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
      "teb_last_occupied_goal_stop", 1);
    wall_line_marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("teb_selected_wall_line", 1);
    monitor_corridor_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_monitor_corridor_markers", 1);
    protruding_obstacle_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_protruding_obstacle_markers", 1);
    corner_approach_footprint_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_corner_approach_footprint_markers", 1);
    collision_footprint_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_collision_footprint_markers", 1);
    near_horizon_debug_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_near_horizon_debug", 1);
    path_wait_debug_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("teb_path_wait_debug", 1);
    edge_distance_publisher_  = node->create_publisher<std_msgs::msg::Float32>("edge_distance", 1);
    steering_angular_velocity_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "/front_steering/steering_angular_velocity", 10);
    steering_angular_acceleration_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "/front_steering/steering_angular_acceleration", 10);
    optimized_steering_angular_velocity_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "/front_steering/optimized_steering_angular_velocity", 10);
    optimized_steering_angular_acceleration_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "/front_steering/optimized_steering_angular_acceleration", 10);
    steering_publish_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(std::max(0.02, control_duration_)),
      std::bind(&TebLocalPlannerROS::publishFrontSteeringTopics, this));
    RCLCPP_INFO(logger_,
                "TEB 创建发布话题 /teb_selected_wall_line、/teb_monitor_corridor_markers、"
                "/teb_protruding_obstacle_markers、/teb_corner_approach_footprint_markers、"
                "/teb_collision_footprint_markers、/teb_near_horizon_debug、/teb_path_wait_debug、/edge_distance、"
                "/teb_transformed_path、/teb_global_plan、"
                "/teb_transformed_plan_safe_terminal、/teb_last_occupied_goal_stop、"
                "/front_steering/steering_angular_velocity、/front_steering/steering_angular_acceleration、"
                "/front_steering/optimized_steering_angular_velocity、"
                "/front_steering/optimized_steering_angular_acceleration");

    // Create persistent client for static_layer parameter updates
    static_layer_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
    RCLCPP_INFO(logger_, "已创建 static_layer 参数客户端");

    // Create persistent clients for footprint parameter updates
    local_footprint_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
    global_footprint_client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/global_costmap/global_costmap/set_parameters");
    RCLCPP_INFO(logger_, "已创建局部/全局代价地图 footprint 参数客户端");

    // Get current footprint values from local and global costmaps for restoration via service calls
    // Use background threads to avoid blocking initialization
    spawnBackgroundTask([this, node]() {
      auto local_get_params_client = node->create_client<rcl_interfaces::srv::GetParameters>(
        "/local_costmap/local_costmap/get_parameters");
      if (!waitForServiceInterruptible(local_get_params_client, std::chrono::seconds(30))) {
        if (isBackgroundWorkAllowed()) {
          RCLCPP_WARN(logger_, "局部代价地图 get_parameters 服务 30 秒内不可用");
        }
        return;
      }
      if (!isBackgroundWorkAllowed()) {
        return;
      }

      auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      get_request->names.push_back("footprint");
      auto future = local_get_params_client->async_send_request(get_request);
      if (!waitForFutureInterruptible(
            future, std::chrono::seconds(5),
            [this]() { return isBackgroundWorkAllowed(); })) {
        if (isBackgroundWorkAllowed()) {
          RCLCPP_WARN(logger_, "局部代价地图 get_parameters 服务调用超时");
        }
        return;
      }

      auto response = future.get();
      if (!response->values.empty()) {
        if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
          local_costmap_footprint_ = response->values[0].string_value;
          RCLCPP_INFO(logger_, "已保存局部代价地图 footprint: %s", local_costmap_footprint_.c_str());
        } else if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          const auto& local_footprint_array = response->values[0].double_array_value;
          local_costmap_footprint_ = "[";
          for (size_t i = 0; i < local_footprint_array.size(); i += 2) {
            if (i + 1 < local_footprint_array.size()) {
              local_costmap_footprint_ += "[" + std::to_string(local_footprint_array[i]) + ", " + std::to_string(local_footprint_array[i+1]) + "]";
              if (i + 2 < local_footprint_array.size()) local_costmap_footprint_ += ", ";
            }
          }
          local_costmap_footprint_ += "]";
          RCLCPP_INFO(logger_, "已保存局部代价地图 footprint: %s", local_costmap_footprint_.c_str());
        } else {
          RCLCPP_WARN(logger_, "局部代价地图 footprint 类型未知: %d", response->values[0].type);
        }
      } else {
        RCLCPP_WARN(logger_, "响应中未找到局部代价地图 footprint");
      }
    });

    spawnBackgroundTask([this, node]() {
      auto global_get_params_client = node->create_client<rcl_interfaces::srv::GetParameters>(
        "/global_costmap/global_costmap/get_parameters");
      if (!waitForServiceInterruptible(global_get_params_client, std::chrono::seconds(30))) {
        if (isBackgroundWorkAllowed()) {
          RCLCPP_WARN(logger_, "全局代价地图 get_parameters 服务 30 秒内不可用");
        }
        return;
      }
      if (!isBackgroundWorkAllowed()) {
        return;
      }

      auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      get_request->names.push_back("footprint");
      auto future = global_get_params_client->async_send_request(get_request);
      if (!waitForFutureInterruptible(
            future, std::chrono::seconds(5),
            [this]() { return isBackgroundWorkAllowed(); })) {
        if (isBackgroundWorkAllowed()) {
          RCLCPP_WARN(logger_, "全局代价地图 get_parameters 服务调用超时");
        }
        return;
      }

      auto response = future.get();
      if (!response->values.empty()) {
        if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
          global_costmap_footprint_ = response->values[0].string_value;
          RCLCPP_INFO(logger_, "已保存全局代价地图 footprint: %s", global_costmap_footprint_.c_str());
        } else if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          const auto& global_footprint_array = response->values[0].double_array_value;
          global_costmap_footprint_ = "[";
          for (size_t i = 0; i < global_footprint_array.size(); i += 2) {
            if (i + 1 < global_footprint_array.size()) {
              global_costmap_footprint_ += "[" + std::to_string(global_footprint_array[i]) + ", " + std::to_string(global_footprint_array[i+1]) + "]";
              if (i + 2 < global_footprint_array.size()) global_costmap_footprint_ += ", ";
            }
          }
          global_costmap_footprint_ += "]";
          RCLCPP_INFO(logger_, "已保存全局代价地图 footprint: %s", global_costmap_footprint_.c_str());
        } else {
          RCLCPP_WARN(logger_, "全局代价地图 footprint 类型未知: %d", response->values[0].type);
        }
      } else {
        RCLCPP_WARN(logger_, "响应中未找到全局代价地图 footprint");
      }
    });
   }
   else
   {
     RCLCPP_INFO(logger_, "teb_local_planner 已初始化，跳过重复初始化");
   }
}

void TebLocalPlannerROS::initializeFrontSteerDriveInterface(
  nav2_util::LifecycleNode::SharedPtr node)
{
  if (front_steer_interface_mode_ != "cmd_vel" &&
      front_steer_interface_mode_ != "front_steer_cmd")
  {
    throw std::invalid_argument(
      "front_steer_drive_interface.mode must be cmd_vel or front_steer_cmd");
  }
  if (!(std::isfinite(front_steer_state_timeout_) && front_steer_state_timeout_ > 0.0))
  {
    throw std::invalid_argument(
      "front_steer_drive_interface.state_timeout must be finite and > 0");
  }
  if (!std::isfinite(front_steer_projection_tolerance_) || front_steer_projection_tolerance_ < 0.0)
  {
    throw std::invalid_argument("max_projection_omega_error must be finite and >= 0");
  }
  if (front_steer_interface_mode_ == "front_steer_cmd" &&
      (!cfg_->robot.front_steering_rear_drive || cfg_->robot.cmd_angle_instead_rotvel ||
       !validFrontSteerConfiguration(cfg_->robot)))
  {
    throw std::invalid_argument(
      "front_steer_cmd mode requires a valid front-steering configuration and cmd_angle_instead_rotvel=false");
  }
  if (front_steer_command_topic_.empty() || front_steer_state_topic_.empty())
  {
    throw std::invalid_argument("front steer command/state topics must not be empty");
  }

  // This diagnostic switch is intentionally mutable while the interface
  // configuration itself remains fixed until the next configure cycle.
  declare_parameter_if_not_declared(node,
    name_ + ".front_steer_drive_interface.debug_logging", rclcpp::ParameterValue(false));
  bool debug_logging = false;
  node->get_parameter(name_ + ".front_steer_drive_interface.debug_logging", debug_logging);
  front_steer_debug_logging_.store(debug_logging);

  if (front_steer_odom_enabled_) {
    capella_ros_msg::msg::FrontSteerDriveStu probe;
    if (front_steer_odom_topic_.empty() || !odomToFrontSteerState(
        geometry_msgs::msg::Twist(), cfg_->robot, front_steer_odom_wheel_radius_, probe,
        front_steer_odom_angular_deadband_)) {
      throw std::invalid_argument("Invalid odom_to_state topic, geometry, wheel_radius, or angular_deadband");
    }
    front_steer_sim_state_pub_ = node->create_publisher<
      capella_ros_msg::msg::FrontSteerDriveStu>(
        front_steer_state_topic_, rclcpp::SystemDefaultsQoS());
    front_steer_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      front_steer_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TebLocalPlannerROS::frontSteerOdomCallback, this, std::placeholders::_1));
    RCLCPP_WARN(logger_, "仿真 odom 转换已开启: %s -> %s；请停用真实底盘反馈发布器",
      front_steer_odom_topic_.c_str(), front_steer_state_topic_.c_str());
  }

  front_steer_publish_valid_ = false;
  front_steer_control_dt_prepared_ = false;
  front_steer_aligning_ = false;
  front_steer_lag_speed_limit_ = std::numeric_limits<double>::infinity();
  front_steer_published_linear_ = 0.0;
  front_steer_published_angle_ = 0.0;
  front_steer_published_rate_ = 0.0;
  front_steering_cmd_valid_ = false;
  {
    std::lock_guard<std::mutex> lock(front_steer_stu_mutex_);
    front_steer_stu_valid_ = false;
  }
  front_steer_params_handler_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      const std::string prefix = name_ + ".front_steer_drive_interface.";
      bool debug_change = false;
      bool debug_value = false;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == prefix + "debug_logging") {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "debug_logging 必须是布尔值";
            break;
          }
          debug_change = true;
          debug_value = parameter.as_bool();
          continue;
        }
        if (parameter.get_name().compare(0, prefix.size(), prefix) == 0) {
          result.successful = false;
          result.reason = "Front-steer interface parameters are fixed during configure";
          break;
        }
      }
      if (result.successful && debug_change) {
        front_steer_debug_logging_.store(debug_value);
      }
      return result;
    });
  // Always subscribe to feedback.  Legacy cmd_vel mode can fall back to the
  // Nav2 velocity, while direct mode treats missing feedback as a fault.
  front_steer_stu_sub_ = node->create_subscription<
    capella_ros_msg::msg::FrontSteerDriveStu>(
      front_steer_state_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TebLocalPlannerROS::frontSteerDriveStuCallback, this, std::placeholders::_1));

  front_steer_stu_linear_debug_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    front_steer_state_topic_ + "/debug/linear_x", rclcpp::SystemDefaultsQoS());
  front_steer_stu_angle_debug_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    front_steer_state_topic_ + "/debug/front_steer_angle", rclcpp::SystemDefaultsQoS());
  front_steer_stu_wheel_debug_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    front_steer_state_topic_ + "/debug/front_wheel_speed", rclcpp::SystemDefaultsQoS());
  front_steer_measured_rate_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    "/front_steering/measured_steering_angular_velocity", rclcpp::SystemDefaultsQoS());

  if (front_steer_interface_mode_ == "front_steer_cmd")
  {
    front_steer_cmd_pub_ = node->create_publisher<
      capella_ros_msg::msg::FrontSteerDriveCmd>(
        front_steer_command_topic_, rclcpp::SystemDefaultsQoS());
    front_steer_cmd_linear_debug_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      front_steer_command_topic_ + "/debug/linear_x", rclcpp::SystemDefaultsQoS());
    front_steer_cmd_angle_debug_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      front_steer_command_topic_ + "/debug/front_steer_angle", rclcpp::SystemDefaultsQoS());
    front_steer_cmd_twist_debug_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      front_steer_command_topic_ + "/debug/twist", rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      logger_, "TEB 使用前轮转向直连接口: %s <- %s (默认 QoS, state_timeout=%.3fs)",
      front_steer_command_topic_.c_str(), front_steer_state_topic_.c_str(),
      front_steer_state_timeout_);
  }
  else
  {
    RCLCPP_INFO(
      logger_, "TEB 使用传统 cmd_vel 接口: %s；底盘状态 %s 有效时优先用于规划",
      legacy_command_topic_.c_str(), front_steer_state_topic_.c_str());
  }
}

void TebLocalPlannerROS::frontSteerOdomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!front_steer_odom_enabled_ || !front_steer_sim_state_pub_) {return;}
  capella_ros_msg::msg::FrontSteerDriveStu state;
  bool valid;
  {
    std::lock_guard<std::mutex> lock(cfg_->configMutex());
    valid = odomToFrontSteerState(
      msg->twist.twist, cfg_->robot, front_steer_odom_wheel_radius_, state,
      front_steer_odom_angular_deadband_);
  }
  if (!valid) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "丢弃非法或不符合前轮转向模型的 odom，不刷新 /front_steer_stu");
    return;
  }
  state.header.stamp = msg->header.stamp;
  if (state.header.stamp.sec == 0 && state.header.stamp.nanosec == 0 && clock_) {
    state.header.stamp = clock_->now();
  }
  state.header.frame_id = !msg->child_frame_id.empty() ?
    msg->child_frame_id : msg->header.frame_id;
  front_steer_sim_state_pub_->publish(state);
}

void TebLocalPlannerROS::frontSteerDriveStuCallback(
  const capella_ros_msg::msg::FrontSteerDriveStu::ConstSharedPtr msg)
{
  const auto receive_time = std::chrono::steady_clock::now();
  // front_wheel_speed is the steering joint output RPM about +Z, not the
  // rolling speed of the front tire. Keep its sign when converting to rad/s.
  const double measured_rate = static_cast<double>(msg->front_wheel_speed) *
    (2.0 * std::acos(-1.0) / 60.0);
  const bool finite = std::isfinite(static_cast<double>(msg->linear_x)) &&
    std::isfinite(static_cast<double>(msg->front_steer_angle)) &&
    std::isfinite(static_cast<double>(msg->front_wheel_speed)) && std::isfinite(measured_rate);
  {
    std::lock_guard<std::mutex> lock(front_steer_stu_mutex_);
    front_steer_stu_ = *msg;
    front_steer_stu_receive_time_ = receive_time;
    front_steer_stu_valid_ = finite;
  }
  // Mirror raw feedback, including invalid fields, so faults remain visible in rqt.
  // Visualization failure must not prevent caching and validating chassis feedback.
  try {
    std_msgs::msg::Float64 value;
    value.data = msg->linear_x;
    if (front_steer_stu_linear_debug_pub_) {front_steer_stu_linear_debug_pub_->publish(value);}
    value.data = msg->front_steer_angle;
    if (front_steer_stu_angle_debug_pub_) {front_steer_stu_angle_debug_pub_->publish(value);}
    value.data = msg->front_wheel_speed;
    if (front_steer_stu_wheel_debug_pub_) {front_steer_stu_wheel_debug_pub_->publish(value);}
    if (std::isfinite(measured_rate) && front_steer_measured_rate_pub_) {
      value.data = measured_rate;
      front_steer_measured_rate_pub_->publish(value);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "Failed to publish front-steer feedback visualization: %s", ex.what());
  }
  if (!finite)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "收到非法 /front_steer_stu，状态将被视为无效 (linear_x=%.3f angle=%.3f wheel_rpm=%.3f)",
      static_cast<double>(msg->linear_x), static_cast<double>(msg->front_steer_angle),
      static_cast<double>(msg->front_wheel_speed));
  }
  if (front_steer_debug_logging_.load()) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
      "前轮转向底盘反馈：v=%.6f米/秒，转角=%.6f弧度，转向轴转速=%.6f转/分，"
      "转向角速度=%.6f弧度/秒，有效=%d，消息时间=%d.%09u",
      static_cast<double>(msg->linear_x), static_cast<double>(msg->front_steer_angle),
      static_cast<double>(msg->front_wheel_speed), measured_rate, finite ? 1 : 0,
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  }
}

bool TebLocalPlannerROS::getFreshFrontSteerVelocity(geometry_msgs::msg::Twist & velocity)
{
  capella_ros_msg::msg::FrontSteerDriveStu state;
  std::chrono::steady_clock::time_point receive_time;
  bool valid = false;
  {
    std::lock_guard<std::mutex> lock(front_steer_stu_mutex_);
    state = front_steer_stu_;
    receive_time = front_steer_stu_receive_time_;
    valid = front_steer_stu_valid_;
  }
  if (!valid) {return false;}
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - receive_time).count();
  const bool usable = frontSteerStateToVelocity(
    state, age, front_steer_state_timeout_, cfg_->robot, velocity);
  RCLCPP_DEBUG(logger_,
    "Front-steer feedback valid=%d age=%.6f v=%.6f delta=%.6f front_wheel_rpm=%.3f",
    usable ? 1 : 0, age, static_cast<double>(state.linear_x),
    static_cast<double>(state.front_steer_angle), static_cast<double>(state.front_wheel_speed));
  return usable;
}

void TebLocalPlannerROS::selectFrontSteerVelocity(
  const geometry_msgs::msg::Twist & nav2_velocity)
{
  robot_vel_ = nav2_velocity;
  geometry_msgs::msg::Twist chassis_velocity;
  const bool fresh_chassis_state = getFreshFrontSteerVelocity(chassis_velocity);
  if (fresh_chassis_state) {
    robot_vel_ = chassis_velocity;
  } else if (front_steer_interface_mode_ == "front_steer_cmd") {
    robot_vel_ = geometry_msgs::msg::Twist();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "前轮转向直连接口没有新鲜有效的 /front_steer_stu，停止并等待底盘状态");
    if (planner_) {planner_->clearPlanner();}
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    throw nav2_core::PlannerException(
      "TebLocalPlannerROS: front_steer_stu missing, invalid, or timed out");
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "未使用新鲜 /front_steer_stu，cmd_vel 模式回退到 Nav2 velocity");
  }
}

bool TebLocalPlannerROS::publishFrontSteerCommand(
  double linear_x, double steering_angle, double omega)
{
  if (!front_steer_cmd_pub_) {return true;}
  const bool stationary_steering = front_steer_aligning_ && linear_x == 0.0 &&
    steering_angle != 0.0 && omega == 0.0;
  const bool stop = !stationary_steering && std::isfinite(linear_x) &&
    std::isfinite(steering_angle) && std::abs(linear_x) <= cfg_->robot.steering_min_speed;
  // Projection and final dynamics validation must use one interval. Recomputing
  // it after visualization changes the inferred previous/current steering rate.
  if (front_steer_publish_valid_ && !front_steer_control_dt_prepared_) {
    front_steer_control_dt_ = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - front_steer_last_publish_time_).count();
  }
  front_steer_control_dt_prepared_ = false;
  geometry_msgs::msg::Twist feedback;
  const double publish_age = front_steer_publish_valid_ ? std::chrono::duration<double>(
    std::chrono::steady_clock::now() - front_steer_last_publish_time_).count() : 0.0;
  const char * reason = nullptr;
  if (!stop && (!std::isfinite(front_steer_control_dt_) ||
      front_steer_control_dt_ <= 1e-4 || front_steer_control_dt_ > front_steer_state_timeout_ ||
      publish_age > front_steer_state_timeout_)) {
    reason = "untrusted control interval";
  } else if (!stop && !getFreshFrontSteerVelocity(feedback)) {
    reason = "missing, invalid or stale chassis feedback";
  } else {
    reason = frontSteerCommandError(linear_x, steering_angle, omega, front_steer_control_dt_,
      cfg_->robot, front_steer_published_angle_, front_steer_published_rate_, stationary_steering);
  }
  if (!reason && !stop && !stationary_steering) {
    const double linear_dt = front_steer_publish_valid_ ?
      std::min(front_steer_control_dt_, publish_age) : front_steer_control_dt_;
    linear_x = limitFrontSteerLinearAcceleration(
      linear_x, front_steer_published_linear_, cfg_->robot.acc_lim_x, linear_dt);
    // Deceleration needed for current velocity/yaw limits is safety-critical.
    // Preserve the steering angle; never change it to undo the speed ramp.
    if (!std::isfinite(linear_x)) {
      reason = "invalid longitudinal acceleration interval or history";
    } else {
      linear_x = clampDouble(linear_x, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x);
      const double curvature = std::tan(steering_angle) / cfg_->robot.wheelbase;
      if (std::abs(curvature) > 0.0 && std::abs(linear_x * curvature) > cfg_->robot.max_vel_theta) {
        linear_x = std::copysign(cfg_->robot.max_vel_theta / std::abs(curvature), linear_x);
      }
      if (front_steer_lag_speed_scaling_ && std::isfinite(front_steer_lag_speed_limit_) &&
          std::abs(linear_x) > front_steer_lag_speed_limit_) {
        linear_x = std::copysign(front_steer_lag_speed_limit_, linear_x);
      }
      omega = linear_x * curvature;
      // Deliberate ramp samples below steering_min_speed must not be converted
      // to a stop, otherwise 0 -> 0.03 m/s would reset the ramp every cycle.
      reason = frontSteerCommandError(linear_x, steering_angle, omega, front_steer_control_dt_,
        cfg_->robot, front_steer_published_angle_, front_steer_published_rate_, true);
    }
  }
  if (reason) {
    if (front_steer_debug_logging_.load()) {
      RCLCPP_ERROR(logger_,
        "拒绝前轮转向指令：v=%.6f，转角=%.6f，偏航角速度=%.6f，dt=%.6f，"
        "上次转角=%.6f，上次转向角速度=%.6f，原因=%s",
        linear_x, steering_angle, omega, front_steer_control_dt_,
        front_steer_published_angle_, front_steer_published_rate_, reason);
    } else {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
        "拒绝前轮转向指令：v=%.6f，转角=%.6f，偏航角速度=%.6f，dt=%.6f，"
        "上次转角=%.6f，上次转向角速度=%.6f，原因=%s",
        linear_x, steering_angle, omega, front_steer_control_dt_,
        front_steer_published_angle_, front_steer_published_rate_, reason);
    }
  }
  capella_ros_msg::msg::FrontSteerDriveCmd command;
  if (!reason && !stop) {
    command.linear_x = static_cast<float>(linear_x);
    command.front_steer_angle = static_cast<float>(steering_angle);
  }
  front_steer_cmd_pub_->publish(command);
  const double rate = (!reason && !stop && front_steer_control_dt_ > 0.0)
    ? (steering_angle - front_steer_published_angle_) / front_steer_control_dt_ : 0.0;
  front_steer_published_linear_ = command.linear_x;
  front_steer_published_angle_ = command.front_steer_angle;
  front_steer_published_rate_ = rate;
  front_steer_last_publish_time_ = std::chrono::steady_clock::now();
  front_steer_publish_valid_ = true;
  // Keep recovery analysis, rear-wheel diagnostics and Nav2's returned Twist
  // consistent with the actual float32 command sent after longitudinal slew.
  last_cmd_ = geometry_msgs::msg::Twist();
  last_cmd_.linear.x = command.linear_x;
  if (command.linear_x != 0.0f) {
    last_cmd_.angular.z = static_cast<double>(command.linear_x) *
      std::tan(static_cast<double>(command.front_steer_angle)) / cfg_->robot.wheelbase;
  }
  front_steering_rear_left_ = last_cmd_.linear.x - 0.5 * cfg_->robot.rear_track * last_cmd_.angular.z;
  front_steering_rear_right_ = last_cmd_.linear.x + 0.5 * cfg_->robot.rear_track * last_cmd_.angular.z;
  if (front_steer_debug_logging_.load()) {
    RCLCPP_INFO(logger_,
      "前轮转向最终下发：v=%.6f米/秒，转角=%.6f弧度，偏航角速度=%.6f弧度/秒，"
      "转向角速度=%.6f弧度/秒，dt=%.6f秒，停止=%d，原因=%s",
      static_cast<double>(command.linear_x), static_cast<double>(command.front_steer_angle),
      last_cmd_.angular.z, rate, front_steer_control_dt_, stop ? 1 : 0,
      reason ? reason : "无");
  }
  if (reason || stop) {
    front_steer_aligning_ = false;
    // A safety stop intentionally overrides steering dynamics. Resume from the
    // command actually sent, never the discarded projection from this cycle.
    std::lock_guard<std::mutex> lock(steering_topic_mutex_);
    front_steering_cmd_valid_ = true;
    front_steering_delta_ = 0.0;
    front_steering_delta_rate_ = 0.0;
    front_steering_delta_acceleration_ = 0.0;
    last_cmd_ = geometry_msgs::msg::Twist();
  }
  // Mirror the message actually sent, after all limiting and safety-stop decisions.
  // Twist retains its standard v/omega meaning; steering angle has a scalar topic.
  try {
    std_msgs::msg::Float64 value;
    value.data = command.linear_x;
    if (front_steer_cmd_linear_debug_pub_) {front_steer_cmd_linear_debug_pub_->publish(value);}
    value.data = command.front_steer_angle;
    if (front_steer_cmd_angle_debug_pub_) {front_steer_cmd_angle_debug_pub_->publish(value);}
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.linear_x;
    if (command.linear_x != 0.0f) {
      twist.angular.z = static_cast<double>(command.linear_x) *
        std::tan(static_cast<double>(command.front_steer_angle)) / cfg_->robot.wheelbase;
    }
    if (front_steer_cmd_twist_debug_pub_) {front_steer_cmd_twist_debug_pub_->publish(twist);}
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "Failed to publish front-steer command visualization: %s", ex.what());
  }
  return reason == nullptr;
}

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
     RCLCPP_ERROR(logger_, "teb_local_planner 尚未初始化，请先调用 initialize()");
     return;
   }
 
   // store the global plan
   constexpr double kMinAdjacentSpacing = 1e-5;
   auto orientationIsFinite = [](const geometry_msgs::msg::Quaternion & orientation) -> bool {
     if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
         !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
     {
       return false;
     }
     return std::isfinite(tf2::getYaw(orientation));
   };

   global_plan_.clear();
   global_plan_.reserve(orig_global_plan.poses.size());
   origin_plan_.clear();
   origin_plan_.reserve(orig_global_plan.poses.size());
   size_t dropped_nan_orientation = 0;
   size_t dropped_close_pose = 0;
   for (const auto & in_pose : orig_global_plan.poses) {
     geometry_msgs::msg::PoseStamped out_pose;
     out_pose.pose = in_pose.pose;
     out_pose.header = orig_global_plan.header;
     const bool orientation_finite = orientationIsFinite(out_pose.pose.orientation);
     if (!global_plan_.empty()) {
       const double dx = out_pose.pose.position.x - global_plan_.back().pose.position.x;
       const double dy = out_pose.pose.position.y - global_plan_.back().pose.position.y;
       if (std::hypot(dx, dy) < kMinAdjacentSpacing) {
         // 朝向都有效时删后一个；否则删朝向无效的点（保留点的朝向已经有效）。
         if (orientation_finite) {
           ++dropped_close_pose;
         } else {
           ++dropped_nan_orientation;
         }
         continue;
       }
     }
     if (!orientation_finite) {
       ++dropped_nan_orientation;
       continue;
     }
     global_plan_.push_back(out_pose);
     origin_plan_.push_back(out_pose);
   }
   if (dropped_nan_orientation > 0 || dropped_close_pose > 0) {
     RCLCPP_WARN_THROTTLE(
       logger_, *clock_, kTebLogThrottleMs,
       "setPlan：剔除无效路径点，方向无效 %zu，相邻距离 < %.0e %zu，剩余 %zu / %zu",
       dropped_nan_orientation, kMinAdjacentSpacing, dropped_close_pose,
       global_plan_.size(), orig_global_plan.poses.size());
   }
   if (!global_plan_.empty()) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
                         "TEB 设置全局路径，路径点数：%zu", global_plan_.size());
   } else {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, kTebLogThrottleMs, "TEB 设置全局路径失败，路径点数为0");
   }

   if (major_arc_active_) {
     RCLCPP_INFO_THROTTLE(logger_, *clock_, kTebLogThrottleMs, "setPlan：清除优弧锁");
   }
   resetMajorArcLock();
   goal_standoff_latched_ = false;
   goal_standoff_plan_idx_ = -1;
   goal_standoff_force_hold_ = false;
   goal_standoff_hold_active_ = false;
   occupied_goal_no_standoff_ = false;
   occupied_goal_dist_m_ = std::numeric_limits<double>::infinity();
   occupied_goal_near_hold_active_ = false;
   occupied_goal_near_hold_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
   occupied_goal_near_clear_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
 
 
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
     publishFrontSteerCommand(0.0, 0.0);
     throw nav2_core::PlannerException(
       std::string("teb_local_planner 尚未初始化，请先调用 initialize()")
     );
   }
 
   geometry_msgs::msg::TwistStamped cmd_vel;
   
   cmd_vel.header.stamp = clock_->now();
   cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
   cmd_vel.twist.linear.x = 0;
   cmd_vel.twist.linear.y = 0;
   cmd_vel.twist.angular.z = 0;

  front_steer_control_dt_prepared_ = false;
  front_steer_aligning_ = false;
  front_steer_lag_speed_limit_ = std::numeric_limits<double>::infinity();
  front_steer_control_dt_ = front_steer_publish_valid_
    ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
        front_steer_last_publish_time_).count() : control_duration_;

  // Any exceptional or early-return path leaves the direct interface stopped.
  // This also covers planner exceptions raised below the normal return path.
  bool front_steer_command_sent = false;
  struct FrontSteerCommandGuard
  {
    TebLocalPlannerROS * planner;
    bool * sent;
    ~FrontSteerCommandGuard()
    {
      if (planner && sent && !*sent) {
        try {planner->publishFrontSteerCommand(0.0, 0.0);}
        catch (const std::exception & ex) {
          RCLCPP_ERROR(planner->logger_, "Failed to publish safety stop: %s", ex.what());
        }
      }
    }
  } front_steer_command_guard{this, &front_steer_command_sent};
 
   // Update for the current goal checker's state
   geometry_msgs::msg::Pose pose_tolerance;
   geometry_msgs::msg::Twist vel_tolerance;
   if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
     RCLCPP_WARN(logger_, "无法获取目标检查器 (goal checker) 容差");
   } else {
     cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
   }
   
   // Get robot pose
   robot_pose_ = PoseSE2(pose.pose);
   geometry_msgs::msg::PoseStamped robot_pose;
   robot_pose.header = pose.header;
   robot_pose_.toPoseMsg(robot_pose.pose);

   // 退出贴边后：大轮廓碰撞门控 / 超时强制恢复
   updatePendingNormalFootprintRestore();
   
  selectFrontSteerVelocity(velocity);

   if (cfg_->robot.map_to_base_transform_max_age > 0.0) {
     const std::string & base_frame = costmap_ros_->getBaseFrameID();
     try {
       geometry_msgs::msg::TransformStamped tf_map_to_base =
         tf_->lookupTransform(
           cfg_->map_frame,
           base_frame,
           tf2::TimePointZero,
           tf2::durationFromSec(std::max(0.0, cfg_->robot.transform_tolerance)));
       const rclcpp::Time tf_stamp(tf_map_to_base.header.stamp, clock_->get_clock_type());
       const double data_age_sec = std::max(0.0, (clock_->now() - tf_stamp).seconds());
       if (data_age_sec > cfg_->robot.map_to_base_transform_max_age) {
         RCLCPP_WARN_THROTTLE(
           logger_,
           *clock_,
           1000,
           "%s → %s 的 TF 数据滞后, 滞后 %.3f s (阈值 %.3f s), 输出零速度",
           cfg_->map_frame.c_str(),
           base_frame.c_str(),
           data_age_sec,
           cfg_->robot.map_to_base_transform_max_age);
         return cmd_vel;
       }
     } catch (const tf2::TransformException & ex) {
       RCLCPP_WARN_THROTTLE(
         logger_,
         *clock_,
         1000,
         "无法查询 %s → %s 变换: %s, 输出零速度",
         cfg_->map_frame.c_str(),
         base_frame.c_str(),
         ex.what());
       return cmd_vel;
     }
   }
   
  // prune global plan to cut off parts of the past (spatially before the robot)
  // Coarse pass: remove obviously-passed points using a larger distance threshold and an
  // optionally unlimited search range. This prevents the fine pass from skipping too far
  // ahead when the robot detours around a large obstacle.
  if (cfg_->trajectory.rough_global_plan_prune_distance > 0.0)
  {
  pruneGlobalPlan(robot_pose, global_plan_,
                  cfg_->trajectory.rough_global_plan_prune_distance,
                  cfg_->trajectory.rough_global_plan_prune_max_accum_dist);
  }
  // Fine pass: precise pruning with the normal (smaller) distance and limited search range.
  pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance,
                cfg_->trajectory.global_plan_prune_max_accum_dist);

  pruneArrivedLockedCorner(robot_pose);
  // 角点删尾和贴边改写之前，按路径顺序从当前点向前截取
  const bool reverse_segment = hasReverseSegmentAhead();

  {
    std::lock_guard<std::mutex> l(speed_limit_mutex_);
    if (has_speed_limit_) {
      const double limit = std::max(0.0, speed_limit_linear_x_);
      if (std::isfinite(limit) && speed_limit_linear_x_ > 0) {
        if (limit < safe_linear_speed_limit_ && cfg_->robot.max_vel_x != limit) {
          cfg_->robot.max_vel_x = std::min(limit, safe_linear_speed_limit_);
          RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000, "正在应用速度限制，当前最大线速度: %.2f m/s", cfg_->robot.max_vel_x);
        } else if (limit >= safe_linear_speed_limit_ && cfg_->robot.max_vel_x != safe_linear_speed_limit_) {
          cfg_->robot.max_vel_x = safe_linear_speed_limit_;
          RCLCPP_WARN_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
            "收到的速度限制大于安全线速度上限，已钳到安全上限: %.2f m/s", safe_linear_speed_limit_);
        }
      } else if (!std::isfinite(limit) || limit <= 0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
          "速度限制无效 (非有限或 <=0)，当前速度上限: %.2f m/s", cfg_->robot.max_vel_x);
      }
    }
  }

  geometry_msgs::msg::PoseStamped corner_pose_global, corner_pose_robot;
  bool corner_found = false;
  size_t corner_index = 0;

  if (!global_plan_.empty()) {
    geometry_msgs::msg::PoseStamped robot_in_plan;
    bool robot_in_plan_ok = false;
    try {
      geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
        global_plan_.front().header.frame_id,
        robot_pose.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(0.5));
      tf2::doTransform(robot_pose, robot_in_plan, global_to_plan_transform);
      robot_in_plan_ok = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "角点搜索: 机器人位姿变换到路径坐标系失败: %s", ex.what());
    }

    if (robot_in_plan_ok) {
      double sq_dist_threshold = 1e10;
      if (costmap_ != nullptr) {
        double dist_threshold = std::max(
          costmap_->getSizeInCellsX() * costmap_->getResolution() / 2.0,
          costmap_->getSizeInCellsY() * costmap_->getResolution() / 2.0);
        dist_threshold *= 0.85;
        sq_dist_threshold = dist_threshold * dist_threshold;
      }
      const double thresh_deg = static_cast<double>(cfg_->trajectory.theta_threshold);
      const double search_arc = cfg_->trajectory.max_global_plan_lookahead_dist > 1e-9
        ? (cfg_->trajectory.max_global_plan_lookahead_dist + 1.0)
        : -1.0;
      const int closest_raw = findEuclideanClosestPlanIdx(
        global_plan_, robot_in_plan, sq_dist_threshold,
        cfg_->trajectory.transform_global_plan_closest_search_max_accum_dist);
      const int closest_idx = clampClosestIdxBeforeUnprunedCorner(
        global_plan_, robot_in_plan, closest_raw, thresh_deg);
      const int locked_idx = lockedCornerIdx(
        global_plan_, closest_idx, thresh_deg, search_arc);
      if (locked_idx >= 0 && locked_idx < static_cast<int>(global_plan_.size())) {
        corner_found = true;
        corner_index = static_cast<size_t>(locked_idx);
        corner_pose_global = global_plan_.at(corner_index);
        RCLCPP_DEBUG_THROTTLE(
          logger_, *clock_, kTebLogThrottleMs,
          "Edge following: 找到最近的角点: %zu, 角度: %.2f",
          corner_index, polylineVertexAngleDeg(global_plan_, locked_idx));
      }
    }
  }

  if (corner_found)
  {
    corner_pose_global.header.frame_id = global_plan_.at(0).header.frame_id;
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
      try {
        if (tf_->canTransform(corner_pose_global.header.frame_id, costmap_ros_->getBaseFrameID(), tf2::TimePointZero)) {
          geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
            corner_pose_global.header.frame_id,
            costmap_ros_->getBaseFrameID(),
            tf2::TimePointZero);
          corner_pose_global.header.stamp = transform.header.stamp;
        } else {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
            "TF 查询失败 (ExtrapolationException): %s, 尝试使用最新可用时间", ex.what());
          corner_found = false;
        }
      } catch (const tf2::TransformException& ex2) {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, kTebLogThrottleErrorMs, "TF 查询失败: %s", ex2.what());
        corner_found = false;
      }
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, kTebLogThrottleErrorMs, "TF 查询失败: %s", ex.what());
      corner_found = false;
    }
    if (corner_found &&
        !nav2_util::transformPoseInTargetFrame(
          corner_pose_global, corner_pose_robot, *tf_,
          costmap_ros_->getBaseFrameID(), 0.5)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "角点位姿变换到机器人坐标系失败");
      corner_found = false;
    }
  }
  else
  {
    corner_pose_global = robot_pose;
    corner_pose_robot = robot_pose;
  }

  // Apply external speed limit (if any) before planning


  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;
  int prune_row_from_idx = -1;
  int prune_row_to_idx = -1;
  PathWindowKeyPoints path_key_points;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                          transformed_plan, &goal_idx, &tf_plan_to_global,
                          &prune_row_from_idx, &prune_row_to_idx, &path_key_points))
  {
    throw nav2_core::PlannerException(
      std::string("Could not transform the global plan to the frame of the controller")
    );
  }
  if (prune_row_from_idx >= 0 && prune_row_to_idx >= prune_row_from_idx)
  {
    const int last_keep = static_cast<int>(global_plan_.size()) - 1;
    if (last_keep >= 0 && prune_row_to_idx >= last_keep) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, kTebLogThrottleMs,
        "transformGlobalPlan: 裁剪区间含路径终点 [%d,%d]，改为保留最后一个点",
        prune_row_from_idx, prune_row_to_idx);
      prune_row_to_idx = last_keep - 1;
    }
    if (prune_row_from_idx <= prune_row_to_idx &&
        prune_row_to_idx >= 0 &&
        prune_row_from_idx < static_cast<int>(global_plan_.size()))
    {
      const int erase_count = prune_row_to_idx - prune_row_from_idx + 1;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, kTebLogThrottleMs,
        "transformGlobalPlan: 放弃当前行（无安全点或已到达行终点），"
        "删除 global_plan_[%d, %d]（%d 个点）；保留掉头/下一行",
        prune_row_from_idx, prune_row_to_idx, erase_count);
      global_plan_.erase(
        global_plan_.begin() + prune_row_from_idx,
        global_plan_.begin() + prune_row_to_idx + 1);
      if (goal_idx > prune_row_to_idx) {
        goal_idx -= erase_count;
      } else if (goal_idx >= prune_row_from_idx) {
        if (prune_row_from_idx < static_cast<int>(global_plan_.size())) {
          goal_idx = prune_row_from_idx;
        } else {
          goal_idx = std::max(0, static_cast<int>(global_plan_.size()) - 1);
        }
      }
    }
  }
  if (visualization_) {
    visualization_->publishPathKeyPoints(path_key_points);
  }
  // Check if corner has obstacle and prune path before corner if needed
  bool corner_has_obstacle = false;
  bool cut_path_before_corner = false;
  double cut_path_before_corner_distance = 0.0;
  if (corner_found && corner_pose_robot.pose.position.x < cfg_->trajectory.max_global_plan_lookahead_dist &&
      (corner_pose_robot.pose.position.x > 0.3 || 
      (corner_pose_robot.pose.position.x < 0 && 
      std::fabs(robot_vel_.linear.x) > cfg_->trajectory.prune_before_corner_linear_x_threshold)))
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
    corner_check_pose2d.theta = direction_to_corner;

    unsigned char corner_hit_cost = 0;
    const std::string& corner_cost_mode = cfg_->trajectory.corner_approach_check_cost_mode;
    bool corner_approach_blocked = false;
    int corner_blocked_sample_index = -1;
    if (cfg_->trajectory.corner_approach_ray_check_enable) {
      corner_approach_blocked = checkCornerApproachRayBlocked(
        robot_pose_local,
        corner_check_pose2d,
        cfg_->trajectory.corner_approach_check_sample_spacing,
        corner_cost_mode,
        corner_hit_cost,
        &corner_blocked_sample_index);
    } else {
      corner_approach_blocked = checkCornerFootprintPoseBlocked(
        corner_check_pose2d, corner_cost_mode, corner_hit_cost);
      if (corner_approach_blocked) {
        corner_blocked_sample_index = 0;
      } else {
        clearCollisionFootprintDebug("collision_corner", transformed_plan.empty() ?
          std::string() : transformed_plan.front().header.frame_id);
      }
    }

    if (cfg_->trajectory.corner_approach_footprint_marker_enable) {
      const std::string marker_frame_id = transformed_plan.empty() ?
        (costmap_ros_ ? costmap_ros_->getGlobalFrameID() : cfg_->map_frame) :
        transformed_plan.front().header.frame_id;
      publishCornerApproachFootprintMarkers(
        marker_frame_id,
        robot_pose_local,
        corner_check_pose2d,
        cfg_->trajectory.corner_approach_ray_check_enable,
        cfg_->trajectory.corner_approach_check_sample_spacing,
        corner_blocked_sample_index,
        corner_hit_cost);
    }

    if (corner_approach_blocked) {
      if (corner_hit_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        cut_path_before_corner_distance = cfg_->trajectory.cut_path_before_corner_lethal_dist;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, kTebLogThrottleMs,
          "角点/射线路径检测到 LETHAL_OBSTACLE cost=%u (ray_check=%s)",
          static_cast<unsigned>(corner_hit_cost),
          cfg_->trajectory.corner_approach_ray_check_enable ? "true" : "false");
      } else {
        cut_path_before_corner_distance = cfg_->trajectory.cut_path_before_corner_inscribed_dist;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, kTebLogThrottleMs,
          "角点/射线路径检测到 INSCRIBED_INFLATED_OBSTACLE cost=%u (ray_check=%s)",
          static_cast<unsigned>(corner_hit_cost),
          cfg_->trajectory.corner_approach_ray_check_enable ? "true" : "false");
      }
      corner_has_obstacle = true;
    }
    
    // If corner has obstacle, delete all path points before the corner in global_plan_
    if (corner_has_obstacle)
    {
      if (corner_index + 1 >= global_plan_.size()) {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, kTebLogThrottleMs,
          "角点为路径终点(idx=%zu size=%zu)，跳过删除以免掏空路径",
          corner_index, global_plan_.size());
      } else {
      double robot_corner_distance = pow(corner_pose_robot.pose.position.x, 2) + pow(corner_pose_robot.pose.position.y, 2);
      if (corner_index > 0 &&
          corner_index < global_plan_.size() &&
          robot_corner_distance < std::max(pow(cfg_->trajectory.max_global_plan_lookahead_dist / 2.0, 2), 
                                           pow(cut_path_before_corner_distance, 2)) &&
          std::fabs(robot_vel_.linear.x) > cfg_->trajectory.prune_before_corner_linear_x_threshold &&
          corner_pose_robot.pose.position.x < 0)
      {
        // Erase all points before the corner (keep the corner point itself)
        cut_path_before_corner = true;
        global_plan_.erase(global_plan_.begin(), global_plan_.begin() + corner_index + 1);
        RCLCPP_INFO_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
          "角点被障碍物占用，删除角点前的所有路径点，裁剪距离: %.2f m", robot_corner_distance);
      }
      }
      goto jump_prune_transformed_plan;
    }
    
    // If no obstacle, find corner in transformed_plan and prune after corner if needed
    const double corner_position_tolerance = 0.05;  // Tolerance for finding corner in transformed plan
    for (auto check_it = transformed_plan.begin(); 
        check_it != transformed_plan.end(); 
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
  } else if (cfg_->trajectory.corner_approach_footprint_marker_enable) {
    const std::string marker_frame_id = costmap_ros_ ?
      costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
    clearCornerApproachFootprintMarkers(marker_frame_id);
  }

  jump_prune_transformed_plan:
  if (detail::useCurbOrWallEnv() != nullptr && transformed_plan.size() > 1) {
     runEdgeFollowingPathUpdate(transformed_plan, robot_pose);
  }
 
  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);
  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  // --- 狭窄通道 / enable_backward：更新 latch，判断是否启用倒车友好 TEB 参数 ---
  updateNarrowPassageLatch(robot_pose);
  bool enable_backward_cmd = false;
  bool enable_backward_cmd_received = false;
  {
    std::lock_guard<std::mutex> lock(enable_backward_mutex_);
    enable_backward_cmd = enable_backward_cmd_;
    enable_backward_cmd_received = enable_backward_cmd_received_;
  }
  const bool narrow_mode = narrowPolygonsAvailable() && latched_narrow_passage_;
  if (!cfg_->rotation.dual_arc_reverse_enable && dual_arc_reverse_active_) {
    exitDualArcReverseLatch();
  }
  bool dual_arc_timeout_probe = false;
  const bool other_narrow_sources =
    (enable_backward_cmd_received && enable_backward_cmd) ||
    narrow_mode || reverse_segment;
  if (cfg_->rotation.dual_arc_reverse_enable && dual_arc_reverse_active_ &&
      !other_narrow_sources)
  {
    const double timeout_s = cfg_->rotation.dual_arc_reverse_timeout;
    if (timeout_s > 0.0) {
      const double elapsed_s = (clock_->now() - dual_arc_reverse_enter_time_).seconds();
      const double min_hold_s = std::max(0.0, cfg_->rotation.dual_arc_reverse_min_hold);
      if (elapsed_s >= std::max(timeout_s, min_hold_s)) {
        dual_arc_timeout_probe = true;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, kTebLogThrottleMs,
          "双弧失败: latch 超时 %.2fs，本拍 probe 高权重",
          elapsed_s);
      }
    }
  }
  updateBackwardModePublication(reverse_segment);
  const bool dual_arc_for_weights =
    dual_arc_reverse_active_ && !dual_arc_timeout_probe;
  const bool enable_narrow_teb = (enable_backward_cmd_received && enable_backward_cmd) ?
    true : (narrow_mode || reverse_segment || dual_arc_for_weights);
  updateNarrowPassageTebSettings(enable_narrow_teb);
  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, kTebLogThrottleMs,
    "TEB 倒车模式: enable_backward=%s narrow_latch=%s reverse_seg=%s dual_arc=%s "
    "fail=%zu/%d probe=%s enable=%s forward_drive=%.2f prefer_fwd=%d allow_back=%d",
    (enable_backward_cmd_received && enable_backward_cmd) ? "是" : "否",
    narrow_mode ? "是" : "否",
    reverse_segment ? "是" : "否",
    dual_arc_reverse_active_ ? "是" : "否",
    dual_arc_reverse_fail_count_,
    std::max(1, cfg_->rotation.dual_arc_reverse_fail_num),
    dual_arc_timeout_probe ? "是" : "否",
    enable_narrow_teb ? "是" : "否",
    enable_narrow_teb ? cfg_->optim.weight_kinematics_forward_drive_in_narrow_passages :
      normal_weight_kinematics_forward_drive_,
    cfg_->robot.prefer_forward_only ? 1 : 0,
    cfg_->robot.allow_backward_velocity ? 1 : 0);
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    throw nav2_core::PlannerException(
      std::string("Transformed plan is empty. Cannot determine a local plan.")
    );
  }

  {
    geometry_msgs::msg::TwistStamped wait_cmd;
    if (tryPathWaitHold(robot_pose, robot_vel_, wait_cmd))
    {
      return wait_cmd;
    }
  }

  {
    geometry_msgs::msg::TwistStamped standoff_cmd;
    if (tryGoalStandoffHold(robot_pose, robot_vel_, standoff_cmd))
    {
      return standoff_cmd;
    }
  }

  {
    geometry_msgs::msg::TwistStamped occupied_goal_cmd;
    if (tryOccupiedGoalNearHold(robot_vel_, occupied_goal_cmd))
    {
      return occupied_goal_cmd;
    }
  }

  // Check if in-place rotation should be performed before TEB planning
  const bool use_inplace_rotation =
    !cfg_->robot.front_steering_rear_drive &&
    shouldRotateInPlace(robot_vel_, transformed_plan, robot_pose);
  if (use_inplace_rotation)
  {
    geometry_msgs::msg::TwistStamped rotation_cmd_vel;
    // Calculate angle difference to target heading using forward lookahead distance
    const geometry_msgs::msg::PoseStamped *target_pose = nullptr;
    double accumulated_distance = 0.0;
    
    for (size_t i = 1; i < transformed_plan.size(); ++i)
    {
      double dx = transformed_plan[i].pose.position.x - transformed_plan[i-1].pose.position.x;
      double dy = transformed_plan[i].pose.position.y - transformed_plan[i-1].pose.position.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);
      
      if (accumulated_distance >= cfg_->rotation.forward_lookahead_distance)
      {
        target_pose = &transformed_plan[i];
        break;
      }
    }
    
    // If no pose found within lookahead distance, use the last pose
    if (target_pose == nullptr)
    {
      target_pose = &transformed_plan.back();
    }
    
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    double target_yaw = tf2::getYaw(target_pose->pose.orientation);
    double angular_distance_to_heading = target_yaw - robot_yaw;
    // Normalize to [-PI, PI]
    while (angular_distance_to_heading > M_PI) angular_distance_to_heading -= 2.0 * M_PI;
    while (angular_distance_to_heading < -M_PI) angular_distance_to_heading += 2.0 * M_PI;
    
    if (computeRotateToHeadingCommand(angular_distance_to_heading, robot_pose, robot_vel_, rotation_cmd_vel))
    {
      const double v_xy = std::hypot(robot_vel_.linear.x, robot_vel_.linear.y);
      // RCLCPP_INFO_THROTTLE(
      //   logger_, *clock_, 500,
      //   "原地转: 执行中（跳过 TEB），cmd_w=%.3f rad/s，|dtheta|=%.3f rad，w_odom=%.3f，v_xy=%.3f，wn=%.3f，dt=%.3f",
      //   rotation_cmd_vel.twist.angular.z,
      //   std::abs(angular_distance_to_heading),
      //   velocity.angular.z,
      //   v_xy,
      //   cfg_->rotation.rotate_to_heading_angular_vel,
      //   control_duration_);
         // Publish global_plan_ for visualization/debugging
      if (global_plan_pub_ && !global_plan_.empty())
      {
        nav_msgs::msg::Path global_plan_msg;
        global_plan_msg.header.stamp = clock_->now();
        global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
        global_plan_msg.poses = global_plan_;
        global_plan_pub_->publish(global_plan_msg);
      }
      if (dual_arc_reverse_active_) {
        dual_arc_reverse_forward_count_ = 0;
      }
      dual_arc_reverse_fail_count_ = 0;
      return rotation_cmd_vel;
    }
    // If rotation fails due to collision, continue with TEB planning
    // RCLCPP_INFO_THROTTLE(
    //   logger_, *clock_, 500,
    //   "原地转: computeRotateToHeadingCommand 失败，改用 TEB（|dtheta|=%.3f rad、w_odom=%.3f）",
    //   std::abs(angular_distance_to_heading),
    //   velocity.angular.z);
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
   if (costmap_converter_) {
     updateObstacleContainerWithCostmapConverter();
   } else {
     updateObstacleContainerWithCostmap();
   }
   
   // also consider custom obstacles (must be called after other updates, since the container is not cleared)
   updateObstacleContainerWithCustomObstacles();

   // ---- Near-horizon dynamic safety (cache + threat gate + OmegaHold runtime) ----
   // Must run after obstacles_ are filled and before planner_->plan().
   {
     const rclcpp::Time now = clock_->now();
     dynamic_obstacle_cache_.configure(cfg_->obstacles.dynamic_obstacle_cache_time);
     dynamic_obstacle_cache_.update(obstacles_, now);

     // Footprint in robot frame from costmap (fallback handled inside computeFootprintRoiBox)
     std::vector<geometry_msgs::msg::Point> footprint_rf;
     if (costmap_ros_)
     {
       footprint_rf = costmap_ros_->getRobotFootprint();
     }
     const FootprintRoiBox fp_roi = computeFootprintRoiBox(footprint_rf);

     PoseSE2 robot_pose_se2(robot_pose);
     NearHorizonThreatResult threat = near_horizon_threat_gate_.evaluate(
         robot_pose_se2, cfg_->robot_model, fp_roi, dynamic_obstacle_cache_.get(), *cfg_, now);

     cfg_->near_horizon_runtime.omega_hold_enable = threat.enable_omega_hold;
     cfg_->near_horizon_runtime.omega_ref = robot_vel_.angular.z;

     RCLCPP_DEBUG_THROTTLE(
         logger_, *clock_, kTebLogThrottleMs,
         "[NearHorizon] 近动态障碍物门控 mode=%d as_static_edges=%d cache=%zu raw_threat=%d enable=%d "
         "overlap_excl=%d trig_id=%d reason=%s omega_ref=%.3f delta=%.3f "
         "(roi f/r/l/r=%.2f/%.2f/%.2f/%.2f T_near=%.2f hold=%.2f)",
         cfg_->obstacles.dynamic_safety_predictable_mode ? 1 : 0,
         cfg_->obstacles.dynamic_obstacles_as_static_edges ? 1 : 0,
         dynamic_obstacle_cache_.get().size(),
         threat.raw_threat ? 1 : 0,
         threat.enable_omega_hold ? 1 : 0,
         threat.excluded_overlap ? 1 : 0,
         threat.trigger_obstacle_id,
         threat.reason.c_str(),
         cfg_->near_horizon_runtime.omega_ref,
         cfg_->obstacles.delta_omega_scale * cfg_->robot.max_vel_theta,
         cfg_->obstacles.dyn_gate_roi_front, cfg_->obstacles.dyn_gate_roi_rear,
         cfg_->obstacles.dyn_gate_roi_left, cfg_->obstacles.dyn_gate_roi_right,
         cfg_->obstacles.dyn_gate_T_near, cfg_->obstacles.dyn_gate_hold_time);

     if (cfg_->obstacles.publish_near_horizon_debug && near_horizon_debug_marker_pub_)
     {
       visualization_msgs::msg::MarkerArray ma;
       const std::string frame_id =
           costmap_ros_ ? costmap_ros_->getGlobalFrameID() : cfg_->map_frame;
       buildNearHorizonDebugMarkers(
           ma, frame_id, now, robot_pose_se2, fp_roi, *cfg_, dynamic_obstacle_cache_.get(), threat,
           near_horizon_threat_gate_.lastExtrapolatedSamples(), footprint_rf);
       near_horizon_debug_marker_pub_->publish(ma);
     }
   }
   
     
   // Do not allow config changes during the following optimization step
   std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());

   try
   { 
   if(cfg_->robot.is_footprint_dynamic)
   {
     const std::vector<geometry_msgs::msg::Point> costmap_footprint = costmap_ros_->getRobotFootprint();
     if (costmap_footprint != footprint_spec_) {
       footprint_spec_ = costmap_footprint;
       nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
     }
   }

   cfg_->robot.allow_backward_velocity = true;
   geometry_msgs::msg::Twist start_vel = robot_vel_;

   auto planIsFeasible = [&](const std::vector<geometry_msgs::msg::PoseStamped> & plan_in) -> bool {
     if (!planner_->plan(plan_in, &start_vel, cfg_->goal_tolerance.free_goal_vel)) {
       return false;
     }
     refreshOptimizedFrontSteeringCache();
     if (planner_->hasDiverged()) {
       return false;
     }
     return planner_->isTrajectoryFeasible(
       costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
       robot_circumscribed_radius, cfg_->trajectory.feasibility_check_no_poses,
       cfg_->trajectory.feasibility_check_lookahead_distance);
   };

   const double yaw_start = tf2::getYaw(transformed_plan.front().pose.orientation);
   const double yaw_goal = tf2::getYaw(transformed_plan.back().pose.orientation);
   const double d_short = g2o::normalize_theta(yaw_goal - yaw_start);
   double d_long = (std::abs(d_short) < 1e-3)
     ? -2.0 * M_PI
     : d_short - std::copysign(2.0 * M_PI, d_short);
   // ±π 割线处 d_short 变号会把优弧左右互换；已激活时锁住转向
   if (major_arc_active_ && major_arc_rotdir_ != RotType::none) {
     const double mag = std::abs(d_long);
     d_long = (major_arc_rotdir_ == RotType::left) ? mag : -mag;
   }
   const bool major_stagnation_unlock = checkMajorArcStagnationUnlock(yaw_goal);
   const double heading_exit = std::max(
     cfg_->rotation.major_arc_retry_heading_threshold, 0.35);
   const bool heading_ok =
     std::abs(d_short) >= cfg_->rotation.major_arc_retry_heading_threshold;
   const bool keep_major = major_arc_active_ && std::abs(d_short) >= heading_exit;
   const bool major_retry_ok =
     cfg_->rotation.major_arc_retry_enable && heading_ok && !major_stagnation_unlock;
   const bool try_major_first = major_retry_ok && keep_major;
   if (!keep_major) {
     resetMajorArcLock();
   }

   auto runMajorArcPlan = [&](bool reuse_teb) -> bool {
     major_arc_plan_ = buildMajorArcOrientedPlan(transformed_plan, d_long);
     if (major_arc_plan_.size() < 2) {
       return false;
     }
     major_arc_rotdir_ = (d_long > 0.0) ? RotType::left : RotType::right;
     if (!reuse_teb) {
       planner_->clearPlanner();
     }
     planner_->setPreferredTurningDir(major_arc_rotdir_);
     const int saved_classes = cfg_->hcp.max_number_classes;
     const bool saved_overwrite = cfg_->trajectory.global_plan_overwrite_orientation;
     cfg_->hcp.max_number_classes = 1;
     cfg_->trajectory.global_plan_overwrite_orientation = false;
     const bool ok = planIsFeasible(major_arc_plan_);
     cfg_->hcp.max_number_classes = saved_classes;
     cfg_->trajectory.global_plan_overwrite_orientation = saved_overwrite;
     return ok;
   };

   auto tryBothArcs = [&](bool prefer_major_first, bool reuse_major) -> bool {
     bool ok = false;
     if (!prefer_major_first) {
       ok = planIsFeasible(transformed_plan);
       if (ok) {
         resetMajorArcLock();
         planner_->setPreferredTurningDir(last_preferred_rotdir_);
         return true;
       }
     }
     if (!ok && major_retry_ok) {
       if (prefer_major_first) {
         RCLCPP_INFO_THROTTLE(
           logger_, *clock_, 2000,
           "继续优弧 warm-start d_short=%.3f rad d_long=%.3f rad rotdir=%s",
           d_short, d_long,
           (major_arc_rotdir_ == RotType::left) ? "left" : "right");
       } else {
         RCLCPP_INFO_THROTTLE(
           logger_, *clock_, kTebLogThrottleMs,
           "劣弧 TEB 不可行，改用优弧初始化重优化 d_short=%.3f rad d_long=%.3f rad",
           d_short, d_long);
       }
       ok = runMajorArcPlan(prefer_major_first && reuse_major);
       if (ok) {
         const bool newly_locked = !major_arc_active_;
         major_arc_active_ = true;
         if (newly_locked || !major_arc_progress_valid_) {
           captureMajorArcProgress(remainingYawAlongMajorArc(robot_pose_.theta(), yaw_goal));
         }
       } else {
         resetMajorArcLock();
         planner_->setPreferredTurningDir(RotType::none);
         RCLCPP_WARN_THROTTLE(
           logger_, *clock_, kTebLogThrottleMs,
           "优弧重优化仍不可行 d_short=%.3f rad d_long=%.3f rad",
           d_short, d_long);
       }
     } else if (!ok) {
       RCLCPP_INFO_THROTTLE(
         logger_, *clock_, kTebLogThrottleMs,
         "劣弧不可行但跳过优弧重试 enable=%d |d_short|=%.3f rad thr=%.3f rad heading_ok=%d keep=%d stagnation=%d",
         cfg_->rotation.major_arc_retry_enable ? 1 : 0,
         std::abs(d_short), cfg_->rotation.major_arc_retry_heading_threshold,
         heading_ok ? 1 : 0, keep_major ? 1 : 0, major_stagnation_unlock ? 1 : 0);
     }
     return ok;
   };

   bool feasible = tryBothArcs(try_major_first, true);

   if (feasible) {
     dual_arc_reverse_fail_count_ = 0;
   } else if (cfg_->rotation.dual_arc_reverse_enable) {
     const bool already_latched = dual_arc_reverse_active_ && !dual_arc_timeout_probe;
     bool allow_reverse = already_latched || dual_arc_timeout_probe;
     if (!already_latched && !dual_arc_timeout_probe) {
       ++dual_arc_reverse_fail_count_;
       const int need_fails = std::max(1, cfg_->rotation.dual_arc_reverse_fail_num);
       RCLCPP_INFO_THROTTLE(
         logger_, *clock_, kTebLogThrottleMs,
         "双弧连续失败 %zu/%d，%s降权允许倒车",
         dual_arc_reverse_fail_count_, need_fails,
         dual_arc_reverse_fail_count_ >= static_cast<size_t>(need_fails) ? "达到次数，" : "未达次数，暂不");
       allow_reverse =
         dual_arc_reverse_fail_count_ >= static_cast<size_t>(need_fails);
     }
     if (allow_reverse) {
       if (!already_latched) {
         enterDualArcReverseLatch();
       }
       const bool need_replan = !narrow_teb_settings_applied_ || dual_arc_timeout_probe;
       if (need_replan) {
         RCLCPP_INFO_THROTTLE(
           logger_, *clock_, kTebLogThrottleMs,
           "双弧均不可行，降 forward_drive 后同周期重试");
         updateNarrowPassageTebSettings(true);
         planner_->clearPlanner();
         resetMajorArcLock();
         planner_->setPreferredTurningDir(RotType::none);
         dual_arc_timeout_probe = false;
         feasible = tryBothArcs(false, false);
         if (feasible) {
           dual_arc_reverse_fail_count_ = 0;
         }
       }
     }
   }

   if (!feasible)
   {
     cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
     planner_->clearPlanner();
     ++no_infeasible_plans_;
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     dual_arc_reverse_forward_count_ = 0;
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
     );
   }
 
   // TEB steering edges are soft constraints. A finite optimized trajectory may
   // slightly exceed the angle bound or rotate while below steering_min_speed.
   // Validate the emitted command after projection instead of rejecting that
   // trajectory here; projection enforces the physical limits and rejects a
   // command whose yaw-rate correction exceeds max_projection_omega_error.
   if (front_steer_interface_mode_ == "front_steer_cmd") {
     auto optimal = std::dynamic_pointer_cast<TebOptimalPlanner>(planner_);
     if (!optimal) {
       auto homotopy = std::dynamic_pointer_cast<HomotopyClassPlanner>(planner_);
       if (homotopy) {optimal = homotopy->bestTeb();}
     }
     bool kinematics_ok = optimal && optimal->teb().sizeTimeDiffs() > 0;
     std::string kinematics_detail = !optimal ?
       "原因=未找到优化轨迹（包含多路径规划的最优轨迹）" :
       "原因=优化轨迹没有有效的时间段，时间段数=0";
     if (kinematics_ok) {
       const auto & teb = optimal->teb();
       const int segments = std::min(teb.sizeTimeDiffs(),
         std::max(1, cfg_->trajectory.control_look_ahead_poses));
       bool soft_violation_logged = false;
       for (int i = 0; i < segments && kinematics_ok; ++i) {
         SegmentKinematics segment;
         const bool valid_segment = computeSteeringSegment(teb.Pose(i), teb.Pose(i + 1),
           teb.TimeDiff(i), cfg_->robot.wheelbase, cfg_->robot.steering_min_speed,
           cfg_->robot.steering_omega_epsilon, segment);
         kinematics_ok = valid_segment;
         if (!kinematics_ok) {
           const char * reason = isTimeDiffDegenerate(segment.dt) ?
             "时间间隔非有限或小于最小值" :
             (!(cfg_->robot.wheelbase > 0.0) || !std::isfinite(cfg_->robot.wheelbase)) ?
             "轴距无效" : "轨迹段计算结果含非有限值";
           kinematics_detail = "原因=" + std::string(reason) +
             "，段序号=" + std::to_string(i) +
             "，起点=(" + std::to_string(teb.Pose(i).x()) + "," +
               std::to_string(teb.Pose(i).y()) + "," +
               std::to_string(teb.Pose(i).theta()) + ")" +
             "，终点=(" + std::to_string(teb.Pose(i + 1).x()) + "," +
               std::to_string(teb.Pose(i + 1).y()) + "," +
               std::to_string(teb.Pose(i + 1).theta()) + ")" +
             "，时间间隔=" + std::to_string(segment.dt) + "秒" +
             "，轴距=" + std::to_string(cfg_->robot.wheelbase) + "米" +
             "，纵向速度=" + std::to_string(segment.velocity) + "米/秒" +
             "，角速度=" + std::to_string(segment.omega) + "弧度/秒" +
             "，计算转角=" + std::to_string(segment.steering_angle) + "弧度";
         } else if (!soft_violation_logged &&
                    (segment.inplace_rotation ||
                     std::abs(segment.steering_angle) > cfg_->robot.max_steering_angle + 1e-6)) {
           // These are optimizer soft-constraint residuals, not command faults.
           // Keep the first offending segment per cycle for a useful diagnosis.
           soft_violation_logged = true;
           RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
             "前轮转向轨迹软约束越界（不直接拒绝轨迹，最终指令仍需安全校验）："
             "段序号=%d，原因=%s，纵向速度=%.6f米/秒（低速阈值=%.6f），"
             "角速度=%.6f弧度/秒（静止角速度阈值=%.6f），"
             "纵向位移=%.6f米，航向变化=%.6f弧度，"
             "计算转角=%.6f弧度（%s），转角上限=%.6f弧度，时间间隔=%.6f秒",
             i, segment.inplace_rotation ? "低速时仍有转向" : "计算转角超过上限",
             segment.velocity, cfg_->robot.steering_min_speed,
             segment.omega, cfg_->robot.steering_omega_epsilon,
             segment.longitudinal_displacement, segment.delta_theta,
             segment.steering_angle,
             segment.stopped ? "低速时不反算转角" : "根据角速度和纵向速度反算",
             cfg_->robot.max_steering_angle, segment.dt);
         }
       }
     }
     if (!kinematics_ok) {
       planner_->clearPlanner();
       ++no_infeasible_plans_;
       time_last_infeasible_plan_ = clock_->now();
       throw nav2_core::PlannerException("前轮转向优化轨迹数值无效：" + kinematics_detail);
     }
   }

   // Get the velocity command for this sampling interval
   if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
   {
     planner_->clearPlanner();
     ++no_infeasible_plans_; // increase number of infeasible solutions in a row
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     
     dual_arc_reverse_forward_count_ = 0;
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
     );
   }
   
   if (front_steer_interface_mode_ == "front_steer_cmd" &&
       (!std::isfinite(cmd_vel.twist.linear.x) || !std::isfinite(cmd_vel.twist.linear.y) ||
        !std::isfinite(cmd_vel.twist.angular.z))) {
     planner_->clearPlanner();
     ++no_infeasible_plans_;
     time_last_infeasible_plan_ = clock_->now();
     throw nav2_core::PlannerException("Non-finite front-steer planner command");
   }

   // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
   saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                    cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);

  if (cfg_->robot.front_steering_rear_drive &&
      (cfg_->robot.front_steering_hard_limit ||
       front_steer_interface_mode_ == "front_steer_cmd"))
   {
     applyFrontSteeringCommandLimits(
       cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
   }
   // convert rot-vel to steering angle if desired (carlike robot).
   // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
   // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
   if (!cfg_->robot.front_steering_rear_drive && cfg_->robot.cmd_angle_instead_rotvel)
   {
     cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
     if (!std::isfinite(cmd_vel.twist.angular.z))
     {
       cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
       last_cmd_ = cmd_vel.twist;
       planner_->clearPlanner();
 
       ++no_infeasible_plans_; // increase number of infeasible solutions in a row
       time_last_infeasible_plan_ = clock_->now();
       dual_arc_reverse_forward_count_ = 0;
       throw nav2_core::PlannerException(
         std::string("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...")
       );
     }
   }
   

   if (dual_arc_reverse_active_) {
     if (dual_arc_timeout_probe) {
       exitDualArcReverseLatch();
       updateNarrowPassageTebSettings(other_narrow_sources);
     } else {
       const double vx_thr = cfg_->rotation.dual_arc_reverse_forward_vx;
       const int need_frames = std::max(1, cfg_->rotation.dual_arc_reverse_forward_num);
       const double min_hold_s = std::max(0.0, cfg_->rotation.dual_arc_reverse_min_hold);
       const double elapsed_s = (clock_->now() - dual_arc_reverse_enter_time_).seconds();
       if (cmd_vel.twist.linear.x >= vx_thr) {
         ++dual_arc_reverse_forward_count_;
       } else {
         dual_arc_reverse_forward_count_ = 0;
       }
       if (elapsed_s >= min_hold_s &&
           dual_arc_reverse_forward_count_ >= static_cast<size_t>(need_frames)) {
         RCLCPP_INFO_THROTTLE(
           logger_, *clock_, kTebLogThrottleMs,
           "双弧失败: 连续前进 %zu 帧 vx=%.3f 且保持 %.2fs，切回高权重",
           dual_arc_reverse_forward_count_, cmd_vel.twist.linear.x, elapsed_s);
         exitDualArcReverseLatch();
         updateNarrowPassageTebSettings(other_narrow_sources);
       }
     }
   }
   
   // store last command (for recovery analysis etc.)
   last_cmd_ = cmd_vel.twist;
   
   // Now visualize everything    
   planner_->visualize();
   visualization_->publishObstacles(obstacles_);
   visualization_->publishViaPoints(via_points_);
   visualization_->publishGlobalPlan(global_plan_);
   }
   catch (const TebAssertionFailureException& ex)
   {
     planner_->clearPlanner();
     ++no_infeasible_plans_;
     time_last_infeasible_plan_ = clock_->now();
     last_cmd_ = cmd_vel.twist;
     RCLCPP_ERROR_THROTTLE(
       logger_, *(clock_), 1000,
       "TEB 数值断言失败: %s，正在重置规划器",
       ex.what());
     throw nav2_core::PlannerException(
       std::string("TebLocalPlannerROS: TEB numeric failure. Resetting planner..."));
   }

   if (global_plan_pub_ && !global_plan_.empty())
   {
     nav_msgs::msg::Path global_plan_msg;
     global_plan_msg.header.stamp = clock_->now();
     global_plan_msg.header.frame_id = global_plan_.front().header.frame_id;
     global_plan_msg.poses = global_plan_;
     global_plan_pub_->publish(global_plan_msg);
   }

  if (front_steer_interface_mode_ == "front_steer_cmd")
  {
    double steering_angle = 0.0;
    {
      std::lock_guard<std::mutex> lock(steering_topic_mutex_);
      steering_angle = front_steering_cmd_valid_ ? front_steering_delta_ : 0.0;
    }
    if (cmd_vel.twist.linear.x == 0.0 && !front_steer_aligning_) {steering_angle = 0.0;}
    const bool accepted = publishFrontSteerCommand(
      cmd_vel.twist.linear.x, steering_angle, cmd_vel.twist.angular.z);
    front_steer_command_sent = true;
    if (!accepted) {
      planner_->clearPlanner();
      ++no_infeasible_plans_;
      time_last_infeasible_plan_ = clock_->now();
      throw nav2_core::PlannerException("Front-steer final command validation failed");
    }
    cmd_vel.twist = last_cmd_;
  }
   
   // Reset failure history only after the final interface check succeeds.
   no_infeasible_plans_ = 0;
   return cmd_vel;
 }
 

 void TebLocalPlannerROS::applyFrontSteeringCommandLimits(double& vx, double& vy, double& omega)
 {
   FrontSteeringCommandState previous;
   {
     std::lock_guard<std::mutex> lock(steering_topic_mutex_);
     previous.valid = front_steering_cmd_valid_;
     previous.steering_angle = front_steering_delta_;
     previous.steering_rate = front_steering_delta_rate_;
   }
   const bool direct = front_steer_interface_mode_ == "front_steer_cmd";
   if (direct) {
     front_steer_control_dt_ = front_steer_publish_valid_ ?
       std::chrono::duration<double>(std::chrono::steady_clock::now() -
         front_steer_last_publish_time_).count() : control_duration_;
     // An old, explicit zero command plus fresh standstill feedback is idle,
     // not an active motion command timing out. Start a new bounded interval.
     if (front_steer_publish_valid_ && front_steer_control_dt_ > front_steer_state_timeout_ &&
         front_steer_published_angle_ == 0.0 && front_steer_published_rate_ == 0.0 &&
         last_cmd_.linear.x == 0.0 && last_cmd_.angular.z == 0.0) {
       geometry_msgs::msg::Twist idle_feedback;
       if (getFreshFrontSteerVelocity(idle_feedback) &&
           std::abs(idle_feedback.linear.x) <= cfg_->robot.steering_min_speed &&
           std::abs(idle_feedback.angular.z) <= 1e-6) {
         front_steer_publish_valid_ = false;
         front_steer_control_dt_ = control_duration_;
       }
     }
     previous.valid = true;
     previous.steering_angle = front_steer_published_angle_;
     previous.steering_rate = front_steer_published_rate_;
     if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
         !std::isfinite(front_steer_control_dt_) || front_steer_control_dt_ <= 1e-4 ||
         front_steer_control_dt_ > front_steer_state_timeout_) {
       if (planner_) {planner_->clearPlanner();}
       ++no_infeasible_plans_;
       time_last_infeasible_plan_ = clock_->now();
       throw nav2_core::PlannerException("Invalid front-steer input or control interval: dt=" +
         std::to_string(front_steer_control_dt_) + " timeout=" + std::to_string(front_steer_state_timeout_) +
         " v=" + std::to_string(vx) + " vy=" + std::to_string(vy) + " omega=" + std::to_string(omega));
     }
   }
   const double dt_ctrl = direct ? front_steer_control_dt_ :
     (control_duration_ > 1e-4 ? control_duration_ : cfg_->trajectory.dt_ref);
   FrontSteeringCommand command;
   front_steer_aligning_ = false;
   if (direct) {
     double measured_angle;
     double measured_rate;
     {
       std::lock_guard<std::mutex> lock(front_steer_stu_mutex_);
       measured_angle = front_steer_stu_.front_steer_angle;
       measured_rate = static_cast<double>(front_steer_stu_.front_wheel_speed) *
         (2.0 * std::acos(-1.0) / 60.0);
     }
     const char * error = projectFrontSteerDriveCommand(vx, vy, omega, dt_ctrl,
       cfg_->robot, previous, measured_angle, front_steer_projection_tolerance_,
       command, front_steer_aligning_, front_steer_lag_speed_scaling_, &front_steer_lag_speed_limit_);
     if (error) {
       const std::string reason = std::string(error) + ": v=" + std::to_string(vx) +
         " omega_before=" + std::to_string(omega) +
         " limit=" + std::to_string(front_steer_projection_tolerance_) +
         " previous_delta=" + std::to_string(previous.steering_angle) +
         " previous_rate=" + std::to_string(previous.steering_rate) +
         " measured_delta=" + std::to_string(measured_angle) +
         " dt=" + std::to_string(dt_ctrl);
       if (front_steer_debug_logging_.load()) {
         RCLCPP_ERROR(logger_, "%s", reason.c_str());
       } else {
         RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "%s", reason.c_str());
       }
       if (planner_) {planner_->clearPlanner();}
       ++no_infeasible_plans_;
       time_last_infeasible_plan_ = clock_->now();
       throw nav2_core::PlannerException(reason);
     }
     front_steer_control_dt_prepared_ = true;
     if (front_steer_lag_speed_scaling_ && !front_steer_aligning_ &&
         std::isfinite(front_steer_lag_speed_limit_) && std::abs(command.vx) < std::abs(vx)) {
       RCLCPP_INFO_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
         "Front-steer steering lag speed cap: requested_v=%.3f cap=%.3f measured_delta=%.3f command_delta=%.3f",
         vx, front_steer_lag_speed_limit_, measured_angle, command.steering_angle);
     }
     if (front_steer_aligning_) {
       RCLCPP_INFO_THROTTLE(logger_, *clock_, kTebLogThrottleMs,
         "Front-steer aligning at zero speed: requested v=%.3f omega=%.3f measured_delta=%.3f command_delta=%.3f",
         vx, omega, measured_angle, command.steering_angle);
     }
     if (front_steer_debug_logging_.load()) {
       const double remaining_angle = measured_rate >= 0.0 ?
         cfg_->robot.max_steering_angle - measured_angle :
         cfg_->robot.max_steering_angle + measured_angle;
       const double braking_angle = measured_rate * measured_rate /
         (2.0 * cfg_->robot.max_steering_acceleration);
       RCLCPP_INFO(logger_,
         "前轮转向投影：请求v=%.6f米/秒、偏航角速度=%.6f弧度/秒；"
         "底盘转角=%.6f弧度、实测转向角速度=%.6f弧度/秒；"
         "上次指令转角=%.6f弧度、转向角速度=%.6f弧度/秒；"
         "输出v=%.6f米/秒、转角=%.6f弧度、转向角速度=%.6f弧度/秒；"
         "距转角极限=%.6f弧度、按最大减速度估算制动角度=%.6f弧度、余量=%.6f弧度；"
         "dt=%.6f秒、对齐等待=%d",
         vx, omega, measured_angle, measured_rate,
         previous.steering_angle, previous.steering_rate,
         command.vx, command.steering_angle, command.steering_rate,
         remaining_angle, braking_angle, remaining_angle - braking_angle,
         dt_ctrl, front_steer_aligning_ ? 1 : 0);
     }
   } else {
     command = projectFrontSteeringCommand(vx, vy, omega, dt_ctrl, cfg_->robot, previous);
   }
   vx = command.vx;
   vy = command.vy;
   omega = command.omega;
   if (direct && std::abs(vx) <= cfg_->robot.steering_min_speed) {
     vx = vy = omega = 0.0;
   }
   {
     const double steering_acceleration =
       (previous.valid && dt_ctrl > 1e-4) ?
       (command.steering_rate - previous.steering_rate) / dt_ctrl : 0.0;
     std::lock_guard<std::mutex> lock(steering_topic_mutex_);
     front_steering_delta_acceleration_ = steering_acceleration;
     front_steering_delta_ = command.steering_angle;
     front_steering_delta_rate_ = command.steering_rate;
     front_steering_cmd_valid_ = true;
   }
   front_steering_rear_left_ = command.rear_left;
   front_steering_rear_right_ = command.rear_right;
   RCLCPP_DEBUG(
     logger_, "前轮转向指令 v=%.3f omega=%.3f delta=%.3f rate=%.3f acc=%.3f vL=%.3f vR=%.3f",
     vx, omega, front_steering_delta_, command.steering_rate, front_steering_delta_acceleration_,
     front_steering_rear_left_, front_steering_rear_right_);
 }

 void TebLocalPlannerROS::refreshOptimizedFrontSteeringCache()
 {
   if (!planner_ || !cfg_ || !cfg_->robot.front_steering_rear_drive)
   {
     return;
   }
   double steering_rate = 0.0;
   double steering_acceleration = 0.0;
   if (!planner_->getOptimizedFrontSteeringDynamics(steering_rate, steering_acceleration))
   {
     return;
   }
   std::lock_guard<std::mutex> lock(steering_topic_mutex_);
   optimized_steering_rate_ = steering_rate;
   optimized_steering_acceleration_ = steering_acceleration;
   optimized_steering_valid_ = true;
   if (front_steer_debug_logging_.load()) {
     RCLCPP_INFO(logger_,
       "前轮转向优化结果：预测转向角速度=%.6f弧度/秒，预测转向角加速度=%.6f弧度/秒²",
       steering_rate, steering_acceleration);
   }
 }

 void TebLocalPlannerROS::publishFrontSteeringTopics()
 {
   if (!cfg_ || !cfg_->robot.front_steering_rear_drive)
   {
     return;
   }

   double optimized_rate = 0.0;
   double optimized_acceleration = 0.0;
   bool publish_optimized = false;
   double limited_rate = 0.0;
   double limited_acceleration = 0.0;
   bool publish_limited = false;
   {
     std::lock_guard<std::mutex> lock(steering_topic_mutex_);
     publish_optimized = optimized_steering_valid_;
     optimized_rate = optimized_steering_rate_;
     optimized_acceleration = optimized_steering_acceleration_;
     publish_limited = cfg_->robot.front_steering_hard_limit && front_steering_cmd_valid_;
     limited_rate = front_steering_delta_rate_;
     limited_acceleration = front_steering_delta_acceleration_;
   }

   if (publish_optimized)
   {
     if (optimized_steering_angular_velocity_pub_)
     {
       std_msgs::msg::Float64 rate_message;
       rate_message.data = optimized_rate;
       optimized_steering_angular_velocity_pub_->publish(rate_message);
     }
     if (optimized_steering_angular_acceleration_pub_)
     {
       std_msgs::msg::Float64 acceleration_message;
       acceleration_message.data = optimized_acceleration;
       optimized_steering_angular_acceleration_pub_->publish(acceleration_message);
     }
   }
   if (!publish_limited)
   {
     return;
   }
   if (steering_angular_velocity_pub_)
   {
     std_msgs::msg::Float64 rate_message;
     rate_message.data = limited_rate;
     steering_angular_velocity_pub_->publish(rate_message);
   }
   if (steering_angular_acceleration_pub_)
   {
     std_msgs::msg::Float64 acceleration_message;
     acceleration_message.data = limited_acceleration;
     steering_angular_acceleration_pub_->publish(acceleration_message);
   }
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
                 "不要把 max_vel_x_backwards 设为 <=0；禁止倒车请增大 weight_kinematics_forward_drive");
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
                   "The inscribed radius of the footprint specified for TEB optimization (%.2f) + min_obstacle_dist (%.2f) are smaller "
                   "than the inscribed radius of the robot's footprint in the costmap parameters (%.2f, including 'footprint_padding'). "
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
         const double min_turning_radius = effectiveMinTurningRadius(cfg_->robot);
         if (min_turning_radius != 0 && max_vel_current > 0)
             max_vel_theta = std::max( max_vel_current/std::abs(min_turning_radius),  cfg_->robot.max_vel_theta );
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
                 RCLCPP_INFO(logger_, "检测到可能振荡（机器人或局部路径），启动恢复：优化时偏好当前转向");
             }
             time_last_oscillation_ = clock_->now();
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
         }
         else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
         {
             last_preferred_rotdir_ = RotType::none;
             planner_->setPreferredTurningDir(last_preferred_rotdir_);
             RCLCPP_INFO(logger_, "振荡恢复已关闭或超时");
         }
     }

     if (major_arc_active_ && major_arc_rotdir_ != RotType::none && planner_) {
       planner_->setPreferredTurningDir(major_arc_rotdir_);
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

void TebLocalPlannerROS::speedLimitCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> l(speed_limit_mutex_);
  speed_limit_linear_x_ = msg->data;
  has_speed_limit_ = true;
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

   // Reset static_layer state to default (enabled)
   current_static_layer_state_.store(true);
   desired_static_layer_state_.store(true);
   current_global_footprint_state_.store(false);
   desired_global_footprint_state_.store(false);
   current_local_footprint_state_.store(false);
   desired_local_footprint_state_.store(false);
   clearPendingNormalFootprintRestore();

   return;
 }
 void TebLocalPlannerROS::deactivate() {
   publishFrontSteerCommand(0.0, 0.0);
   visualization_->on_deactivate();
   restoreNarrowPassageTebSettings();
   resetDualArcReverseLatch();
   resetBackwardModePublicationState(true);

   return;
 }
 void TebLocalPlannerROS::cleanup() {
   publishFrontSteerCommand(0.0, 0.0);
   shutting_down_.store(true, std::memory_order_release);
   joinBackgroundTasks(std::chrono::milliseconds(1500));
   // Destroy LinePathCompare here (joins its TF thread) before plugin map clear.
   wall_line_ptr_.reset();
   curb_line_subscriber_.reset();

   if (visualization_) {
     try {
       visualization_->on_cleanup();
     } catch (const std::exception & ex) {
       RCLCPP_WARN(logger_, "可视化 on_cleanup 失败: %s", ex.what());
     }
   }
   stopCostmapConverterWorker(std::chrono::milliseconds(800));

   static_layer_client_.reset();
   local_footprint_client_.reset();
   global_footprint_client_.reset();
   backward_mode_pub_.reset();
   front_steer_params_handler_.reset();
   front_steer_cmd_pub_.reset();
   front_steer_odom_sub_.reset();
   front_steer_sim_state_pub_.reset();
   front_steer_stu_sub_.reset();
   front_steer_cmd_linear_debug_pub_.reset();
   front_steer_cmd_angle_debug_pub_.reset();
   front_steer_cmd_twist_debug_pub_.reset();
   front_steer_stu_linear_debug_pub_.reset();
   front_steer_stu_angle_debug_pub_.reset();
   front_steer_stu_wheel_debug_pub_.reset();
   front_steer_measured_rate_pub_.reset();
   {
     std::lock_guard<std::mutex> lock(front_steer_stu_mutex_);
     front_steer_stu_valid_ = false;
   }
   safe_terminal_marker_pub_.reset();
   occupied_goal_stop_marker_pub_.reset();
   resetBackwardModePublicationState(false);
   initialized_ = false;
 }

}  // namespace teb_local_planner

// register this planner as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
