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

 #ifndef TEB_LOCAL_PLANNER_ROS_H_
 #define TEB_LOCAL_PLANNER_ROS_H_
 
 #include <pluginlib/class_loader.hpp>
 
 #include <rclcpp/rclcpp.hpp>
 
 // Navigation2 local planner base class and utilities
 #include <nav2_core/controller.hpp>
 
 // timed-elastic-band related classes
 #include "teb_local_planner/optimal_planner.h"
 #include "teb_local_planner/homotopy_class_planner.h"
 #include "teb_local_planner/visualization.h"
 #include "teb_local_planner/recovery_behaviors.h"
 
 // message types
 #include <nav_msgs/msg/path.hpp>
 #include <nav_msgs/msg/odometry.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <sensor_msgs/msg/point_cloud2.hpp>
 #include <nav_msgs/msg/occupancy_grid.hpp>
 #include <visualization_msgs/msg/marker_array.hpp>
 #include <visualization_msgs/msg/marker.hpp>
 #include <costmap_converter_msgs/msg/obstacle_msg.hpp>
 
 // transforms
 #include <tf2_ros/transform_listener.h>
 #include <tf2/transform_datatypes.h>
 
 // costmap
 #include <costmap_converter/costmap_converter_interface.h>
 #include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
 #include <nav2_costmap_2d/footprint_collision_checker.hpp>
 
 #include <nav2_util/lifecycle_node.hpp>
 #include <nav2_costmap_2d/costmap_2d_ros.hpp>
 #include <nav_2d_utils/parameters.hpp>
 #include "rcl_interfaces/msg/set_parameters_result.hpp"
 // dynamic reconfigure
 //#include "teb_local_planner/TebLocalPlannerReconfigureConfig.h>
 //#include <dynamic_reconfigure/server.h>
 #include "std_msgs/msg/bool.hpp"
 #include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "laserline/line_path_compare.hpp"
#include "capella_ros_msg/msg/lane_center_paths.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <limits>
#include "capella_ros_msg/msg/multi_curve.hpp"
 namespace teb_local_planner
 {
 using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
 using CostmapROSPtr = std::shared_ptr<nav2_costmap_2d::Costmap2DROS>;

 /**
  * @brief Structure to represent a protruding obstacle that caused exit from edge-following mode
  */
 struct ProtrudingObstacle
 {
   Eigen::Vector2d position;      //!< Position of the protruding obstacle (global frame)
   double influence_radius;        //!< Influence radius = bounding circle radius of the obstacle
   rclcpp::Time detection_time;    //!< Time when the obstacle was first detected
 };
 
 /**
   * @class TebLocalPlannerROS
   * @brief Implements the actual abstract navigation stack routines of the teb_local_planner plugin
   * @todo Escape behavior, more efficient obstacle handling
   */
 class TebLocalPlannerROS : public nav2_core::Controller
 {
 
 public:
   /**
     * @brief Constructor of the teb plugin
     */
   TebLocalPlannerROS();
 
   /**
     * @brief  Destructor of the plugin
     */
   ~TebLocalPlannerROS();
   
   /**
    * @brief Configure the teb plugin
    * 
    * @param node The node of the instance
    * @param tf Pointer to a transform listener
    * @param costmap_ros Cost map representing occupied and free space
    */
   void configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name,
     std::shared_ptr<tf2_ros::Buffer> tf,
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
   void activate() override;
   void deactivate() override;
   void cleanup() override;
 
   /**
     * @brief Initializes the teb plugin
     */
   void initialize(nav2_util::LifecycleNode::SharedPtr node);
 
   /**
     * @brief Set the plan that the teb local planner is following
     * @param orig_global_plan The plan to pass to the local planner
     * @return
     */
   void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;
 
   /**
     * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * @param pose is the current position
     * @param velocity is the current velocity
     * @return velocity commands to send to the base
     */
   geometry_msgs::msg::TwistStamped computeVelocityCommands(
     const geometry_msgs::msg::PoseStamped &pose,
     const geometry_msgs::msg::Twist &velocity,
       nav2_core::GoalChecker * goal_checker);
   
     
   /** @name Public utility functions/methods */
   //@{
   
     /**
     * @brief  Transform a tf::Pose type into a Eigen::Vector2d containing the translational and angular velocities.
     * 
     * Translational velocities (x- and y-coordinates) are combined into a single translational velocity (first component).
     * @param tf_vel tf::Pose message containing a 1D or 2D translational velocity (x,y) and an angular velocity (yaw-angle)
     * @return Translational and angular velocity combined into an Eigen::Vector2d
     */
 //  static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);
 
   /**
    * @brief Get the current robot footprint/contour model
    * @param nh const reference to the local rclcpp::Node::SharedPtr
    * @return Robot footprint model used for optimization
    */
   RobotFootprintModelPtr getRobotFootprintFromParamServer(nav2_util::LifecycleNode::SharedPtr node);
   
   /** 
    * @brief Set the footprint from the given XmlRpcValue.
    * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
    * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::msg::Point
    * @param footprint_xmlrpc should be an array of arrays, where the top-level array should have 3 or more elements, and the
    * sub-arrays should all have exactly 2 elements (x and y coordinates).
    * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
    * It is used only for reporting errors. 
    * @return container of vertices describing the polygon
    */
 // Using ROS2 parameter server
 //  static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);
   
   /** 
    * @brief Get a number from the given XmlRpcValue.
    * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
    * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::msg::Point
    * @param value double value type
    * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
    * It is used only for reporting errors. 
    * @returns double value
    */
 // Using ROS2 parameter server
 //  static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
   
   //@}
 
 protected:
 
   /**
     * @brief Update internal obstacle vector based on occupied costmap cells
     * @remarks All occupied cells will be added as point obstacles.
     * @remarks All previous obstacles are cleared.
     * @sa updateObstacleContainerWithCostmapConverter
     * @todo Include temporal coherence among obstacle msgs (id vector)
     * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
     */
   void updateObstacleContainerWithCostmap();
   
   /**
    * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
    * @remarks Requires a loaded costmap_converter plugin.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmap
    */
   void updateObstacleContainerWithCostmapConverter();
   
   /**
    * @brief Update internal obstacle vector based on custom messages received via subscriber
    * @remarks All previous obstacles are NOT cleared. Call this method after other update methods.
    * @sa updateObstacleContainerWithCostmap, updateObstacleContainerWithCostmapConverter
    */
   void updateObstacleContainerWithCustomObstacles();
 
 
   /**
    * @brief Update internal via-point container based on the current reference plan
    * @remarks All previous via-points will be cleared.
    * @param transformed_plan (local) portion of the global plan (which is already transformed to the planning frame)
    * @param min_separation minimum separation between two consecutive via-points
    */
   void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation);
 
    /**
    * @brief Update internal wall_line_points container based on the current reference plan
    * @remarks All previous wall_line_points will be cleared.
    * @param wall_line contains direction of wall line
    */
   void updateWallLineVec(
     const std::vector<nav_msgs::msg::Path>& wall_line, 
     nav_msgs::msg::Path& input_path,
     const double parallel_tolerance,
     const double distance_tolerance,
     const geometry_msgs::msg::PoseStamped& robot_pose);
     /**
    * @brief Update internal curb_line_points container based on the current reference plan
    * @remarks All previous curb_line_points will be cleared.
    * @param wall_line contains direction of wall line
    */
   void updateCurbLineVec(
     const nav_msgs::msg::Path curb_line, 
     nav_msgs::msg::Path& input_path,
     const double parallel_tolerance,
     const double distance_tolerance,
     const geometry_msgs::msg::PoseStamped& robot_pose);

  void updateMultiCurveVec(
    const capella_ros_msg::msg::MultiCurve multi_curve,
    nav_msgs::msg::Path& input_path,
    const double parallel_tolerance,
    const double distance_tolerance,
    const geometry_msgs::msg::PoseStamped& robot_pose);

  void edgeReferencePathsCallback(const capella_ros_msg::msg::LaneCenterPaths::ConstSharedPtr msg);
  void updateReferenceLineVec(
    const std::vector<nav_msgs::msg::Path>& reference_path_list,
    nav_msgs::msg::Path& input_path,
    const double parallel_tolerance_degrees,
    const double distance_tolerance_meters,
    const geometry_msgs::msg::PoseStamped& robot_pose);
  void runEdgeFollowingPathUpdate(
    std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    const geometry_msgs::msg::PoseStamped& robot_pose);
  bool edgeFollowingEntryGuards(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const nav_msgs::msg::Path& input_path);
  bool trySelectWallSegmentFromCandidates(
    const std::vector<nav_msgs::msg::Path>& wall_candidates,
    const nav_msgs::msg::Path& input_path,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double parallel_tolerance_degrees,
    double distance_tolerance_meters,
    Eigen::Vector2d& selected_edge_segment_start,
    Eigen::Vector2d& selected_edge_segment_end,
    double& minimum_average_distance_to_plan,
    double& robot_perpendicular_distance_to_edge_line);
  bool trySelectSegmentFromTwoPointPath(
    const nav_msgs::msg::Path& two_point_line_path,
    const nav_msgs::msg::Path& input_path,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double parallel_tolerance_degrees,
    double distance_tolerance_meters,
    Eigen::Vector2d& selected_edge_segment_start,
    Eigen::Vector2d& selected_edge_segment_end,
    double& minimum_average_distance_to_plan,
    double& robot_perpendicular_distance_to_edge_line);
  bool trySelectBestReferencePathFromList(
    const std::vector<nav_msgs::msg::Path>& reference_path_candidates,
    const nav_msgs::msg::Path& input_path,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double parallel_tolerance_degrees,
    double distance_tolerance_meters,
    Eigen::Vector2d& selected_edge_segment_start,
    Eigen::Vector2d& selected_edge_segment_end,
    double& minimum_average_distance_to_plan,
    double& robot_perpendicular_distance_to_edge_line);
  void applyWallLineSegmentAndVisual(
    const Eigen::Vector2d& wall_line_segment_start,
    const Eigen::Vector2d& wall_line_segment_end,
    const nav_msgs::msg::Path& input_path,
    double minimum_average_distance_to_plan,
    double robot_perpendicular_distance_to_edge_line);
  void mergeFusionPrimaryWithReference(
    const Eigen::Vector2d& fusion_primary_segment_start,
    const Eigen::Vector2d& fusion_primary_segment_end,
    bool fusion_primary_segment_valid,
    double fusion_primary_minimum_average_distance_to_plan,
    double fusion_primary_robot_perpendicular_distance_to_edge,
    const Eigen::Vector2d& reference_segment_start,
    const Eigen::Vector2d& reference_segment_end,
    bool reference_segment_valid,
    double reference_minimum_average_distance_to_plan,
    double reference_robot_perpendicular_distance_to_edge,
    const nav_msgs::msg::Path& input_path);

  /**
   * @brief Callback for surrounding vehicle poses
   * @param msg PoseArray of vehicles around the robot (positions already in map frame)
   */
  void vehiclePosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

  void vehicleScanCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief 维护车身激光 + converter 多边形滚动栅格（map 对齐）；OccupancyGrid 在 updateObstacleContainerWithCostmapConverter 末尾发布
   * @param obstacles 可为空指针：仅做点云、窗口重映射与衰减，不融合 converter 多边形
   */
  void updateVehicleScanOccupancyGrid(
    const costmap_converter::ObstacleArrayConstPtr& obstacles,
    const std::string& obstacles_frame_id);

  void appendVehicleScanInflatedHullObstacles();

  void publishVehicleScanOccupancyGrid();
   
   
   /**
     * @brief Callback for the dynamic_reconfigure node.
     * 
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
   // TODO : dynamic reconfigure is not supported on ROS2
 //  void reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level);
   
   
    /**
     * @brief Callback for custom obstacles that are not obtained from the costmap 
     * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
     */
   void customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg);
   
    /**
     * @brief Callback for custom via-points
     * @param via_points_msg pointer to the message containing a list of via-points
     */
   void customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg);
 
    /**
     * @brief Prune global plan such that already passed poses are cut off
     * 
     * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
     * If no valid transformation can be found, the method returns \c false.
     * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
     * If no pose within the specified treshold \c dist_behind_robot can be found,
     * nothing will be pruned and the method returns \c false.
     * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
     * @param global_pose The global pose of the robot
     * @param[in,out] global_plan The plan to be transformed
     * @param dist_behind_robot Distance behind the robot that should be kept [meters]
     * @param max_prune_dist Max cumulative Euclidean length along the plan (from first pose) when searching for the prune pose [m]; <=0 = no limit
     * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
     */
   bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose,
                        std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot=1, double max_prune_dist=8.0);
   
   /**
     * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
     * 
     * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h 
     * such that the index of the current goal pose is returned as well as 
     * the transformation between the global plan and the planning frame.
     * @param global_plan The plan to be transformed
     * @param global_pose The global pose of the robot
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
     * @param[out] transformed_plan Populated with the transformed plan
     * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @return \c true if the global plan is transformed, \c false otherwise
     */
   bool transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                            const geometry_msgs::msg::PoseStamped& global_pose,  const nav2_costmap_2d::Costmap2D& costmap,
                            const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
                            int* current_goal_idx = NULL, geometry_msgs::msg::TransformStamped* tf_plan_to_global = NULL) const;
     
   /**
     * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
     * 
     * If the current (local) goal point is not the final one (global)
     * substitute the goal orientation by the angle of the direction vector between 
     * the local goal and the subsequent pose of the global plan. 
     * This is often helpful, if the global planner does not consider orientations. \n
     * A moving average filter is utilized to smooth the orientation.
     * @param global_plan The global plan
     * @param local_goal Current local goal
     * @param current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @param moving_average_length number of future poses of the global plan to be taken into account
     * @return orientation (yaw-angle) estimate
     */
   double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                                       int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;
         
         
   /**
    * @brief Saturate the translational and angular velocity to given limits.
    * 
    * The limit of the translational velocity for backwards driving can be changed independently.
    * Do not choose max_vel_x_backwards <= 0. If no backward driving is desired, change the optimization weight for
    * penalizing backwards driving instead.
    * @param[in,out] vx The translational velocity that should be saturated.
    * @param[in,out] vy Strafing velocity which can be nonzero for holonomic robots
    * @param[in,out] omega The angular velocity that should be saturated.
    * @param max_vel_x Maximum translational velocity for forward driving
    * @param max_vel_y Maximum strafing velocity (for holonomic robots)
    * @param max_vel_theta Maximum (absolute) angular velocity
    * @param max_vel_x_backwards Maximum translational velocity for backwards driving
    */
   void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                         double max_vel_theta, double max_vel_x_backwards) const;
 
   
   /**
    * @brief Convert translational and rotational velocities to a steering angle of a carlike robot
    * 
    * The conversion is based on the following equations:
    * - The turning radius is defined by \f$ R = v/omega \f$
    * - For a car like robot withe a distance L between both axles, the relation is: \f$ tan(\phi) = L/R \f$
    * - phi denotes the steering angle.
    * @remarks You might provide distances instead of velocities, since the temporal information is not required.
    * @param v translational velocity [m/s]
    * @param omega rotational velocity [rad/s]
    * @param wheelbase distance between both axles (drive shaft and steering axle), the value might be negative for back_wheeled robots
    * @param min_turning_radius Specify a lower bound on the turning radius
    * @return Resulting steering angle in [rad] inbetween [-pi/2, pi/2]
    */
   double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;
   
   /**
    * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
    * 
    * This method prints warnings if validation fails.
    * @remarks Currently, we only validate the inscribed radius of the footprints
    * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
    * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
    * @param min_obst_dist desired distance to obstacles
    */
   void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);
   
   
   void configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,  int& goal_idx);
   
   /**
    * @brief Limits the maximum linear speed of the robot.
    * @param speed_limit expressed in absolute value (in m/s)
    * or in percentage from maximum robot speed.
    * @param percentage Setting speed limit in percentage if true
    * or in absolute values in false case.
    */
   void setSpeedLimit(const double & speed_limit,  const bool & percentage);
 
   std::vector<Eigen::Vector2d> get_line_vec();
 
   /**
    * @brief Switch between normal mode and edge-following mode parameters
    * @param enable_edge_mode If true, switch to edge-following mode; otherwise, switch to normal mode
    */
   void switchParameterMode(bool enable_edge_mode);

   /**
    * @brief Set static_layer.enabled parameter via service call
    * @param enabled The desired state of static_layer.enabled
    * @param is_delayed True if called from delayed timer callback, false for immediate call
    */
   void setStaticLayerEnabled(bool enabled, bool is_delayed = false);

   /**
    * @brief Set footprint parameter for local costmap via service call
    * @param enable If true, set edge footprint; if false, restore normal footprint
    * @param is_delayed True if called from delayed timer callback, false for immediate call
    */
   void setLocalFootprintEnabled(bool enable, bool is_delayed = false);

   /**
    * @brief Set footprint parameter for global costmap via service call
    * @param enable If true, set edge footprint; if false, restore normal footprint
    * @param is_delayed True if called from delayed timer callback, false for immediate call
    */
   void setGlobalFootprintEnabled(bool enable, bool is_delayed = false);

 private:
   // Definition of member variables
   rclcpp_lifecycle::LifecycleNode::WeakPtr nh_;
   rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
   rclcpp::Clock::SharedPtr clock_;
   rclcpp::Node::SharedPtr intra_proc_node_;
   // external objects (store weak pointers)
   CostmapROSPtr costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
   nav2_costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
   TFBufferPtr tf_; //!< pointer to Transform Listener
   TebConfig::UniquePtr cfg_; //!< Config class that stores and manages all related parameters
     
   // internal objects (memory management owned)
   PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
   ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
   ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
   std::vector<Eigen::Vector2d> wall_line_points_; //!< Container of wall_line_points_ that should be considered during local trajectory optimization
   capella_ros_msg::msg::MultiCurve edge_multi_curve_;
   TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
   std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model_;
   FailureDetector failure_detector_; //!< Detect if the robot got stucked
   
   std::vector<geometry_msgs::msg::PoseStamped> global_plan_, origin_plan_; //!< Store the current global plan
   
   pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
   std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  
 
   //std::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
   rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_; //!< Subscriber for custom obstacles received via a ObstacleMsg.
   // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rotation_sigh_sub_;
   // void rotation_sigh_callback(const std_msgs::msg::Bool &msg);
   std::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
   costmap_converter_msgs::msg::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message
 
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
   bool custom_via_points_active_; //!< Keep track whether valid via-points have been received from via_points_sub_
   std::mutex via_point_mutex_; //!< Mutex that locks the via_points container (multi-threaded)
   std::mutex update_wall_line_mutex_; //!< Mutex that locks the wall_line container (multi-threaded)
   std::mutex update_curb_line_mutex_; //!< Mutex that locks the wall_line container (multi-threaded)
   std::mutex update_edge_multi_curve_mutex_; //!< Mutex that locks the wall_line container (multi-threaded)

  // Vehicle poses around the robot (in map frame)
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vehicle_poses_sub_; //!< Subscriber for /vehicle_poses_around
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vehicle_scan_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr vehicle_scan_grid_pub_;
  std::mutex vehicle_scan_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_vehicle_cloud_;
  //!< 滚动栅格：255 自由，0 车相关占据；全局格索引 (ix,iy) 对齐 map
  std::vector<uint8_t> vehicle_scan_grid_;
  std::vector<uint8_t> vehicle_scan_stale_;
  std::vector<uint8_t> vehicle_scan_touched_;
  int vehicle_scan_origin_ix_{0};
  int vehicle_scan_origin_iy_{0};
  int vehicle_scan_nx_{0};
  int vehicle_scan_ny_{0};
  bool vehicle_scan_grid_ready_{false};

  std::vector<geometry_msgs::msg::PoseStamped> global_vehicle_poses_; //!< Filtered vehicle poses near the robot
  std::mutex global_vehicle_poses_mutex_; //!< Mutex that locks the global_vehicle_poses_ container (multi-threaded)
 
   PoseSE2 robot_pose_; //!< Store current robot pose
   PoseSE2 robot_goal_; //!< Store current robot goal
   geometry_msgs::msg::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
   rclcpp::Time time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
   int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
   rclcpp::Time time_last_oscillation_; //!< Store at which time stamp the last oscillation was detected
   RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
   geometry_msgs::msg::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()
   
   std::vector<geometry_msgs::msg::Point> footprint_spec_; //!< Store the footprint of the robot 
   double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
   double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot
     
   // flags
   bool initialized_; //!< Keeps track about the correct initialization of this class
   std::string name_; //!< Name of plugin ID
   double weight_wall_line_direction_, weight_wall_line_dist_;
   double weight_via_point_;
   double min_obstacle_dist_;
   geometry_msgs::msg::PoseStamped last_corner_pose_;
   // double via_sep_;
   // std::shared_ptr<DynamicGoalPub> dynamic_goal_pub_;
   std::shared_ptr<line_path_compare::LinePathCompare> wall_line_ptr_;
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_line_marker_publisher_;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr edge_distance_publisher_;
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr curb_line_subscriber_;
   rclcpp::Subscription<capella_ros_msg::msg::MultiCurve>::SharedPtr multi_curve_subscriber_;
   void multi_curve_callback(const capella_ros_msg::msg::MultiCurve::ConstSharedPtr msg);
   std::mutex multi_curve_mutex_;
   capella_ros_msg::msg::MultiCurve multi_curve_;
   rclcpp::Time multi_curve_update_time_;
   void curb_line_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);
   std::mutex curb_line_mutex_;
   nav_msgs::msg::Path curb_line_path_;
   rclcpp::Subscription<capella_ros_msg::msg::LaneCenterPaths>::SharedPtr edge_reference_paths_sub_;
   std::mutex edge_reference_paths_mutex_;
   capella_ros_msg::msg::LaneCenterPaths edge_reference_paths_cache_;
   rclcpp::Time edge_reference_paths_msg_time_{0};
   bool edge_reference_have_message_{false};
   std::vector<Eigen::Vector2d> reference_line_hold_;
   rclcpp::Time reference_line_last_success_time_{0};
   rclcpp::Time fusion_primary_lock_until_{0};
   double reference_line_hold_minimum_average_distance_to_plan_{0.0};
   double reference_line_hold_robot_perpendicular_distance_to_edge_line_{0.0};
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr transformed_path;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
   double cfg_max_angular_vel_, cfg_max_angular_acc_, cfg_max_vel_x_;
   rclcpp::Time wall_line_update_time_;
   rclcpp::Time curb_line_update_time_;
   // Parameters for normal mode (saved during initialization)
   double prune_angle_threshold_;  // rad, default 90deg
   double normal_weight_optimaltime_;
   double normal_min_obstacle_dist_;
   std::string normal_footprint_vertices_;
   // Parameters for edge-following mode
   double edge_weight_optimaltime_;
   double edge_min_obstacle_dist_;
   double min_wall_line_length_;
   std::string edge_footprint_vertices_;
   bool is_edge_following_mode_;

   // Wall line locking state
   bool wall_line_locked_{false};
   Eigen::Vector2d locked_wall_start_;
   Eigen::Vector2d locked_wall_end_;
   int wall_line_stable_count_{0};

   // Protruding obstacles that caused exit from edge-following mode
   std::vector<ProtrudingObstacle> protruding_obstacles_;
   std::vector<rclcpp::Time> protrusion_detection_timestamps_;  //!< Sliding window timestamps for protrusion confirmation

   rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr static_layer_client_; //!< Persistent client for static_layer parameter updates
   std::atomic<bool> desired_static_layer_state_{true}; //!< Desired state of static_layer.enabled
   std::atomic<bool> current_static_layer_state_{true}; //!< Current confirmed state of static_layer.enabled
   std::mutex static_layer_mutex_; //!< Mutex for protecting static_layer state updates
   
   // Footprint parameters for local and global costmaps
   std::string local_costmap_footprint_; //!< Stored local costmap footprint for restoration
   std::string global_costmap_footprint_; //!< Stored global costmap footprint for restoration
   rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_footprint_client_; //!< Client for local costmap footprint parameter updates
   rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_footprint_client_; //!< Client for global costmap footprint parameter updates
   std::atomic<bool> desired_local_footprint_state_{true}; //!< Desired state: true=normal footprint, false=edge footprint
   std::atomic<bool> desired_global_footprint_state_{true}; //!< Desired state: true=normal footprint, false=edge footprint
   std::atomic<bool> current_local_footprint_state_{true}; //!< Current confirmed state of local footprint
   std::atomic<bool> current_global_footprint_state_{true}; //!< Current confirmed state of global footprint
   std::atomic<bool> desired_normal_vel_x_restore_{true}; //!< Whether a delayed max_vel_x restore to normal is pending
   double speed_limit_linear_x_{std::numeric_limits<double>::infinity()};
   bool has_speed_limit_;
   double safe_linear_speed_limit_;
   double keep_wall_line_time_;
   /**
    * @brief 贴边退出：车辆是否在「机器人航向前后条带 ∩ 墙法向内外条带」内（墙段与机器人、车辆均为 map 系）
    */
   bool isVehicleInEdgeFollowingExitCorridor(
     const geometry_msgs::msg::PoseStamped& robot_pose,
     const Eigen::Vector2d& wall_w0,
     const Eigen::Vector2d& wall_w1,
     const geometry_msgs::msg::Pose& vehicle_pose) const;

   /**
    * @brief 末端路径点(plan系)变换到控制器坐标系后用 footprint 做占据检测；无障碍返回 true。
    */
   bool globalPlanPoseFootprintFreeInControllerFrame(
     const geometry_msgs::msg::PoseStamped& pose_plan_frame,
     const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
     geometry_msgs::msg::PoseStamped* out_pose_global_frame) const;

   /**
    * Oriented footprint（控制器系）相对局部滚动代价图：外包框不与图相交则视为无障碍；
    * 顶点全部落在图内时沿用 DWB scorePose；跨界时仅对已裁剪至图内的轮廓段做栅栏带代价判定。
    */
   bool orientedFootprintNavigableOnPartialCostmap(
     const geometry_msgs::msg::Pose2D & pose2d_controller_frame,
     const std::vector<geometry_msgs::msg::Point> & oriented_footprint) const;

   /**
    * @brief 参照 SMAC Hybrid：仅在末端点在局部代价地图上碰撞时，在 plan 系平面内网格搜索偏移，取欧式距离最短的无碰位姿。
    * @return 若末端已无障碍或搜索到替代位姿则为 true；无法找到仍为 false。
    */
   bool adjustOccupiedGlobalPlanGoalInPlace(
     const geometry_msgs::msg::PoseStamped& original_last_pose_plan_frame,
     const geometry_msgs::msg::TransformStamped& plan_to_global_transform,
     geometry_msgs::msg::PoseStamped& last_pose_plan_frame_inout) const;

   double new_vehicle_distance_threshold_;  //!< Distance threshold for adding/removing vehicles from global_vehicle_poses_
   double erase_vehicle_distance_threshold_;  //!< Distance threshold for adding/removing vehicles from global_vehicle_poses_
   static constexpr double same_vehicle_threshold_sq_ = 0.5;
 
   // In-place rotation
   double control_duration_;  // Control loop duration (1.0 / controller_frequency)
   std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> rotation_collision_checker_;
  geometry_msgs::msg::PoseStamped last_rotation_pose_;
  rclcpp::Time last_rotation_pose_time_{0};
  bool was_inplace_rotation_active_{false};
   
   /**
    * @brief Check if in-place rotation should be performed
    * @param velocity Current robot velocity (linear threshold only; collision prediction uses configured rotate speed)
    * @param transformed_plan The transformed plan (lookahead target heading only)
    * @param robot_pose Current robot pose
    * @return true if rotation should be performed, false otherwise
    */
   bool shouldRotateInPlace(
     const geometry_msgs::msg::Twist & velocity,
     const std::vector<geometry_msgs::msg::PoseStamped> & transformed_plan,
     const geometry_msgs::msg::PoseStamped & robot_pose);

   /**
    * @brief 原地转向碰障预检：按剩余角位移用 ω∈[rotate_min, rotate_to_heading] 与 max_angular_accel 减速模型 + 当前 ω 斜坡仿真
    */
   bool checkRotateToHeadingCollisionNominal(
     const double & angular_distance_to_heading,
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::Twist & velocity);

   /** oriented footprint 任一顶点不在局部 costmap 栅格内（越界）时返回 true，按“无碰”处理 */
   bool orientedFootprintAnyVertexOutsideLocalCostmap(
     double x, double y, double theta,
     const std::vector<geometry_msgs::msg::Point> & footprint_poly) const;

   /** 沿整条 plan 弧长从起点向前采样；全长取 plan 全长，向前有效长度 capped 为 trajectory.max_global_plan_lookahead_dist（≤0 则用 plan 全长）；footprint 判定同 isRotationCollisionFreeDecel；越界顶点不判碰 */
   bool isTransformedPlanFootprintSamplesCollisionFree(
     const std::vector<geometry_msgs::msg::PoseStamped> & plan,
     double sample_spacing_m) const;

   /** 剩余角位移 remaining(rad) 下允许的最大角速度幅值（减速至 rotate_min 所需距离由 ω²−ω_min²=2αs 决定） */
   /** @param omega_current_abs 当前角速度幅值(rad/s)，用于自动估计减速区间；手动 blend 时忽略 */
   double rotateToHeadingOmegaMagnitude(double remaining_angle_rad, double omega_current_abs) const;

   /** 沿 rotation_magnitude_rad 旋转弧均匀采样 yaw，仅 LETHAL 否决；与 footprint_spec_ 一致。initial_omega_z 保留兼容，不参与判定 */
   bool isRotationCollisionFreeDecel(
     const geometry_msgs::msg::PoseStamped & pose,
     double omega_direction_sign,
     double rotation_magnitude_rad,
     double initial_omega_z) const;
   
   /**
    * @brief Compute rotation command for in-place rotation
    * @param angular_distance_to_heading Angle difference to target heading
    * @param pose Current robot pose
    * @param velocity Current robot velocity (ω 按 max_angular_accel 斜坡逼近减速模型目标角速度)
    * @param[out] cmd_vel Output velocity command
    * @return true if rotation command is valid, false if collision detected
    */
   bool computeRotateToHeadingCommand(
     const double & angular_distance_to_heading,
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::Twist & velocity,
     geometry_msgs::msg::TwistStamped & cmd_vel);
   
   /**
    * @brief Check if rotation is collision-free
    * @param cmd_vel Velocity command to check
    * @param angular_distance_to_heading Angle difference to target heading
    * @param pose Current robot pose
    * @return true if collision-free, false otherwise
    */
   bool isRotationCollisionFree(
     const geometry_msgs::msg::TwistStamped & cmd_vel,
     const double & angular_distance_to_heading,
     const geometry_msgs::msg::PoseStamped & pose);
 
   // Speed limit topic handling
   void speedLimitCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_limit_sub_;
   std::mutex speed_limit_mutex_;
   
     
 protected:
   // Dynamic parameters handler
   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
   
 }; // end namespace teb_local_planner
 
 #endif // TEB_LOCAL_PLANNER_ROS_H_
 
 
 