/*********************************************************************
 * Near-horizon dynamic-obstacle safety helpers for predictable motion.
 *
 * Cache + threat gate + RViz debug markers. See 动态障碍物避让.md.
 *********************************************************************/

#ifndef TEB_LOCAL_PLANNER_NEAR_HORIZON_DYNAMIC_SAFETY_H
#define TEB_LOCAL_PLANNER_NEAR_HORIZON_DYNAMIC_SAFETY_H

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/teb_config.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace teb_local_planner
{

/**
 * @brief One cached dynamic obstacle sample (survives short perception dropouts).
 */
struct CachedDynamicObstacle
{
  int id = -1;                          //!< Association id (-1 if unknown)
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero(); //!< Raw planar velocity before clamp
  std::vector<Eigen::Vector2d> contour; //!< Optional polygon vertices in map frame (empty => point)
  double radius = 0.0;                  //!< Optional circular radius
  rclcpp::Time last_seen;
  bool from_cache = false;              //!< True if coasting without fresh measurement
};

/**
 * @brief Footprint AABB in robot frame + ROI expansions (meters beyond each side).
 *
 * ROI box in robot frame:
 *   x in [-(rear_extent + roi_rear),  +(front_extent + roi_front)]
 *   y in [-(right_extent + roi_right), +(left_extent + roi_left)]
 */
struct FootprintRoiBox
{
  double front_extent = 0.5; //!< max x of footprint in robot frame
  double rear_extent = 0.5;  //!< |min x| of footprint in robot frame
  double left_extent = 0.5;  //!< max y
  double right_extent = 0.5; //!< |min y|

  double roiXMax(double roi_front) const { return front_extent + roi_front; }
  double roiXMin(double roi_rear) const { return -(rear_extent + roi_rear); }
  double roiYMax(double roi_left) const { return left_extent + roi_left; }
  double roiYMin(double roi_right) const { return -(right_extent + roi_right); }
};

/**
 * @brief Result of near-horizon threat evaluation (feeds OmegaHold + debug).
 */
struct NearHorizonThreatResult
{
  bool raw_threat = false;           //!< Threat this cycle before hysteresis
  bool enable_omega_hold = false;    //!< After hold_time hysteresis
  bool excluded_overlap = false;     //!< At least one obstacle overlapped robot (OmegaHold skipped for it)
  int trigger_obstacle_id = -1;
  double trigger_min_dist = 1e9;
  std::string reason;                //!< Short human-readable reason for logs
};

/**
 * @brief Short-horizon cache for dynamic obstacles (anti frame-drop).
 */
class DynamicObstacleCache
{
public:
  void configure(double cache_time_sec) { cache_time_sec_ = cache_time_sec; }

  /**
   * @brief Merge fresh dynamic obstacles into cache; drop expired entries.
   * @param obstacles Current planner obstacle container
   * @param now Current time
   */
  void update(const ObstContainer& obstacles, const rclcpp::Time& now);

  const std::vector<CachedDynamicObstacle>& get() const { return cached_; }

private:
  double cache_time_sec_ = 0.3;
  std::vector<CachedDynamicObstacle> cached_;
  int next_synthetic_id_ = 100000;
};

/**
 * @brief Near-horizon threat gate for EdgeNearHorizonOmegaHold.
 *
 * Pipeline per obstacle:
 *   clamp velocity -> short-horizon contour extrapolation ->
 *   if overlap with current robot footprint: exclude (static edges only) ->
 *   else if extrapolated contour enters footprint+ROI: mark threat ->
 *   time hysteresis on enable.
 */
class NearHorizonThreatGate
{
public:
  NearHorizonThreatResult evaluate(
      const PoseSE2& robot_pose,
      const RobotFootprintModelConstPtr& robot_model,
      const FootprintRoiBox& footprint_roi,
      const std::vector<CachedDynamicObstacle>& cached,
      const TebConfig& cfg,
      const rclcpp::Time& now);

  /** Last evaluation debug samples (map-frame extrapolated centroids). */
  const std::vector<Eigen::Vector2d>& lastExtrapolatedSamples() const { return last_samples_; }
  const NearHorizonThreatResult& lastResult() const { return last_result_; }

private:
  Eigen::Vector2d clampVelocity(const Eigen::Vector2d& v, double v_x_max) const;
  bool pointInRoiRobotFrame(const Eigen::Vector2d& p_robot, const FootprintRoiBox& fp,
                            double roi_front, double roi_rear, double roi_left, double roi_right,
                            double inflation) const;

  bool enable_latched_ = false;
  rclcpp::Time last_raw_threat_time_;
  bool has_last_raw_threat_time_ = false;

  NearHorizonThreatResult last_result_;
  std::vector<Eigen::Vector2d> last_samples_;
};

/**
 * @brief Build RViz MarkerArray for near-horizon debug visualization.
 */
void buildNearHorizonDebugMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const PoseSE2& robot_pose,
    const FootprintRoiBox& footprint_roi,
    const TebConfig& cfg,
    const std::vector<CachedDynamicObstacle>& cached,
    const NearHorizonThreatResult& threat,
    const std::vector<Eigen::Vector2d>& extrapolated_samples,
    const std::vector<geometry_msgs::msg::Point>& footprint_robot_frame);

/**
 * @brief Compute footprint AABB extents in robot frame from polygon points (robot frame).
 */
FootprintRoiBox computeFootprintRoiBox(const std::vector<geometry_msgs::msg::Point>& footprint_robot_frame);

} // namespace teb_local_planner

#endif
