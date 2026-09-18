/*********************************************************************
 * PATH-corridor wait / detour gate for confirmed dynamic obstacles.
 *
 * Subscribes independently from TEB `obstacles`. See
 * 动态障碍物识别与停车绕行方案.md
 *********************************************************************/

#ifndef TEB_LOCAL_PLANNER_PATH_DYNAMIC_WAIT_H
#define TEB_LOCAL_PLANNER_PATH_DYNAMIC_WAIT_H

#include <teb_local_planner/distance_calculations.h>
#include <teb_local_planner/teb_config.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace teb_local_planner
{

enum class PathWaitMode
{
  Follow = 0,
  Wait = 1,
  Detour = 2
};

struct PathWaitObstacle
{
  int id = -1;
  Point2dContainer polygon;  //!< xy hull in map; empty if circle/point
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;
};

struct PathWaitResult
{
  PathWaitMode mode = PathWaitMode::Follow;
  PathWaitMode previous_mode = PathWaitMode::Follow;
  std::vector<int> blocking_ids;
  std::vector<int> unlatched_blocking_ids;
  std::vector<int> waiting_ids;
  std::vector<int> latched_ids;
  double elapsed_wait_s = 0.0;
  double remaining_wait_s = 0.0;
  double half_width = 0.0;
  std::vector<Eigen::Vector2d> centerline;
  std::vector<Eigen::Vector2d> corridor_outline;
  std::vector<PathWaitObstacle> blocking_obstacles;
  std::string reason;
};

/**
 * @brief Compute footprint lateral half-width (max |y| and inscribed radius).
 */
double computePathWaitFootprintHalfWidth(
    const std::vector<geometry_msgs::msg::Point>& costmap_footprint,
    double inscribed_radius);

/**
 * @brief Sample PATH centerline from closest pose, along-arc up to lookahead.
 */
std::vector<Eigen::Vector2d> samplePathWaitCenterline(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const Eigen::Vector2d& robot_xy,
    double lookahead_dist);

class PathDynamicWaitGate
{
public:
  PathWaitResult evaluate(
      const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
      const Eigen::Vector2d& robot_xy,
      const std::vector<PathWaitObstacle>& obstacles,
      double footprint_half_width,
      double min_obstacle_dist,
      const TebConfig& cfg,
      const rclcpp::Time& now,
      bool message_valid);

  PathWaitMode mode() const { return mode_; }

private:
  void clearIdClocks();
  void updatePerIdClocks(const std::unordered_set<int>& blocking,
                         const rclcpp::Time& now,
                         double t_clear);

  PathWaitMode mode_ = PathWaitMode::Follow;
  rclcpp::Time clear_since_;
  bool has_clear_since_ = false;
  std::unordered_set<int> latched_ids_;
  std::unordered_map<int, rclcpp::Time> wait_enter_;
  std::unordered_map<int, rclcpp::Time> absent_since_;
};

void buildPathWaitDebugMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const PathWaitResult& result);

inline const char* pathWaitModeName(PathWaitMode mode)
{
  switch (mode)
  {
    case PathWaitMode::Wait: return "WAIT";
    case PathWaitMode::Detour: return "DETOUR";
    default: return "FOLLOW";
  }
}

inline std::string joinPathWaitIds(const std::vector<int>& ids)
{
  if (ids.empty())
  {
    return "-";
  }
  std::ostringstream ss;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (i)
    {
      ss << ",";
    }
    ss << ids[i];
  }
  return ss.str();
}

}  // namespace teb_local_planner

#endif
