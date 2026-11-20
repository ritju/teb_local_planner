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
#include <limits>
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
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double radius = 0.0;
};

struct PathWaitResult
{
  PathWaitMode mode = PathWaitMode::Follow;
  PathWaitMode previous_mode = PathWaitMode::Follow;
  std::vector<int> blocking_ids;
  std::vector<int> observed_ids; //!< 当前障碍轮廓与走廊相交；解锁依据，不含预测扫掠
  std::vector<int> unlatched_blocking_ids;
  std::vector<int> waiting_ids;
  std::vector<int> latched_ids;
  std::vector<int> awaiting_return_ids; //!< 当前轮廓确认离开，尚未再次进入
  std::vector<std::string> clock_events; //!< 每个 ID 的进入、离开、去抖及计时切换事件
  double elapsed_wait_s = 0.0;
  double remaining_wait_s = 0.0;
  double half_width = 0.0;
  bool has_projection = false;
  Eigen::Vector2d projection = Eigen::Vector2d::Zero();
  int projection_segment = -1;
  bool has_collision = false;
  Eigen::Vector2d collision_point = Eigen::Vector2d::Zero();
  double collision_arc = std::numeric_limits<double>::infinity();
  int collision_id = -1;
  bool braking_limited = false;
  double available_distance = std::numeric_limits<double>::infinity();
  double stopping_distance = 0.0;
  double safe_speed = std::numeric_limits<double>::infinity();
  std::vector<Eigen::Vector2d> centerline;
  std::vector<Eigen::Vector2d> corridor_outline;
  std::vector<PathWaitObstacle> blocking_obstacles;
  std::vector<PathWaitObstacle> predicted_blocking_obstacles;
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
    double lookahead_dist,
    int first_segment = 0,
    int last_segment = -1);

struct PathWaitProjection
{
  Eigen::Vector2d point = Eigen::Vector2d::Zero();
  int segment = -1;
  double fraction = 0.0;
  double arc = 0.0;
};

PathWaitProjection projectPathWaitPose(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    const Eigen::Vector2d& robot_xy,
    int first_segment = 0,
    int last_segment = -1);

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
      bool message_valid,
      double robot_yaw = 0.0,
      const std::vector<geometry_msgs::msg::Point>& footprint = {});

  PathWaitMode mode() const { return mode_; }

private:
  void clearIdClocks(bool preserve_awaiting_return = false);
  void updatePerIdClocks(const std::unordered_set<int>& blocking,
                         const std::unordered_set<int>& observed,
                         const rclcpp::Time& now,
                         double t_clear,
                         std::vector<std::string>& events);

  PathWaitMode mode_ = PathWaitMode::Follow;
  rclcpp::Time clear_since_;
  bool has_clear_since_ = false;
  std::unordered_set<int> latched_ids_;
  std::unordered_map<int, rclcpp::Time> wait_enter_;
  std::unordered_map<int, rclcpp::Time> absent_since_;
  std::unordered_set<int> observed_once_ids_;
  std::unordered_set<int> awaiting_return_ids_; //!< 当前轮廓离开已确认，返回时才重开等待
  bool has_progress_ = false;
  Eigen::Vector2d last_projection_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d last_goal_ = Eigen::Vector2d::Zero();
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

inline const char* pathWaitModeChinese(PathWaitMode mode)
{
  switch (mode)
  {
    case PathWaitMode::Wait: return "等待";
    case PathWaitMode::Detour: return "绕行";
    default: return "跟随";
  }
}

inline const char* pathWaitReasonChinese(const std::string& reason)
{
  if (reason == "disabled") return "功能关闭";
  if (reason == "corridor_clear") return "走廊清空";
  if (reason == "clear_hold") return "清空去抖";
  if (reason == "dynamic_on_path") return "路径上有动态障碍";
  if (reason == "id_clear_hold") return "障碍编号离开去抖";
  if (reason == "wait_timeout") return "等待超时";
  if (reason == "latched_detour") return "已锁定绕行";
  if (reason == "awaiting_current_return") return "等待当前轮廓返回后再计时";
  return "未知";
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
