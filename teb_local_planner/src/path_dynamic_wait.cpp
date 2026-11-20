/*********************************************************************
 * PATH-corridor wait / detour gate for confirmed dynamic obstacles.
 *********************************************************************/

#include <teb_local_planner/path_dynamic_wait.h>
#include <teb_local_planner/occupied_goal_braking.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace teb_local_planner
{

namespace
{

bool pointInPolygon(const Eigen::Vector2d& p, const Point2dContainer& v)
{
  const int n = static_cast<int>(v.size());
  if (n < 3)
  {
    return false;
  }
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    const double yi = v[i].y();
    const double yj = v[j].y();
    const double xi = v[i].x();
    const double xj = v[j].x();
    const bool intersect = ((yi > p.y()) != (yj > p.y())) &&
      (p.x() < (xj - xi) * (p.y() - yi) / (yj - yi + 1e-12) + xi);
    if (intersect)
    {
      inside = !inside;
    }
  }
  return inside;
}

double distPointToPolyline(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& line)
{
  if (line.empty())
  {
    return std::numeric_limits<double>::infinity();
  }
  if (line.size() == 1)
  {
    return (p - line.front()).norm();
  }
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < line.size(); ++i)
  {
    best = std::min(best, distance_point_to_segment_2d(p, line[i], line[i + 1]));
  }
  return best;
}

bool obstacleHitsCorridor(
    const PathWaitObstacle& obst,
    const std::vector<Eigen::Vector2d>& centerline,
    double half_width)
{
  if (centerline.empty() || half_width < 0.0)
  {
    return false;
  }

  if (obst.polygon.size() >= 3)
  {
    for (const Eigen::Vector2d& p : centerline)
    {
      if (pointInPolygon(p, obst.polygon))
      {
        return true;
      }
    }
    if (centerline.size() == 1)
    {
      return distance_point_to_polygon_2d(centerline.front(), obst.polygon) <= half_width;
    }
    for (size_t i = 0; i + 1 < centerline.size(); ++i)
    {
      if (distance_segment_to_polygon_2d(centerline[i], centerline[i + 1], obst.polygon) <= half_width)
      {
        return true;
      }
    }
    return false;
  }

  if (obst.polygon.size() == 2)
  {
    if (centerline.size() == 1)
      return distance_point_to_segment_2d(centerline.front(), obst.polygon[0], obst.polygon[1]) <= half_width + std::max(0.0, obst.radius);
    for (size_t i = 0; i + 1 < centerline.size(); ++i)
      if (distance_segment_to_segment_2d(centerline[i], centerline[i + 1], obst.polygon[0], obst.polygon[1]) <= half_width + std::max(0.0, obst.radius))
        return true;
    return false;
  }
  const double r = std::max(0.0, obst.radius);
  return distPointToPolyline(obst.center, centerline) <= (half_width + r);
}

double distanceFootprintToObstacle(const Point2dContainer& footprint, const PathWaitObstacle& obst)
{
  if (footprint.empty()) return std::numeric_limits<double>::infinity();
  if (obst.polygon.size() >= 2)
  {
    if (obst.polygon.size() >= 3 && pointInPolygon(footprint.front(), obst.polygon)) return 0.0;
    if (pointInPolygon(obst.polygon.front(), footprint)) return 0.0;
    double distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < footprint.size(); ++i)
      for (size_t j = 0; j + 1 < obst.polygon.size(); ++j)
        distance = std::min(distance, distance_segment_to_segment_2d(
            footprint[i], footprint[(i + 1) % footprint.size()],
            obst.polygon[j], obst.polygon[j + 1]));
    if (obst.polygon.size() >= 3)
      for (size_t i = 0; i < footprint.size(); ++i)
        distance = std::min(distance, distance_segment_to_segment_2d(
            footprint[i], footprint[(i + 1) % footprint.size()],
            obst.polygon.back(), obst.polygon.front()));
    return std::max(0.0, distance - obst.radius);
  }
  const Eigen::Vector2d center = obst.polygon.empty() ? obst.center : obst.polygon.front();
  if (pointInPolygon(center, footprint)) return 0.0;
  return std::max(0.0, distance_point_to_polygon_2d(center, footprint) - obst.radius);
}

Point2dContainer placedFootprint(const std::vector<geometry_msgs::msg::Point>& footprint,
                                 const Eigen::Vector2d& at, double yaw)
{
  Point2dContainer placed;
  const double c = std::cos(yaw), s = std::sin(yaw);
  placed.reserve(footprint.size());
  for (const auto& p : footprint)
    placed.emplace_back(at.x() + c * p.x - s * p.y, at.y() + s * p.x + c * p.y);
  return placed;
}

PathWaitObstacle sweptObstacle(const PathWaitObstacle& source, double horizon)
{
  PathWaitObstacle swept = source;
  const Eigen::Vector2d travel = source.velocity * horizon;
  if (!std::isfinite(travel.x()) || !std::isfinite(travel.y()) ||
      travel.squaredNorm() < 1e-12)
    return swept;

  Point2dContainer points = source.polygon;
  if (points.empty()) points.push_back(source.center);
  const size_t original_size = points.size();
  points.reserve(original_size * 2);
  for (size_t i = 0; i < original_size; ++i) points.push_back(points[i] + travel);
  std::sort(points.begin(), points.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                  const Eigen::Vector2d& c) {
    const Eigen::Vector2d ab = b - a, ac = c - a;
    return ab.x() * ac.y() - ab.y() * ac.x();
  };
  Point2dContainer hull;
  for (const auto& p : points)
  {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0.0)
      hull.pop_back();
    hull.push_back(p);
  }
  const size_t lower_size = hull.size();
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; --i)
  {
    while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), points[i]) <= 0.0)
      hull.pop_back();
    hull.push_back(points[i]);
  }
  if (hull.size() > 1) hull.pop_back();
  swept.polygon = std::move(hull);
  return swept;
}

bool firstFootprintCollision(const std::vector<Eigen::Vector2d>& line,
                             const std::vector<geometry_msgs::msg::Point>& footprint,
                             double robot_yaw, bool has_connector,
                             const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                             int projection_segment,
                             const PathWaitObstacle& obst, double clearance,
                             double& hit_arc, Eigen::Vector2d& hit_point)
{
  if (line.empty() || footprint.size() < 3) return false;
  double arc = 0.0;
  for (size_t i = 0; i < line.size(); ++i)
  {
    const Eigen::Vector2d a = i == 0 ? line.front() : line[i - 1];
    const Eigen::Vector2d b = line[i];
    const double length = (b - a).norm();
    double yaw = robot_yaw;
    if (i > (has_connector ? 1u : 0u) && length >= 1e-6)
    {
      const int plan_index = projection_segment + static_cast<int>(i) - (has_connector ? 2 : 1);
      if (plan_index >= 0 && plan_index < static_cast<int>(plan.size()))
      {
        const auto& q = plan[plan_index].pose.orientation;
        yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      }
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.025)));
    for (int j = 0; j <= steps; ++j)
    {
      const double fraction = static_cast<double>(j) / steps;
      const Eigen::Vector2d at = a + fraction * (b - a);
      if (distanceFootprintToObstacle(placedFootprint(footprint, at, yaw), obst) <= clearance + 0.025)
      {
        hit_arc = arc + fraction * length;
        hit_point = at;
        return true;
      }
    }
    arc += length;
  }
  return false;
}

void offsetPolyline(
    const std::vector<Eigen::Vector2d>& c,
    double w,
    std::vector<Eigen::Vector2d>& left,
    std::vector<Eigen::Vector2d>& right)
{
  left.clear();
  right.clear();
  if (c.empty())
  {
    return;
  }
  if (c.size() == 1)
  {
    const Eigen::Vector2d n(0.0, 1.0);
    left.push_back(c.front() + w * n);
    right.push_back(c.front() - w * n);
    return;
  }
  left.resize(c.size());
  right.resize(c.size());
  for (size_t i = 0; i < c.size(); ++i)
  {
    Eigen::Vector2d t;
    if (i == 0)
    {
      t = c[1] - c[0];
    }
    else if (i + 1 == c.size())
    {
      t = c[i] - c[i - 1];
    }
    else
    {
      t = c[i + 1] - c[i - 1];
    }
    if (t.squaredNorm() < 1e-12)
    {
      t = Eigen::Vector2d(1.0, 0.0);
    }
    t.normalize();
    const Eigen::Vector2d n(-t.y(), t.x());
    left[i] = c[i] + w * n;
    right[i] = c[i] - w * n;
  }
}

std::vector<Eigen::Vector2d> makeCorridorOutline(
    const std::vector<Eigen::Vector2d>& centerline, double half_width)
{
  std::vector<Eigen::Vector2d> left;
  std::vector<Eigen::Vector2d> right;
  offsetPolyline(centerline, half_width, left, right);
  std::vector<Eigen::Vector2d> outline;
  outline.reserve(left.size() + right.size() + 1);
  outline.insert(outline.end(), left.begin(), left.end());
  outline.insert(outline.end(), right.rbegin(), right.rend());
  if (!outline.empty())
  {
    outline.push_back(outline.front());
  }
  return outline;
}

visualization_msgs::msg::Marker baseMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int id,
    int type)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = rclcpp::Duration::from_seconds(0.4);
  return m;
}

}  // namespace

double computePathWaitFootprintHalfWidth(
    const std::vector<geometry_msgs::msg::Point>& costmap_footprint,
    double inscribed_radius)
{
  double half = std::max(0.0, inscribed_radius);
  for (const geometry_msgs::msg::Point& p : costmap_footprint)
  {
    half = std::max(half, std::abs(p.y));
  }
  return half;
}

PathWaitProjection projectPathWaitPose(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    const Eigen::Vector2d& robot_xy, int first_segment, int last_segment)
{
  PathWaitProjection best;
  if (plan.empty()) return best;
  if (plan.size() == 1)
  {
    best.point = Eigen::Vector2d(plan.front().pose.position.x, plan.front().pose.position.y);
    best.segment = 0;
    return best;
  }
  first_segment = std::max(0, first_segment);
  last_segment = last_segment < 0 ? static_cast<int>(plan.size()) - 2 :
    std::min(last_segment, static_cast<int>(plan.size()) - 2);
  double best_sq = std::numeric_limits<double>::infinity();
  double arc = 0.0;
  for (int i = 0; i + 1 < static_cast<int>(plan.size()); ++i)
  {
    const Eigen::Vector2d a(plan[i].pose.position.x, plan[i].pose.position.y);
    const Eigen::Vector2d b(plan[i + 1].pose.position.x, plan[i + 1].pose.position.y);
    const Eigen::Vector2d delta = b - a;
    const double length = delta.norm();
    if (i >= first_segment && i <= last_segment && length > 1e-9)
    {
      const double fraction = std::max(0.0, std::min(1.0,
        (robot_xy - a).dot(delta) / delta.squaredNorm()));
      const Eigen::Vector2d point = a + fraction * delta;
      const double sq = (robot_xy - point).squaredNorm();
      if (sq < best_sq)
      {
        best_sq = sq;
        best.point = point;
        best.segment = i;
        best.fraction = fraction;
        best.arc = arc + fraction * length;
      }
    }
    arc += length;
  }
  return best;
}

std::vector<Eigen::Vector2d> samplePathWaitCenterline(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const Eigen::Vector2d& robot_xy,
    double lookahead_dist,
    int first_segment,
    int last_segment)
{
  std::vector<Eigen::Vector2d> out;
  if (global_plan.empty() || lookahead_dist <= 0.0)
  {
    return out;
  }

  const auto projection = projectPathWaitPose(global_plan, robot_xy, first_segment, last_segment);
  if (projection.segment < 0) return out;
  out.push_back(robot_xy);
  if ((projection.point - robot_xy).norm() > 1e-6) out.push_back(projection.point);
  double acc = 0.0;
  Eigen::Vector2d a = projection.point;
  for (int i = projection.segment; i + 1 < static_cast<int>(global_plan.size()); ++i)
  {
    const Eigen::Vector2d b(global_plan[i + 1].pose.position.x, global_plan[i + 1].pose.position.y);
    const double d = (b - a).norm();
    if (d < 1e-6)
    {
      continue;
    }
    if (acc + d >= lookahead_dist)
    {
      const double frac = std::min(1.0, std::max(0.0, (lookahead_dist - acc) / d));
      out.push_back(a + frac * (b - a));
      return out;
    }
    acc += d;
    out.push_back(b);
    a = b;
  }
  return out;
}

void PathDynamicWaitGate::clearIdClocks(bool preserve_awaiting_return)
{
  latched_ids_.clear();
  wait_enter_.clear();
  absent_since_.clear();
  observed_once_ids_.clear();
  if (!preserve_awaiting_return)
    awaiting_return_ids_.clear();
}

void PathDynamicWaitGate::updatePerIdClocks(
    const std::unordered_set<int>& blocking,
    const std::unordered_set<int>& observed,
    const rclcpp::Time& now,
    double t_clear,
    std::vector<std::string>& events)
{
  for (int id : observed)
  {
    if (awaiting_return_ids_.erase(id) != 0)
    {
      wait_enter_.erase(id);
      latched_ids_.erase(id);
      events.push_back("编号=" + std::to_string(id) + " 当前轮廓返回走廊，重新开始完整等待");
    }
    else if (observed_once_ids_.count(id) == 0)
    {
      events.push_back("编号=" + std::to_string(id) + " 当前轮廓进入走廊");
    }
    else if (absent_since_.count(id) != 0)
    {
      events.push_back("编号=" + std::to_string(id) + " 当前轮廓短暂离开后返回，保留原等待计时");
    }
    observed_once_ids_.insert(id);
    absent_since_.erase(id);
  }
  for (int id : blocking)
  {
    if (awaiting_return_ids_.count(id) != 0 && mode_ == PathWaitMode::Follow)
    {
      events.push_back("编号=" + std::to_string(id) +
        " 预测阻挡再次出现，但当前轮廓尚未返回；保持待返回，不启动6秒计时");
    }
    if (observed_once_ids_.count(id) == 0 && absent_since_.erase(id) != 0)
    {
      events.push_back("编号=" + std::to_string(id) +
        " 预测阻挡再次出现，清空尚未确认，保留原等待计时");
    }
  }

  std::vector<int> tracked;
  tracked.reserve(wait_enter_.size() + latched_ids_.size());
  for (std::unordered_map<int, rclcpp::Time>::const_iterator it = wait_enter_.begin();
       it != wait_enter_.end(); ++it)
  {
    tracked.push_back(it->first);
  }
  for (std::unordered_set<int>::const_iterator it = latched_ids_.begin();
       it != latched_ids_.end(); ++it)
  {
    tracked.push_back(*it);
  }
  for (size_t i = 0; i < tracked.size(); ++i)
  {
    const int id = tracked[i];
    // 当前轮廓离开后先确认离开；确认后等待真正返回，再重开完整等待。
    // 仅有预测阻挡的 ID，必须等预测连续清空后才重置计时。
    if (observed.count(id) == 0 && awaiting_return_ids_.count(id) == 0 &&
        (observed_once_ids_.count(id) != 0 || blocking.count(id) == 0) &&
        absent_since_.count(id) == 0)
    {
      absent_since_[id] = now;
      events.push_back("编号=" + std::to_string(id) +
        (observed_once_ids_.count(id) != 0 ? " 当前轮廓离开走廊，开始去抖" :
         " 预测阻挡消失，开始去抖"));
    }
  }

  std::vector<int> prune;
  for (std::unordered_map<int, rclcpp::Time>::const_iterator it = absent_since_.begin();
       it != absent_since_.end(); ++it)
  {
    if ((now - it->second).seconds() >= t_clear)
    {
      prune.push_back(it->first);
    }
  }
  for (size_t i = 0; i < prune.size(); ++i)
  {
    const int id = prune[i];
    absent_since_.erase(id);
    if (observed_once_ids_.count(id) != 0)
    {
      awaiting_return_ids_.insert(id);
      events.push_back("编号=" + std::to_string(id) +
        " 当前轮廓离开已确认；预测仍阻挡时等待实际返回再重置计时");
    }
    else
    {
      wait_enter_.erase(id);
      latched_ids_.erase(id);
      events.push_back("编号=" + std::to_string(id) + " 预测阻挡连续清空，重置等待计时");
    }
  }
  // 当前轮廓返回时已先解锁；只在确实阻挡的这一拍开启新计时。
  for (int id : blocking)
  {
    if (awaiting_return_ids_.count(id) == 0 &&
        latched_ids_.count(id) == 0 && wait_enter_.count(id) == 0)
    {
      wait_enter_[id] = now;
      events.push_back("编号=" + std::to_string(id) + " 预测阻挡出现，开始完整等待");
    }
  }
}

PathWaitResult PathDynamicWaitGate::evaluate(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const Eigen::Vector2d& robot_xy,
    const std::vector<PathWaitObstacle>& obstacles,
    double footprint_half_width,
    double min_obstacle_dist,
    const TebConfig& cfg,
    const rclcpp::Time& now,
    bool message_valid,
    double robot_yaw,
    const std::vector<geometry_msgs::msg::Point>& footprint)
{
  PathWaitResult result;
  result.previous_mode = mode_;
  const double scale = std::max(0.0, cfg.obstacles.path_wait_corridor_scale);
  result.half_width = std::max(0.0, footprint_half_width) +
                      std::max(0.0, min_obstacle_dist) * scale;
  int first_segment = 0, last_segment = -1;
  if (!global_plan.empty())
  {
    const Eigen::Vector2d goal(global_plan.back().pose.position.x, global_plan.back().pose.position.y);
    if (has_progress_ && (goal - last_goal_).norm() < 0.1 &&
        (robot_xy - last_projection_).norm() < cfg.obstacles.path_wait_lookahead_dist)
    {
      const auto anchor = projectPathWaitPose(global_plan, last_projection_);
      if (anchor.segment >= 0 && (anchor.point - last_projection_).norm() < 0.5)
      {
        first_segment = anchor.segment;
        last_segment = anchor.segment;
        double back = 0.0, ahead = 0.0;
        while (first_segment > 0 && back < 2.0)
        {
          --first_segment;
          back += std::hypot(global_plan[first_segment + 1].pose.position.x - global_plan[first_segment].pose.position.x,
                             global_plan[first_segment + 1].pose.position.y - global_plan[first_segment].pose.position.y);
        }
        while (last_segment + 2 < static_cast<int>(global_plan.size()) &&
               ahead < cfg.obstacles.path_wait_lookahead_dist)
        {
          ++last_segment;
          ahead += std::hypot(global_plan[last_segment + 1].pose.position.x - global_plan[last_segment].pose.position.x,
                              global_plan[last_segment + 1].pose.position.y - global_plan[last_segment].pose.position.y);
        }
      }
    }
    const auto projection = projectPathWaitPose(global_plan, robot_xy, first_segment, last_segment);
    if (projection.segment >= 0)
    {
      result.has_projection = true;
      result.projection = projection.point;
      result.projection_segment = projection.segment;
      last_projection_ = projection.point;
      last_goal_ = goal;
      has_progress_ = true;
    }
  }
  result.centerline = samplePathWaitCenterline(
      global_plan, robot_xy, cfg.obstacles.path_wait_lookahead_dist, first_segment, last_segment);
  result.corridor_outline = makeCorridorOutline(result.centerline, result.half_width);

  if (!cfg.obstacles.path_wait_enable)
  {
    mode_ = PathWaitMode::Follow;
    clearIdClocks();
    has_clear_since_ = false;
    result.mode = mode_;
    result.reason = "disabled";
    return result;
  }

  std::unordered_set<int> blocking;
  std::unordered_set<int> observed;
  if (message_valid)
  {
    for (const PathWaitObstacle& obst : obstacles)
    {
      double hit_arc = 0.0;
      Eigen::Vector2d hit_point;
      const double deceleration = std::max(1e-6, cfg.robot.acc_lim_x);
      const double horizon = std::max(0.0, cfg.obstacles.path_wait_brake_response_time) +
                             std::max(0.0, cfg.robot.max_vel_x) / deceleration;
      const PathWaitObstacle predicted = sweptObstacle(obst, horizon);
      double current_hit_arc = 0.0;
      Eigen::Vector2d current_hit_point;
      const bool current_footprint_hit = firstFootprintCollision(
          result.centerline, footprint, robot_yaw,
          result.has_projection && (robot_xy - result.projection).norm() > 1e-6,
          global_plan, result.projection_segment, obst,
          std::max(0.0, min_obstacle_dist) * scale,
          current_hit_arc, current_hit_point);
      if (current_footprint_hit ||
          obstacleHitsCorridor(obst, result.centerline, result.half_width))
        observed.insert(obst.id);
      const bool footprint_hit = firstFootprintCollision(
          result.centerline, footprint, robot_yaw,
          result.has_projection && (robot_xy - result.projection).norm() > 1e-6,
          global_plan, result.projection_segment, predicted,
          std::max(0.0, min_obstacle_dist) * scale, hit_arc, hit_point);
      if (footprint_hit && hit_arc < result.collision_arc)
      {
        result.has_collision = true;
        result.collision_arc = hit_arc;
        result.collision_point = hit_point;
        result.collision_id = obst.id;
        result.available_distance = hit_arc;
      }
      if (footprint_hit || obstacleHitsCorridor(predicted, result.centerline, result.half_width))
      {
        blocking.insert(obst.id);
        result.blocking_obstacles.push_back(obst);
        result.predicted_blocking_obstacles.push_back(predicted);
      }
    }
  }

  result.blocking_ids.assign(blocking.begin(), blocking.end());
  std::sort(result.blocking_ids.begin(), result.blocking_ids.end());
  result.observed_ids.assign(observed.begin(), observed.end());
  std::sort(result.observed_ids.begin(), result.observed_ids.end());

  const double t_clear = std::max(0.0, cfg.obstacles.path_wait_clear_time);
  const double t_wait = std::max(0.0, cfg.obstacles.path_wait_timeout);
  updatePerIdClocks(blocking, observed, now, t_clear, result.clock_events);

  if (blocking.empty())
  {
    if (!has_clear_since_)
    {
      clear_since_ = now;
      has_clear_since_ = true;
    }
    const double clear_s = (now - clear_since_).seconds();
    if (clear_s >= t_clear)
    {
      mode_ = PathWaitMode::Follow;
      // 全局走廊清空只结束本轮 WAIT；同一 ID 的当前轮廓返回条件仍需保留。
      clearIdClocks(true);
      result.reason = "corridor_clear";
    }
    else
    {
      result.reason = "clear_hold";
    }
  }
  else
  {
    has_clear_since_ = false;

    bool latched_now = false;
    for (int id : result.blocking_ids)
    {
      if (latched_ids_.count(id) != 0)
      {
        continue;
      }
      std::unordered_map<int, rclcpp::Time>::const_iterator it = wait_enter_.find(id);
      const double elapsed = (it == wait_enter_.end()) ? 0.0 : (now - it->second).seconds();
      if (elapsed >= t_wait)
      {
        latched_ids_.insert(id);
        wait_enter_.erase(id);
        latched_now = true;
        result.clock_events.push_back("编号=" + std::to_string(id) +
          " 等待达到" + std::to_string(t_wait) + "秒，锁定绕行");
      }
    }

    bool still_waiting = false;
    bool waiting_on_path = false;
    double max_elapsed = 0.0;
    double min_remaining = std::numeric_limits<double>::infinity();
    for (std::unordered_map<int, rclcpp::Time>::const_iterator it = wait_enter_.begin();
         it != wait_enter_.end(); ++it)
    {
      const int id = it->first;
      if (latched_ids_.count(id) != 0)
      {
        continue;
      }
      const double elapsed = (now - it->second).seconds();
      if (elapsed >= t_wait)
      {
        continue;
      }
      still_waiting = true;
      result.waiting_ids.push_back(id);
      if (elapsed > max_elapsed)
      {
        max_elapsed = elapsed;
      }
      const double remaining = t_wait - elapsed;
      if (remaining < min_remaining)
      {
        min_remaining = remaining;
      }
      if (blocking.count(id) != 0)
      {
        waiting_on_path = true;
      }
    }
    std::sort(result.waiting_ids.begin(), result.waiting_ids.end());

    if (still_waiting)
    {
      mode_ = PathWaitMode::Wait;
      result.elapsed_wait_s = max_elapsed;
      result.remaining_wait_s = min_remaining;
      result.reason = waiting_on_path ? "dynamic_on_path" : "id_clear_hold";
    }
    else
    {
      bool awaiting_current_return = false;
      for (int id : result.blocking_ids)
      {
        if (awaiting_return_ids_.count(id) != 0)
        {
          awaiting_current_return = true;
          result.waiting_ids.push_back(id);
        }
      }
      if (awaiting_current_return)
      {
        // 预测仍阻挡时先停车，不能在等待当前轮廓返回期间放行绕行。
        // 该阶段不累计 6 秒；实际返回时 updatePerIdClocks 才重新开钟。
        mode_ = PathWaitMode::Wait;
        result.remaining_wait_s = t_wait;
        result.reason = "awaiting_current_return";
      }
      else
      {
        mode_ = PathWaitMode::Detour;
        result.reason = latched_now ? "wait_timeout" : "latched_detour";
      }
    }
  }

  for (int id : result.blocking_ids)
  {
    if (latched_ids_.count(id) == 0)
    {
      result.unlatched_blocking_ids.push_back(id);
    }
  }

  result.mode = mode_;
  result.latched_ids.assign(latched_ids_.begin(), latched_ids_.end());
  std::sort(result.latched_ids.begin(), result.latched_ids.end());
  result.awaiting_return_ids.assign(awaiting_return_ids_.begin(), awaiting_return_ids_.end());
  std::sort(result.awaiting_return_ids.begin(), result.awaiting_return_ids.end());
  return result;
}

void buildPathWaitDebugMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const PathWaitResult& result)
{
  visualization_msgs::msg::Marker del;
  del.header.frame_id = frame_id;
  del.header.stamp = stamp;
  del.ns = "";
  del.id = 0;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(del);

  std_msgs::msg::ColorRGBA col;
  col.a = 0.9;
  if (result.mode == PathWaitMode::Wait)
  {
    col.r = 1.0; col.g = 0.15; col.b = 0.15;
  }
  else if (result.mode == PathWaitMode::Detour)
  {
    col.r = 1.0; col.g = 0.55; col.b = 0.05;
  }
  else
  {
    col.r = 0.15; col.g = 0.85; col.b = 0.25;
  }

  if (result.corridor_outline.size() >= 2)
  {
    visualization_msgs::msg::Marker strip = baseMarker(
        frame_id, stamp, "corridor", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    strip.scale.x = 0.04;
    strip.color = col;
    strip.color.a = 0.8;
    for (const Eigen::Vector2d& p : result.corridor_outline)
    {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = 0.05;
      strip.points.push_back(gp);
    }
    markers.markers.push_back(strip);
  }

  if (result.centerline.size() >= 2)
  {
    visualization_msgs::msg::Marker cl = baseMarker(
        frame_id, stamp, "centerline", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    cl.scale.x = 0.03;
    cl.color.r = 0.2; cl.color.g = 0.4; cl.color.b = 1.0; cl.color.a = 0.9;
    for (const Eigen::Vector2d& p : result.centerline)
    {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = 0.06;
      cl.points.push_back(gp);
    }
    markers.markers.push_back(cl);
  }

  if (result.has_projection)
  {
    auto projection = baseMarker(frame_id, stamp, "path_projection", 0,
                                 visualization_msgs::msg::Marker::SPHERE);
    projection.pose.position.x = result.projection.x();
    projection.pose.position.y = result.projection.y();
    projection.pose.position.z = 0.15;
    projection.scale.x = projection.scale.y = projection.scale.z = 0.18;
    projection.color.r = 0.0; projection.color.g = 1.0;
    projection.color.b = 1.0; projection.color.a = 1.0;
    markers.markers.push_back(projection);
    if (!result.centerline.empty())
    {
      auto connector = baseMarker(frame_id, stamp, "path_connector", 0,
                                  visualization_msgs::msg::Marker::LINE_STRIP);
      connector.scale.x = 0.05;
      connector.color.r = 0.0; connector.color.g = 1.0;
      connector.color.b = 1.0; connector.color.a = 0.9;
      for (const auto& p : {result.centerline.front(), result.projection})
      {
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = 0.1;
        connector.points.push_back(gp);
      }
      markers.markers.push_back(connector);
    }
  }
  if (result.has_collision)
  {
    auto collision = baseMarker(frame_id, stamp, "path_first_collision", 0,
                                visualization_msgs::msg::Marker::SPHERE);
    collision.pose.position.x = result.collision_point.x();
    collision.pose.position.y = result.collision_point.y();
    collision.pose.position.z = 0.18;
    collision.scale.x = collision.scale.y = collision.scale.z = 0.22;
    collision.color.r = 1.0; collision.color.g = 0.0;
    collision.color.b = 0.0; collision.color.a = 1.0;
    markers.markers.push_back(collision);
  }
  if (!result.centerline.empty() && result.stopping_distance > 0.0 &&
      std::isfinite(result.stopping_distance))
  {
    double remaining = result.stopping_distance;
    for (size_t i = 1; i < result.centerline.size(); ++i)
    {
      const Eigen::Vector2d delta = result.centerline[i] - result.centerline[i - 1];
      const double length = delta.norm();
      if (length < 1e-9) continue;
      if (remaining <= length)
      {
        const Eigen::Vector2d point = result.centerline[i - 1] + remaining / length * delta;
        auto limit = baseMarker(frame_id, stamp, "path_braking_limit", 0,
                                visualization_msgs::msg::Marker::SPHERE);
        limit.pose.position.x = point.x();
        limit.pose.position.y = point.y();
        limit.pose.position.z = 0.2;
        limit.scale.x = limit.scale.y = limit.scale.z = 0.16;
        limit.color.r = 1.0; limit.color.g = 1.0;
        limit.color.b = 0.0; limit.color.a = 1.0;
        markers.markers.push_back(limit);
        break;
      }
      remaining -= length;
    }
  }

  int hull_id = 0;
  for (const PathWaitObstacle& obst : result.blocking_obstacles)
  {
    visualization_msgs::msg::Marker hull = baseMarker(
        frame_id, stamp, "blocking", hull_id++, visualization_msgs::msg::Marker::LINE_STRIP);
    hull.scale.x = 0.05;
    hull.color.r = 1.0; hull.color.g = 0.0; hull.color.b = 0.8; hull.color.a = 1.0;
    if (obst.polygon.size() >= 2)
    {
      for (const Eigen::Vector2d& p : obst.polygon)
      {
        geometry_msgs::msg::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.08;
        hull.points.push_back(gp);
      }
      geometry_msgs::msg::Point gp;
      gp.x = obst.polygon.front().x();
      gp.y = obst.polygon.front().y();
      gp.z = 0.08;
      hull.points.push_back(gp);
    }
    else
    {
      hull.type = visualization_msgs::msg::Marker::CYLINDER;
      hull.pose.position.x = obst.center.x();
      hull.pose.position.y = obst.center.y();
      hull.pose.position.z = 0.08;
      const double d = std::max(0.15, obst.radius * 2.0);
      hull.scale.x = d;
      hull.scale.y = d;
      hull.scale.z = 0.05;
    }
    markers.markers.push_back(hull);
  }

  int prediction_id = 0;
  for (const PathWaitObstacle& obst : result.predicted_blocking_obstacles)
  {
    if (obst.polygon.size() < 2) continue;
    auto predicted = baseMarker(frame_id, stamp, "blocking_motion_envelope", prediction_id++,
                                visualization_msgs::msg::Marker::LINE_STRIP);
    predicted.scale.x = 0.035;
    predicted.color.r = 1.0; predicted.color.g = 0.85;
    predicted.color.b = 0.0; predicted.color.a = 0.85;
    for (const auto& p : obst.polygon)
    {
      geometry_msgs::msg::Point gp;
      gp.x = p.x(); gp.y = p.y(); gp.z = 0.1;
      predicted.points.push_back(gp);
    }
    if (obst.polygon.size() >= 3)
    {
      geometry_msgs::msg::Point gp;
      gp.x = obst.polygon.front().x(); gp.y = obst.polygon.front().y(); gp.z = 0.1;
      predicted.points.push_back(gp);
    }
    markers.markers.push_back(predicted);
  }

  visualization_msgs::msg::Marker text = baseMarker(
      frame_id, stamp, "state", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  text.scale.z = 0.28;
  text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
  if (!result.centerline.empty())
  {
    text.pose.position.x = result.centerline.front().x();
    text.pose.position.y = result.centerline.front().y();
    text.pose.position.z = 0.6;
  }
  std::ostringstream ss;
  ss << pathWaitModeChinese(result.mode)
     << " 投影段=" << result.projection_segment
     << " 接触弧长=" << result.collision_arc
     << " 安全速度=" << result.safe_speed
     << " 走廊半宽=" << result.half_width
     << " 已等待=" << result.elapsed_wait_s
     << "秒 剩余=" << result.remaining_wait_s
     << "秒 " << pathWaitReasonChinese(result.reason)
     << " 预测阻挡=[" << joinPathWaitIds(result.blocking_ids) << "]"
     << " 当前阻挡=[" << joinPathWaitIds(result.observed_ids) << "]"
     << " 等待=[" << joinPathWaitIds(result.waiting_ids) << "]"
     << " 已锁定=[" << joinPathWaitIds(result.latched_ids) << "]"
     << " 待返回=[" << joinPathWaitIds(result.awaiting_return_ids) << "]";
  text.text = ss.str();
  markers.markers.push_back(text);

  if (result.mode == PathWaitMode::Wait)
  {
    visualization_msgs::msg::Marker timer = baseMarker(
        frame_id, stamp, "wait_timer", 0,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    timer.scale.z = 0.4;
    timer.color.r = 1.0; timer.color.g = 0.95; timer.color.b = 0.1; timer.color.a = 1.0;
    timer.pose.position = text.pose.position;
    timer.pose.position.z = 1.2;
    std::ostringstream timer_text;
    timer_text.setf(std::ios::fixed);
    timer_text.precision(1);
    timer_text << "WAIT ID " << joinPathWaitIds(result.waiting_ids) << "  ";
    if (result.reason == "awaiting_current_return")
      timer_text << "RETURN PENDING";
    else
      timer_text << result.elapsed_wait_s << " / "
                 << (result.elapsed_wait_s + result.remaining_wait_s) << " s";
    timer.text = timer_text.str();
    markers.markers.push_back(timer);
  }
}

}  // namespace teb_local_planner
