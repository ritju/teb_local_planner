/*********************************************************************
 * PATH-corridor wait / detour gate for confirmed dynamic obstacles.
 *********************************************************************/

#include <teb_local_planner/path_dynamic_wait.h>

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

  const double r = std::max(0.0, obst.radius);
  return distPointToPolyline(obst.center, centerline) <= (half_width + r);
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

std::vector<Eigen::Vector2d> samplePathWaitCenterline(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const Eigen::Vector2d& robot_xy,
    double lookahead_dist)
{
  std::vector<Eigen::Vector2d> out;
  if (global_plan.empty() || lookahead_dist <= 0.0)
  {
    return out;
  }

  int closest = 0;
  double best_sq = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(global_plan.size()); ++i)
  {
    const double dx = global_plan[i].pose.position.x - robot_xy.x();
    const double dy = global_plan[i].pose.position.y - robot_xy.y();
    const double sq = dx * dx + dy * dy;
    if (sq < best_sq)
    {
      best_sq = sq;
      closest = i;
    }
  }

  out.emplace_back(global_plan[closest].pose.position.x, global_plan[closest].pose.position.y);
  double acc = 0.0;
  for (int i = closest; i + 1 < static_cast<int>(global_plan.size()); ++i)
  {
    const Eigen::Vector2d a(global_plan[i].pose.position.x, global_plan[i].pose.position.y);
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
  }
  return out;
}

void PathDynamicWaitGate::clearIdClocks()
{
  latched_ids_.clear();
  wait_enter_.clear();
  absent_since_.clear();
}

void PathDynamicWaitGate::updatePerIdClocks(
    const std::unordered_set<int>& blocking,
    const rclcpp::Time& now,
    double t_clear)
{
  for (int id : blocking)
  {
    absent_since_.erase(id);
    if (latched_ids_.count(id) == 0 && wait_enter_.count(id) == 0)
    {
      wait_enter_[id] = now;
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
    if (blocking.count(id) == 0 && absent_since_.count(id) == 0)
    {
      absent_since_[id] = now;
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
    wait_enter_.erase(prune[i]);
    absent_since_.erase(prune[i]);
    latched_ids_.erase(prune[i]);
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
    bool message_valid)
{
  PathWaitResult result;
  result.previous_mode = mode_;
  const double scale = std::max(0.0, cfg.obstacles.path_wait_corridor_scale);
  result.half_width = std::max(0.0, footprint_half_width) +
                      std::max(0.0, min_obstacle_dist) * scale;
  result.centerline = samplePathWaitCenterline(
      global_plan, robot_xy, cfg.obstacles.path_wait_lookahead_dist);
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
  if (message_valid)
  {
    for (const PathWaitObstacle& obst : obstacles)
    {
      if (obstacleHitsCorridor(obst, result.centerline, result.half_width))
      {
        blocking.insert(obst.id);
        result.blocking_obstacles.push_back(obst);
      }
    }
  }

  result.blocking_ids.assign(blocking.begin(), blocking.end());
  std::sort(result.blocking_ids.begin(), result.blocking_ids.end());

  const double t_clear = std::max(0.0, cfg.obstacles.path_wait_clear_time);
  const double t_wait = std::max(0.0, cfg.obstacles.path_wait_timeout);
  updatePerIdClocks(blocking, now, t_clear);

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
      clearIdClocks();
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
      mode_ = PathWaitMode::Detour;
      result.reason = latched_now ? "wait_timeout" : "latched_detour";
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
  ss << pathWaitModeName(result.mode)
     << " hw=" << result.half_width
     << " wait=" << result.elapsed_wait_s
     << "s rem=" << result.remaining_wait_s
     << "s " << result.reason
     << " block=[" << joinPathWaitIds(result.blocking_ids) << "]"
     << " waiting=[" << joinPathWaitIds(result.waiting_ids) << "]"
     << " latch=[" << joinPathWaitIds(result.latched_ids) << "]";
  text.text = ss.str();
  markers.markers.push_back(text);
}

}  // namespace teb_local_planner
