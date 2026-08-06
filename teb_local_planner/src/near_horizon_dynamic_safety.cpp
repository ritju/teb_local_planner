/*********************************************************************
 * Near-horizon dynamic-obstacle safety: cache, threat gate, RViz markers.
 *********************************************************************/

#include <teb_local_planner/near_horizon_dynamic_safety.h>

#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace teb_local_planner
{

namespace
{
Eigen::Vector2d toRobotFrame(const Eigen::Vector2d& p_map, const PoseSE2& robot_pose)
{
  const double c = std::cos(robot_pose.theta());
  const double s = std::sin(robot_pose.theta());
  const double dx = p_map.x() - robot_pose.x();
  const double dy = p_map.y() - robot_pose.y();
  return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
}

Eigen::Vector2d toMapFrame(const Eigen::Vector2d& p_robot, const PoseSE2& robot_pose)
{
  const double c = std::cos(robot_pose.theta());
  const double s = std::sin(robot_pose.theta());
  return Eigen::Vector2d(
      robot_pose.x() + c * p_robot.x() - s * p_robot.y(),
      robot_pose.y() + s * p_robot.x() + c * p_robot.y());
}

void fillContourFromObstacle(const Obstacle& obst, CachedDynamicObstacle& out)
{
  out.contour.clear();
  out.radius = 0.0;
  geometry_msgs::msg::Polygon poly;
  // toPolygonMsg is non-const in Obstacle API; contour extraction does not mutate state.
  const_cast<Obstacle&>(obst).toPolygonMsg(poly);
  out.contour.reserve(poly.points.size());
  for (const auto& p : poly.points)
  {
    out.contour.emplace_back(p.x, p.y);
  }
  // CircularObstacle: keep radius if single-point polygon + cast
  if (const auto* circ = dynamic_cast<const CircularObstacle*>(&obst))
  {
    out.radius = circ->radius();
  }
}

visualization_msgs::msg::Marker makeDeleteAll(const std::string& ns, const std::string& frame_id,
                                              const rclcpp::Time& stamp)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

/** Clear every namespace used by near-horizon debug (DELETEALL is per-ns in RViz). */
void appendNearHorizonDeleteAll(visualization_msgs::msg::MarkerArray& markers,
                                const std::string& frame_id,
                                const rclcpp::Time& stamp)
{
  // Empty ns: clear all markers from this publisher (RViz2).
  markers.markers.push_back(makeDeleteAll("", frame_id, stamp));
  // Also clear named namespaces explicitly for viewers that scope DELETEALL by ns.
  static const char* kNamespaces[] = {
      "robot_footprint", "roi_box", "cached_obstacle", "extrapolated_samples", "status_text"};
  for (const char* ns : kNamespaces)
  {
    markers.markers.push_back(makeDeleteAll(ns, frame_id, stamp));
  }
}
} // namespace

FootprintRoiBox computeFootprintRoiBox(const std::vector<geometry_msgs::msg::Point>& footprint_robot_frame)
{
  FootprintRoiBox box;
  if (footprint_robot_frame.empty())
  {
    // Fallback: small square so ROI still usable when footprint unavailable
    box.front_extent = box.rear_extent = box.left_extent = box.right_extent = 0.3;
    return box;
  }
  double xmin = footprint_robot_frame.front().x;
  double xmax = xmin;
  double ymin = footprint_robot_frame.front().y;
  double ymax = ymin;
  for (const auto& p : footprint_robot_frame)
  {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  box.front_extent = std::max(0.0, xmax);
  box.rear_extent = std::max(0.0, -xmin);
  box.left_extent = std::max(0.0, ymax);
  box.right_extent = std::max(0.0, -ymin);
  return box;
}

void DynamicObstacleCache::update(const ObstContainer& obstacles, const rclcpp::Time& now)
{
  // 1) Ingest fresh dynamic obstacles
  std::vector<CachedDynamicObstacle> fresh;
  fresh.reserve(obstacles.size());
  for (const ObstaclePtr& obst : obstacles)
  {
    if (!obst || !obst->isDynamic())
    {
      continue;
    }
    CachedDynamicObstacle c;
    c.centroid = obst->getCentroid();
    c.velocity = obst->getCentroidVelocity();
    c.last_seen = now;
    c.from_cache = false;
    fillContourFromObstacle(*obst, c);

    // Nearest-neighbor associate to previous cache for stable ids
    int best_id = -1;
    double best_d2 = 0.25 * 0.25; // 0.25 m association gate
    for (const auto& prev : cached_)
    {
      const double d2 = (prev.centroid - c.centroid).squaredNorm();
      if (d2 < best_d2)
      {
        best_d2 = d2;
        best_id = prev.id;
      }
    }
    c.id = (best_id >= 0) ? best_id : next_synthetic_id_++;
    fresh.push_back(c);
  }

  // 2) Coast previous entries not refreshed this cycle (within cache_time)
  for (const auto& prev : cached_)
  {
    bool refreshed = false;
    for (const auto& f : fresh)
    {
      if (f.id == prev.id)
      {
        refreshed = true;
        break;
      }
    }
    if (refreshed)
    {
      continue;
    }
    const double age = (now - prev.last_seen).seconds();
    if (age <= cache_time_sec_)
    {
      CachedDynamicObstacle coast = prev;
      coast.from_cache = true;
      fresh.push_back(coast);
    }
  }

  cached_.swap(fresh);
}

Eigen::Vector2d NearHorizonThreatGate::clampVelocity(const Eigen::Vector2d& v, double v_x_max) const
{
  // Planar obstacles only expose (vx, vy). Clamp Euclidean speed by v_x_max.
  // dyn_gate_omega_z_max is reserved for future oriented obstacles; still applied
  // as an extra speed scale if omega-like lateral component is huge (no-op for now
  // beyond speed clamp documentation in config).
  const double vmax = std::max(0.0, v_x_max);
  const double n = v.norm();
  if (n < 1e-9 || n <= vmax)
  {
    return v;
  }
  return v * (vmax / n);
}

bool NearHorizonThreatGate::pointInRoiRobotFrame(
    const Eigen::Vector2d& p_robot, const FootprintRoiBox& fp,
    double roi_front, double roi_rear, double roi_left, double roi_right,
    double inflation) const
{
  // Expand ROI by obstacle inflation so contour thickness is considered
  const double xmin = fp.roiXMin(roi_rear) - inflation;
  const double xmax = fp.roiXMax(roi_front) + inflation;
  const double ymin = fp.roiYMin(roi_right) - inflation;
  const double ymax = fp.roiYMax(roi_left) + inflation;
  return p_robot.x() >= xmin && p_robot.x() <= xmax &&
         p_robot.y() >= ymin && p_robot.y() <= ymax;
}

NearHorizonThreatResult NearHorizonThreatGate::evaluate(
    const PoseSE2& robot_pose,
    const RobotFootprintModelConstPtr& robot_model,
    const FootprintRoiBox& footprint_roi,
    const std::vector<CachedDynamicObstacle>& cached,
    const TebConfig& cfg,
    const rclcpp::Time& now)
{
  NearHorizonThreatResult result;
  last_samples_.clear();

  if (!cfg.obstacles.dynamic_safety_predictable_mode)
  {
    result.reason = "safety_mode_off";
    enable_latched_ = false;
    last_result_ = result;
    return result;
  }

  const double T_near = cfg.obstacles.dyn_gate_T_near;
  const double dt = 0.1;
  const double v_x_max = cfg.obstacles.dyn_gate_v_x_max;
  const double inflation = cfg.obstacles.dyn_gate_inflation;
  const double overlap_dist = cfg.obstacles.dyn_gate_overlap_dist;
  const double roi_front = cfg.obstacles.dyn_gate_roi_front;
  const double roi_rear = cfg.obstacles.dyn_gate_roi_rear;
  const double roi_left = cfg.obstacles.dyn_gate_roi_left;
  const double roi_right = cfg.obstacles.dyn_gate_roi_right;
  const double hold_time = cfg.obstacles.dyn_gate_hold_time;

  bool any_threat = false;
  bool any_overlap = false;

  for (const auto& c : cached)
  {
    // Build a temporary point obstacle at current centroid for overlap distance
    PointObstacle probe(c.centroid);
    double dist0 = 1e9;
    if (robot_model)
    {
      dist0 = robot_model->calculateDistance(robot_pose, &probe);
      // Account for circular radius if present
      dist0 = std::max(0.0, dist0 - c.radius);
    }
    else
    {
      dist0 = (c.centroid - Eigen::Vector2d(robot_pose.x(), robot_pose.y())).norm();
    }

    if (dist0 < overlap_dist)
    {
      // Velocity likely wrong / already intersecting: do NOT use OmegaHold for this object
      any_overlap = true;
      continue;
    }

    const Eigen::Vector2d v_clamped = clampVelocity(c.velocity, v_x_max);
    bool obst_threat = false;
    double min_d = dist0;

    for (double t = 0.0; t <= T_near + 1e-9; t += dt)
    {
      Eigen::Vector2d pred = c.centroid + v_clamped * t;
      last_samples_.push_back(pred);

      // Contour points: translate stored contour, else use centroid (+ radius samples optional)
      std::vector<Eigen::Vector2d> pts;
      if (!c.contour.empty())
      {
        const Eigen::Vector2d offset = pred - c.centroid;
        pts.reserve(c.contour.size());
        for (const auto& v : c.contour)
        {
          pts.push_back(v + offset);
        }
      }
      else
      {
        pts.push_back(pred);
      }

      for (const auto& p_map : pts)
      {
        const Eigen::Vector2d p_robot = toRobotFrame(p_map, robot_pose);
        if (pointInRoiRobotFrame(p_robot, footprint_roi, roi_front, roi_rear, roi_left, roi_right,
                                 inflation + c.radius))
        {
          obst_threat = true;
          min_d = std::min(min_d, dist0);
          break;
        }
      }
      if (obst_threat)
      {
        break;
      }
    }

    if (obst_threat)
    {
      any_threat = true;
      if (min_d < result.trigger_min_dist)
      {
        result.trigger_min_dist = min_d;
        result.trigger_obstacle_id = c.id;
      }
    }
  }

  result.raw_threat = any_threat;
  result.excluded_overlap = any_overlap;

  // No cached dynamic obstacles => cannot be a near-horizon threat; drop latch immediately
  // (hold_time only bridges brief gate flaps while obstacles are still present).
  if (cached.empty())
  {
    enable_latched_ = false;
    has_last_raw_threat_time_ = false;
  }
  else if (any_threat)
  {
    last_raw_threat_time_ = now;
    has_last_raw_threat_time_ = true;
    enable_latched_ = true;
  }
  else if (enable_latched_ && has_last_raw_threat_time_)
  {
    // Time hysteresis: keep enable for hold_time after last raw threat
    if ((now - last_raw_threat_time_).seconds() > hold_time)
    {
      enable_latched_ = false;
    }
  }
  else
  {
    enable_latched_ = false;
  }

  result.enable_omega_hold = enable_latched_;
  if (any_overlap && !any_threat)
  {
    result.reason = "overlap_excluded_static_only";
  }
  else if (result.enable_omega_hold)
  {
    result.reason = any_threat ? "roi_threat" : "hold_time_latched";
  }
  else
  {
    result.reason = "no_threat";
  }

  last_result_ = result;
  return result;
}

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
    const std::vector<geometry_msgs::msg::Point>& footprint_robot_frame)
{
  markers.markers.clear();
  // IMPORTANT: DELETEALL is namespace-scoped. Previously only ns="nh_clear" was
  // deleted, so cached_obstacle / extrapolated_samples kept showing forever in
  // RViz after dynamic obstacles stopped publishing.
  appendNearHorizonDeleteAll(markers, frame_id, stamp);

  auto color = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  };

  // Stable ids per namespace so ADD replaces previous markers reliably.
  int footprint_id = 0;
  int roi_id = 0;
  int cached_id = 0;
  int sample_id = 0;
  int text_id = 0;

  // --- Robot footprint ---
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "robot_footprint";
    m.id = footprint_id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.scale.x = 0.03;
    m.color = threat.enable_omega_hold ? color(1.f, 0.4f, 0.f, 0.95f) : color(0.1f, 0.8f, 0.2f, 0.9f);
    m.pose.orientation.w = 1.0;
    for (const auto& pr : footprint_robot_frame)
    {
      Eigen::Vector2d pm = toMapFrame(Eigen::Vector2d(pr.x, pr.y), robot_pose);
      geometry_msgs::msg::Point gp;
      gp.x = pm.x();
      gp.y = pm.y();
      gp.z = 0.05;
      m.points.push_back(gp);
    }
    if (!m.points.empty())
    {
      m.points.push_back(m.points.front());
    }
    markers.markers.push_back(m);
  }

  // --- ROI box ---
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "roi_box";
    m.id = roi_id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.scale.x = 0.025;
    m.color = color(0.2f, 0.5f, 1.f, 0.85f);
    m.pose.orientation.w = 1.0;
    const double xmin = footprint_roi.roiXMin(cfg.obstacles.dyn_gate_roi_rear);
    const double xmax = footprint_roi.roiXMax(cfg.obstacles.dyn_gate_roi_front);
    const double ymin = footprint_roi.roiYMin(cfg.obstacles.dyn_gate_roi_right);
    const double ymax = footprint_roi.roiYMax(cfg.obstacles.dyn_gate_roi_left);
    const Eigen::Vector2d corners_r[5] = {
        {xmax, ymax}, {xmax, ymin}, {xmin, ymin}, {xmin, ymax}, {xmax, ymax}};
    for (const auto& cr : corners_r)
    {
      Eigen::Vector2d pm = toMapFrame(cr, robot_pose);
      geometry_msgs::msg::Point gp;
      gp.x = pm.x();
      gp.y = pm.y();
      gp.z = 0.06;
      m.points.push_back(gp);
    }
    markers.markers.push_back(m);
  }

  // --- Cached obstacles (current pose) ---
  for (const auto& c : cached)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "cached_obstacle";
    m.id = cached_id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.scale.x = 0.03;
    const bool is_trigger = (c.id == threat.trigger_obstacle_id);
    m.color = is_trigger ? color(1.f, 0.f, 0.f, 0.95f)
                         : (c.from_cache ? color(1.f, 1.f, 0.2f, 0.7f) : color(1.f, 0.6f, 0.1f, 0.8f));
    m.pose.orientation.w = 1.0;
    if (!c.contour.empty())
    {
      for (const auto& v : c.contour)
      {
        geometry_msgs::msg::Point gp;
        gp.x = v.x();
        gp.y = v.y();
        gp.z = 0.07;
        m.points.push_back(gp);
      }
      m.points.push_back(m.points.front());
    }
    else
    {
      // Point / circle as small diamond
      const double r = std::max(0.05, c.radius);
      const Eigen::Vector2d ctr = c.centroid;
      const Eigen::Vector2d d[5] = {
          {ctr.x() + r, ctr.y()}, {ctr.x(), ctr.y() + r},
          {ctr.x() - r, ctr.y()}, {ctr.x(), ctr.y() - r},
          {ctr.x() + r, ctr.y()}};
      for (const auto& p : d)
      {
        geometry_msgs::msg::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = 0.07;
        m.points.push_back(gp);
      }
    }
    markers.markers.push_back(m);
  }

  // --- Extrapolated samples (only while cache still has dynamic obstacles) ---
  if (!extrapolated_samples.empty() && !cached.empty())
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "extrapolated_samples";
    m.id = sample_id++;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.scale.x = m.scale.y = m.scale.z = 0.08;
    m.color = color(0.6f, 0.2f, 1.f, 0.45f);
    m.pose.orientation.w = 1.0;
    for (const auto& p : extrapolated_samples)
    {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = 0.08;
      m.points.push_back(gp);
    }
    markers.markers.push_back(m);
  }

  // --- Status text ---
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "status_text";
    m.id = text_id++;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.scale.z = 0.25;
    m.color = color(1.f, 1.f, 1.f, 0.95f);
    m.pose.position.x = robot_pose.x();
    m.pose.position.y = robot_pose.y();
    m.pose.position.z = 0.8;
    m.pose.orientation.w = 1.0;
    const double delta =
        cfg.obstacles.delta_omega_scale * cfg.robot.max_vel_theta;
    std::ostringstream ss;
    ss << "OmegaHold=" << (threat.enable_omega_hold ? "ON" : "OFF")
       << " reason=" << threat.reason
       << " cache=" << cached.size()
       << " trig_id=" << threat.trigger_obstacle_id
       << " d=" << threat.trigger_min_dist
       << " dw=" << delta;
    m.text = ss.str();
    markers.markers.push_back(m);
  }
}

} // namespace teb_local_planner
