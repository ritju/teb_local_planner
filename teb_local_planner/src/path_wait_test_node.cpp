/*********************************************************************
 * PATH-wait + TEB optimization demo.
 * Gated TEB omits on-path dynamics while WAIT (should stay on PATH).
 * Naive TEB always includes dynamics (shows being pulled away).
 *********************************************************************/

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/path_dynamic_wait.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/teb_config.h"

#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::Point32 pt32(double x, double y)
{
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = 0.0f;
  return p;
}

costmap_converter_msgs::msg::ObstacleMsg makeBoxObstacle(
  int id, double cx, double cy, double yaw, double length, double width)
{
  costmap_converter_msgs::msg::ObstacleMsg msg;
  msg.id = id;
  msg.radius = 0.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hl = 0.5 * length;
  const double hw = 0.5 * width;
  const double local[4][2] = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}};
  for (int i = 0; i < 4; ++i)
  {
    msg.polygon.points.push_back(pt32(
      cx + c * local[i][0] - s * local[i][1],
      cy + s * local[i][0] + c * local[i][1]));
  }
  return msg;
}

teb_local_planner::PathWaitObstacle toGateObstacle(
  const costmap_converter_msgs::msg::ObstacleMsg & src)
{
  teb_local_planner::PathWaitObstacle dst;
  dst.id = static_cast<int>(src.id);
  Eigen::Vector2d acc = Eigen::Vector2d::Zero();
  for (const auto & p : src.polygon.points)
  {
    const Eigen::Vector2d xy(p.x, p.y);
    dst.polygon.push_back(xy);
    acc += xy;
  }
  if (!dst.polygon.empty())
  {
    dst.center = acc / static_cast<double>(dst.polygon.size());
  }
  dst.radius = src.radius;
  return dst;
}

teb_local_planner::ObstaclePtr toPolygonObstacle(
  const costmap_converter_msgs::msg::ObstacleMsg & src)
{
  auto poly = std::make_shared<teb_local_planner::PolygonObstacle>();
  for (const auto & p : src.polygon.points)
  {
    poly->pushBackVertex(static_cast<double>(p.x), static_cast<double>(p.y));
  }
  poly->finalizePolygon();
  return poly;
}

visualization_msgs::msg::Marker hullMarker(
  const std::string & frame, const rclcpp::Time & stamp,
  const std::string & ns, int id,
  const costmap_converter_msgs::msg::ObstacleMsg & obst,
  float r, float g, float b)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.05;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;
  m.lifetime = rclcpp::Duration::from_seconds(0.35);
  for (const auto & p : obst.polygon.points)
  {
    geometry_msgs::msg::Point gp;
    gp.x = p.x;
    gp.y = p.y;
    gp.z = 0.12;
    m.points.push_back(gp);
  }
  if (!obst.polygon.points.empty())
  {
    geometry_msgs::msg::Point gp;
    gp.x = obst.polygon.points.front().x;
    gp.y = obst.polygon.points.front().y;
    gp.z = 0.12;
    m.points.push_back(gp);
  }
  return m;
}

double distToPolyline(double x, double y, const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  if (path.empty())
  {
    return 0.0;
  }
  if (path.size() == 1)
  {
    const double dx = x - path.front().pose.position.x;
    const double dy = y - path.front().pose.position.y;
    return std::hypot(dx, dy);
  }
  double best = 1e9;
  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    const double ax = path[i].pose.position.x;
    const double ay = path[i].pose.position.y;
    const double bx = path[i + 1].pose.position.x;
    const double by = path[i + 1].pose.position.y;
    const double vx = bx - ax;
    const double vy = by - ay;
    const double len2 = vx * vx + vy * vy;
    double t = 0.0;
    if (len2 > 1e-9)
    {
      t = ((x - ax) * vx + (y - ay) * vy) / len2;
      t = std::max(0.0, std::min(1.0, t));
    }
    best = std::min(best, std::hypot(x - (ax + t * vx), y - (ay + t * vy)));
  }
  return best;
}

}  // namespace

struct SimObstacle
{
  int id = 0;
  bool is_dynamic = true;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double length = 0.45;
  double width = 0.35;

  costmap_converter_msgs::msg::ObstacleMsg toMsg() const
  {
    return makeBoxObstacle(id, x, y, yaw, length, width);
  }
};

class PathWaitTestNode : public nav2_util::LifecycleNode
{
public:
  PathWaitTestNode()
  : nav2_util::LifecycleNode("path_wait_test_node")
  {
    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "base_link");
    declare_parameter("footprint_radius", 0.17);
    declare_parameter("min_obstacle_dist", 0.27);
    declare_parameter("path_wait_lookahead_dist", 6.0);
    declare_parameter("path_wait_corridor_scale", 1.0);
    declare_parameter("path_wait_timeout", 4.0);
    declare_parameter("path_wait_clear_time", 0.5);
    declare_parameter("path_length", 10.0);
    declare_parameter("path_step", 0.2);
    declare_parameter("num_dynamic", 3);
    declare_parameter("num_static", 2);
    declare_parameter("spawn_x_min", 1.2);
    declare_parameter("spawn_x_max", 6.0);
    declare_parameter("spawn_y_min", -1.6);
    declare_parameter("spawn_y_max", 1.6);
    declare_parameter("speed_min", 0.05);
    declare_parameter("speed_max", 0.45);
    declare_parameter("resample_period", 8.0);
    declare_parameter("seed", 1);
    declare_parameter("pulled_dev_threshold", 0.25);

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    footprint_radius_ = get_parameter("footprint_radius").as_double();
    min_obstacle_dist_ = get_parameter("min_obstacle_dist").as_double();
    spawn_x_min_ = get_parameter("spawn_x_min").as_double();
    spawn_x_max_ = get_parameter("spawn_x_max").as_double();
    spawn_y_min_ = get_parameter("spawn_y_min").as_double();
    spawn_y_max_ = get_parameter("spawn_y_max").as_double();
    speed_min_ = get_parameter("speed_min").as_double();
    speed_max_ = get_parameter("speed_max").as_double();
    resample_period_ = get_parameter("resample_period").as_double();
    pulled_dev_threshold_ = get_parameter("pulled_dev_threshold").as_double();
    rng_.seed(static_cast<uint32_t>(get_parameter("seed").as_int()));

    cfg_.map_frame = map_frame_;
    cfg_.robot_model = std::make_shared<teb_local_planner::CircularRobotFootprint>(footprint_radius_);
    cfg_.obstacles.path_wait_enable = true;
    cfg_.obstacles.path_wait_lookahead_dist =
      get_parameter("path_wait_lookahead_dist").as_double();
    cfg_.obstacles.path_wait_corridor_scale =
      get_parameter("path_wait_corridor_scale").as_double();
    cfg_.obstacles.path_wait_timeout = get_parameter("path_wait_timeout").as_double();
    cfg_.obstacles.path_wait_clear_time =
      get_parameter("path_wait_clear_time").as_double();
    cfg_.obstacles.min_obstacle_dist = min_obstacle_dist_;
    cfg_.obstacles.inflation_dist = 0.6;
    cfg_.obstacles.include_dynamic_obstacles = false;
    cfg_.obstacles.dynamic_safety_predictable_mode = true;
    cfg_.hcp.enable_homotopy_class_planning = false;
    cfg_.optim.weight_obstacle = 100.0;
    cfg_.optim.weight_viapoint = 15.0;
    cfg_.optim.no_inner_iterations = 4;
    cfg_.optim.no_outer_iterations = 3;
    cfg_.robot.max_vel_x = 0.4;
    cfg_.robot.max_vel_theta = 1.0;
    cfg_.robot.allow_backward_velocity = true;

    buildPath(
      get_parameter("path_length").as_double(),
      get_parameter("path_step").as_double());
    spawnAll(
      static_cast<int>(get_parameter("num_dynamic").as_int()),
      static_cast<int>(get_parameter("num_static").as_int()));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    dyn_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "/dynamic_obstacles", rclcpp::QoS(1).reliable());
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path_wait_test/global_plan", rclcpp::QoS(1).reliable());
    gated_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path_wait_test/teb_gated", rclcpp::QoS(1).reliable());
    naive_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path_wait_test/teb_naive", rclcpp::QoS(1).reliable());
    debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/teb_path_wait_debug", rclcpp::QoS(1).reliable());
    scene_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_wait_test/scene_markers", rclcpp::QoS(1).reliable());
    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/path_wait_test/footprint", rclcpp::QoS(1).reliable());

    start_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&PathWaitTestNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "path_wait_test: dyn=%ld static=%ld spawn x[%.1f,%.1f] y[%.1f,%.1f] "
      "T_wait=%.1fs hw≈%.3f m. 青=门控TEB  品红=无门控TEB(会被动态障碍带走)",
      static_cast<long>(get_parameter("num_dynamic").as_int()),
      static_cast<long>(get_parameter("num_static").as_int()),
      spawn_x_min_, spawn_x_max_, spawn_y_min_, spawn_y_max_,
      cfg_.obstacles.path_wait_timeout,
      footprint_radius_ + min_obstacle_dist_ * cfg_.obstacles.path_wait_corridor_scale);
  }

private:
  void ensurePlanners()
  {
    if (planner_gated_)
    {
      return;
    }
    const auto self = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(shared_from_this());
    planner_gated_ = std::make_unique<teb_local_planner::TebOptimalPlanner>(
      self, cfg_, &obst_gated_, teb_local_planner::TebVisualizationPtr(), &via_points_);
    planner_naive_ = std::make_unique<teb_local_planner::TebOptimalPlanner>(
      self, cfg_, &obst_naive_, teb_local_planner::TebVisualizationPtr(), &via_points_);
  }

  void buildPath(double length, double step)
  {
    global_plan_.clear();
    via_points_.clear();
    const int n = std::max(2, static_cast<int>(std::ceil(length / std::max(0.05, step))) + 1);
    for (int i = 0; i < n; ++i)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = map_frame_;
      p.pose.position.x = i * (length / (n - 1));
      p.pose.position.y = 0.0;
      p.pose.orientation.w = 1.0;
      global_plan_.push_back(p);
      via_points_.emplace_back(p.pose.position.x, p.pose.position.y);
    }
  }

  double urand(double lo, double hi)
  {
    std::uniform_real_distribution<double> d(lo, hi);
    return d(rng_);
  }

  void randomizeMotion(SimObstacle & o)
  {
    const double speed = urand(speed_min_, speed_max_);
    const double yaw = urand(-M_PI, M_PI);
    o.yaw = yaw;
    o.vx = speed * std::cos(yaw);
    o.vy = speed * std::sin(yaw);
  }

  SimObstacle spawnOne(int id, bool is_dynamic)
  {
    SimObstacle o;
    o.id = id;
    o.is_dynamic = is_dynamic;
    o.length = is_dynamic ? urand(0.35, 0.55) : urand(0.30, 0.50);
    o.width = is_dynamic ? urand(0.28, 0.42) : urand(0.30, 0.45);
    o.x = urand(spawn_x_min_, spawn_x_max_);
    o.y = urand(spawn_y_min_, spawn_y_max_);
    if (is_dynamic)
    {
      randomizeMotion(o);
    }
    else
    {
      o.vx = 0.0;
      o.vy = 0.0;
      o.yaw = urand(-0.4, 0.4);
    }
    return o;
  }

  void spawnAll(int n_dyn, int n_stat)
  {
    obstacles_.clear();
    n_dyn = std::max(0, n_dyn);
    n_stat = std::max(0, n_stat);
    for (int i = 0; i < n_dyn; ++i)
    {
      obstacles_.push_back(spawnOne(100 + i, true));
    }
    for (int i = 0; i < n_stat; ++i)
    {
      obstacles_.push_back(spawnOne(200 + i, false));
    }
  }

  void stepObstacles(double dt)
  {
    for (SimObstacle & o : obstacles_)
    {
      if (!o.is_dynamic)
      {
        continue;
      }
      o.x += o.vx * dt;
      o.y += o.vy * dt;
      if (o.x < spawn_x_min_ || o.x > spawn_x_max_)
      {
        o.vx = -o.vx;
        o.x = std::max(spawn_x_min_, std::min(spawn_x_max_, o.x));
      }
      if (o.y < spawn_y_min_ || o.y > spawn_y_max_)
      {
        o.vy = -o.vy;
        o.y = std::max(spawn_y_min_, std::min(spawn_y_max_, o.y));
      }
      o.yaw = std::atan2(o.vy, o.vx);
    }
  }

  nav_msgs::msg::Path tebToPath(
    const teb_local_planner::TebOptimalPlanner & planner, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    const teb_local_planner::TimedElasticBand & teb = planner.teb();
    for (int i = 0; i < teb.sizePoses(); ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      teb.Pose(i).toPoseMsg(ps.pose);
      path.poses.push_back(ps);
    }
    return path;
  }

  double maxLateralDev(const nav_msgs::msg::Path & teb_path) const
  {
    double m = 0.0;
    for (const auto & p : teb_path.poses)
    {
      m = std::max(m, distToPolyline(p.pose.position.x, p.pose.position.y, global_plan_));
    }
    return m;
  }

  void onTimer()
  {
    ensurePlanners();
    const rclcpp::Time stamp = now();
    const double t = (stamp - start_).seconds();
    const double dt = 0.2;
    stepObstacles(dt);
    if (resample_period_ > 1e-3 && t - last_resample_t_ >= resample_period_)
    {
      last_resample_t_ = t;
      for (SimObstacle & o : obstacles_)
      {
        if (o.is_dynamic)
        {
          randomizeMotion(o);
        }
      }
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = map_frame_;
    for (auto p : global_plan_)
    {
      p.header.stamp = stamp;
      path_msg.poses.push_back(p);
    }
    plan_pub_->publish(path_msg);

    std::vector<costmap_converter_msgs::msg::ObstacleMsg> dyn_msgs;
    std::vector<costmap_converter_msgs::msg::ObstacleMsg> static_msgs;
    for (const SimObstacle & o : obstacles_)
    {
      const auto msg = o.toMsg();
      if (o.is_dynamic)
      {
        dyn_msgs.push_back(msg);
      }
      else
      {
        static_msgs.push_back(msg);
      }
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg arr;
    arr.header.stamp = stamp;
    arr.header.frame_id = map_frame_;
    arr.obstacles = dyn_msgs;
    dyn_pub_->publish(arr);

    std::vector<teb_local_planner::PathWaitObstacle> gate_obsts;
    for (const auto & o : dyn_msgs)
    {
      gate_obsts.push_back(toGateObstacle(o));
    }
    const teb_local_planner::PathWaitResult result = gate_.evaluate(
      global_plan_, Eigen::Vector2d(0.0, 0.0), gate_obsts,
      footprint_radius_, min_obstacle_dist_, cfg_, stamp, true);

    visualization_msgs::msg::MarkerArray debug;
    teb_local_planner::buildPathWaitDebugMarkers(debug, map_frame_, stamp, result);
    debug_pub_->publish(debug);

    obst_naive_.clear();
    obst_gated_.clear();
    for (const auto & s : static_msgs)
    {
      obst_naive_.push_back(toPolygonObstacle(s));
      obst_gated_.push_back(toPolygonObstacle(s));
    }
    for (const auto & d : dyn_msgs)
    {
      obst_naive_.push_back(toPolygonObstacle(d));
    }
    if (result.mode == teb_local_planner::PathWaitMode::Detour)
    {
      for (const auto & d : dyn_msgs)
      {
        if (std::find(result.latched_ids.begin(), result.latched_ids.end(),
                      static_cast<int>(d.id)) != result.latched_ids.end() ||
            std::find(result.blocking_ids.begin(), result.blocking_ids.end(),
                      static_cast<int>(d.id)) != result.blocking_ids.end())
        {
          obst_gated_.push_back(toPolygonObstacle(d));
        }
      }
    }

    if (result.mode == teb_local_planner::PathWaitMode::Wait &&
        result.previous_mode != teb_local_planner::PathWaitMode::Wait)
    {
      planner_gated_->clearPlanner();
    }

    std::vector<geometry_msgs::msg::PoseStamped> init_plan = global_plan_;
    if (init_plan.size() > 40)
    {
      init_plan.resize(40);
    }
    bool gated_ok = false;
    bool naive_ok = false;
    try
    {
      gated_ok = planner_gated_->plan(init_plan);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "gated TEB: %s", ex.what());
    }
    try
    {
      naive_ok = planner_naive_->plan(init_plan);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "naive TEB: %s", ex.what());
    }

    nav_msgs::msg::Path gated_path;
    nav_msgs::msg::Path naive_path;
    if (gated_ok)
    {
      gated_path = tebToPath(*planner_gated_, stamp);
      gated_pub_->publish(gated_path);
    }
    if (naive_ok)
    {
      naive_path = tebToPath(*planner_naive_, stamp);
      naive_pub_->publish(naive_path);
    }
    const double gated_dev = maxLateralDev(gated_path);
    const double naive_dev = maxLateralDev(naive_path);
    const bool naive_pulled = naive_dev > pulled_dev_threshold_;
    const bool gated_held = gated_dev <= pulled_dev_threshold_;

    publishSceneMarkers(
      stamp, dyn_msgs, static_msgs, result, gated_dev, naive_dev, naive_pulled, gated_held);
    publishFootprint(stamp);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[PathWaitTest] gate=%s reason=%s dyn=%zu block=[%s] waiting=[%s] "
      "latch=[%s] elapsed=%.1fs rem=%.1fs | gated_dev=%.2f(%s) naive_dev=%.2f(%s)",
      teb_local_planner::pathWaitModeName(result.mode),
      result.reason.c_str(),
      dyn_msgs.size(),
      teb_local_planner::joinPathWaitIds(result.blocking_ids).c_str(),
      teb_local_planner::joinPathWaitIds(result.waiting_ids).c_str(),
      teb_local_planner::joinPathWaitIds(result.latched_ids).c_str(),
      result.elapsed_wait_s,
      result.remaining_wait_s,
      gated_dev, gated_held ? "贴PATH" : "偏离",
      naive_dev, naive_pulled ? "被带走" : "未拉偏");
  }

  void publishFootprint(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PolygonStamped poly;
    poly.header.stamp = stamp;
    poly.header.frame_id = map_frame_;
    const int n = 16;
    for (int i = 0; i < n; ++i)
    {
      const double a = 2.0 * M_PI * i / n;
      poly.polygon.points.push_back(
        pt32(footprint_radius_ * std::cos(a), footprint_radius_ * std::sin(a)));
    }
    footprint_pub_->publish(poly);
  }

  void publishSceneMarkers(
    const rclcpp::Time & stamp,
    const std::vector<costmap_converter_msgs::msg::ObstacleMsg> & dyn_msgs,
    const std::vector<costmap_converter_msgs::msg::ObstacleMsg> & static_msgs,
    const teb_local_planner::PathWaitResult & result,
    double gated_dev, double naive_dev, bool naive_pulled, bool gated_held)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = map_frame_;
    del.header.stamp = stamp;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int i = 0;
    for (const auto & o : dyn_msgs)
    {
      arr.markers.push_back(hullMarker(map_frame_, stamp, "dynamic_hull", i++, o, 1.0f, 0.2f, 0.2f));
    }
    i = 0;
    for (const auto & o : static_msgs)
    {
      arr.markers.push_back(hullMarker(map_frame_, stamp, "static_hull", i++, o, 0.2f, 0.85f, 0.2f));
    }

    visualization_msgs::msg::Marker box;
    box.header.frame_id = map_frame_;
    box.header.stamp = stamp;
    box.ns = "spawn_box";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.orientation.w = 1.0;
    box.scale.x = 0.02;
    box.color.r = 0.7; box.color.g = 0.7; box.color.b = 0.7; box.color.a = 0.6;
    box.lifetime = rclcpp::Duration::from_seconds(0.35);
    const double xs[5] = {spawn_x_min_, spawn_x_max_, spawn_x_max_, spawn_x_min_, spawn_x_min_};
    const double ys[5] = {spawn_y_min_, spawn_y_min_, spawn_y_max_, spawn_y_max_, spawn_y_min_};
    for (int k = 0; k < 5; ++k)
    {
      geometry_msgs::msg::Point gp;
      gp.x = xs[k]; gp.y = ys[k]; gp.z = 0.02;
      box.points.push_back(gp);
    }
    arr.markers.push_back(box);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = map_frame_;
    text.header.stamp = stamp;
    text.ns = "legend";
    text.id = 0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 3.0;
    text.pose.position.y = -2.3;
    text.pose.position.z = 0.8;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.28;
    text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
    text.lifetime = rclcpp::Duration::from_seconds(0.35);
    std::ostringstream ss;
    ss << teb_local_planner::pathWaitModeName(result.mode)
       << " dyn=" << dyn_msgs.size() << " static=" << static_msgs.size()
       << " block=" << result.blocking_ids.size();
    text.text = ss.str();
    arr.markers.push_back(text);

    visualization_msgs::msg::Marker hint = text;
    hint.id = 1;
    hint.pose.position.y = -2.75;
    hint.scale.z = 0.22;
    std::ostringstream ss2;
    ss2 << "门控偏=" << gated_dev << (gated_held ? "m 贴PATH" : "m 偏离")
        << "  无门控偏=" << naive_dev << (naive_pulled ? "m 被带走" : "m 未拉偏");
    hint.text = ss2.str();
    arr.markers.push_back(hint);

    visualization_msgs::msg::Marker hint2 = text;
    hint2.id = 2;
    hint2.pose.position.y = -3.15;
    hint2.scale.z = 0.20;
    hint2.text = "青=门控TEB  品红=无门控TEB  红框=动态  绿框=静物";
    arr.markers.push_back(hint2);

    scene_pub_->publish(arr);
  }

  teb_local_planner::TebConfig cfg_;
  teb_local_planner::PathDynamicWaitGate gate_;
  teb_local_planner::ViaPointContainer via_points_;
  teb_local_planner::ObstContainer obst_gated_;
  teb_local_planner::ObstContainer obst_naive_;
  std::unique_ptr<teb_local_planner::TebOptimalPlanner> planner_gated_;
  std::unique_ptr<teb_local_planner::TebOptimalPlanner> planner_naive_;
  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  std::vector<SimObstacle> obstacles_;
  std::mt19937 rng_;
  std::string map_frame_;
  std::string base_frame_;
  double footprint_radius_{0.17};
  double min_obstacle_dist_{0.27};
  double spawn_x_min_{1.2};
  double spawn_x_max_{6.0};
  double spawn_y_min_{-1.6};
  double spawn_y_max_{1.6};
  double speed_min_{0.05};
  double speed_max_{0.45};
  double resample_period_{8.0};
  double pulled_dev_threshold_{0.25};
  double last_resample_t_{0.0};
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr dyn_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr naive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathWaitTestNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
