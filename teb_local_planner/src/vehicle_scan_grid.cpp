/*********************************************************************
 * Rolling vehicle scan grid + inflated convex hull obstacles for TEB
 *********************************************************************/
#include "teb_local_planner/teb_local_planner_ros.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace teb_local_planner
{
namespace
{
constexpr uint8_t kFree = 255;
constexpr uint8_t kOcc = 0;

double cross2d(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B)
{
  return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

bool pointInPolygonXY(double x, double y, const std::vector<Eigen::Vector2d>& poly)
{
  if (poly.size() < 3) {
    return false;
  }
  bool inside = false;
  const int n = static_cast<int>(poly.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[static_cast<size_t>(i)].x();
    const double yi = poly[static_cast<size_t>(i)].y();
    const double xj = poly[static_cast<size_t>(j)].x();
    const double yj = poly[static_cast<size_t>(j)].y();
    const bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<Eigen::Vector2d> convexHullMonotone(std::vector<Eigen::Vector2d> P)
{
  const int n = static_cast<int>(P.size());
  if (n <= 2) {
    return P;
  }
  std::sort(P.begin(), P.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  int k = 0;
  std::vector<Eigen::Vector2d> H(static_cast<size_t>(2 * n));
  for (int i = 0; i < n; ++i) {
    while (k >= 2 && cross2d(H[static_cast<size_t>(k - 2)], H[static_cast<size_t>(k - 1)], P[static_cast<size_t>(i)]) <= 0.0) {
      --k;
    }
    H[static_cast<size_t>(k++)] = P[static_cast<size_t>(i)];
  }
  for (int i = n - 2, t = k + 1; i >= 0; --i) {
    while (k >= t && cross2d(H[static_cast<size_t>(k - 2)], H[static_cast<size_t>(k - 1)], P[static_cast<size_t>(i)]) <= 0.0) {
      --k;
    }
    H[static_cast<size_t>(k++)] = P[static_cast<size_t>(i)];
  }
  H.resize(static_cast<size_t>(k - 1));
  return H;
}

void inflateHullRadially(std::vector<Eigen::Vector2d>& hull, double inflation)
{
  if (hull.empty() || inflation <= 0.0) {
    return;
  }
  Eigen::Vector2d c(0.0, 0.0);
  for (const auto& p : hull) {
    c += p;
  }
  c /= static_cast<double>(hull.size());
  for (auto& v : hull) {
    Eigen::Vector2d d = v - c;
    const double len = d.norm();
    if (len > 1e-9) {
      v += (d / len) * inflation;
    }
  }
}

bool aabbIntersectsGridWindow(
  double min_x, double min_y, double max_x, double max_y,
  int origin_ix, int origin_iy, int nx, int ny, double res)
{
  const double gx0 = origin_ix * res;
  const double gy0 = origin_iy * res;
  const double gx1 = (origin_ix + nx) * res;
  const double gy1 = (origin_iy + ny) * res;
  return !(max_x < gx0 || min_x > gx1 || max_y < gy0 || min_y > gy1);
}

}  // namespace

void TebLocalPlannerROS::vehicleScanCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!cfg_->obstacles.enable_vehicle_scan_grid) {
    return;
  }
  std::lock_guard<std::mutex> lock(vehicle_scan_cloud_mutex_);
  latest_vehicle_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
}

void TebLocalPlannerROS::updateVehicleScanOccupancyGrid(
  const costmap_converter::ObstacleArrayConstPtr& obstacles,
  const std::string& obstacles_frame_id)
{
  const double res = cfg_->obstacles.vehicle_scan_grid_resolution;
  if (res < 1e-9) {
    return;
  }
  const double W = cfg_->obstacles.vehicle_scan_grid_width;
  const double H = cfg_->obstacles.vehicle_scan_grid_height;
  if (W < 1e-4 || H < 1e-4) {
    return;
  }

  const double rx = robot_pose_.position().x();
  const double ry = robot_pose_.position().y();

  const int new_origin_ix = static_cast<int>(std::floor((rx - 0.5 * W) / res));
  const int new_origin_iy = static_cast<int>(std::floor((ry - 0.5 * H) / res));
  const int new_nx = std::max(1, static_cast<int>(std::ceil(W / res)));
  const int new_ny = std::max(1, static_cast<int>(std::ceil(H / res)));
  const size_t new_cells = static_cast<size_t>(new_nx) * static_cast<size_t>(new_ny);

  std::vector<uint8_t> grid_new(new_cells, kFree);
  std::vector<uint8_t> stale_new(new_cells, 0);

  if (vehicle_scan_grid_ready_ && vehicle_scan_nx_ > 0 && vehicle_scan_ny_ > 0) {
    for (int j = 0; j < vehicle_scan_ny_; ++j) {
      for (int i = 0; i < vehicle_scan_nx_; ++i) {
        const size_t oidx = static_cast<size_t>(i + j * vehicle_scan_nx_);
        if (vehicle_scan_grid_[oidx] != kOcc) {
          continue;
        }
        const int gwx = vehicle_scan_origin_ix_ + i;
        const int gwy = vehicle_scan_origin_iy_ + j;
        const int ni = gwx - new_origin_ix;
        const int nj = gwy - new_origin_iy;
        if (ni >= 0 && ni < new_nx && nj >= 0 && nj < new_ny) {
          const size_t nid = static_cast<size_t>(ni + nj * new_nx);
          grid_new[nid] = kOcc;
          uint8_t os = vehicle_scan_stale_[oidx];
          stale_new[nid] = static_cast<uint8_t>(std::min(254, static_cast<int>(os) + 1));
        }
      }
    }
  }

  vehicle_scan_origin_ix_ = new_origin_ix;
  vehicle_scan_origin_iy_ = new_origin_iy;
  vehicle_scan_nx_ = new_nx;
  vehicle_scan_ny_ = new_ny;
  vehicle_scan_grid_.swap(grid_new);
  vehicle_scan_stale_.swap(stale_new);
  vehicle_scan_touched_.assign(new_cells, 0);
  vehicle_scan_grid_ready_ = true;

  // --- Point cloud hits (map frame) ---
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_in_map;
  {
    std::lock_guard<std::mutex> lock(vehicle_scan_cloud_mutex_);
    if (latest_vehicle_cloud_ && !latest_vehicle_cloud_->data.empty()) {
      try {
        geometry_msgs::msg::TransformStamped tf_tr =
          tf_->lookupTransform(cfg_->map_frame, latest_vehicle_cloud_->header.frame_id, tf2::TimePointZero);
        sensor_msgs::msg::PointCloud2 cloud_map;
        tf2::doTransform(*latest_vehicle_cloud_, cloud_map, tf_tr);
        cloud_in_map = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_map);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *(clock_), 2000,
          "vehicle scan: TF map<-%s: %s", latest_vehicle_cloud_->header.frame_id.c_str(), ex.what());
      }
    }
  }

  if (cloud_in_map) {
    bool has_xy = false;
    for (const auto& f : cloud_in_map->fields) {
      if (f.name == "x" || f.name == "y") {
        has_xy = true;
        break;
      }
    }
    if (has_xy) {
      sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_in_map, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_in_map, "y");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        const double mx = static_cast<double>(*iter_x);
        const double my = static_cast<double>(*iter_y);
        const int ix = static_cast<int>(std::floor(mx / res)) - vehicle_scan_origin_ix_;
        const int iy = static_cast<int>(std::floor(my / res)) - vehicle_scan_origin_iy_;
        if (ix >= 0 && ix < vehicle_scan_nx_ && iy >= 0 && iy < vehicle_scan_ny_) {
          const size_t id = static_cast<size_t>(ix + iy * vehicle_scan_nx_);
          vehicle_scan_grid_[id] = kOcc;
          vehicle_scan_stale_[id] = 0;
          vehicle_scan_touched_[id] = 1;
        }
      }
    }
  }

  // --- Polygons from converter (map frame vertices); obstacles 可为空（仅点云+衰减+发布）---
  if (obstacles) {
  Eigen::Affine3d tf_map_from_obs = Eigen::Affine3d::Identity();
  bool have_tf = false;
  if (obstacles_frame_id != cfg_->map_frame && tf_) {
    try {
      geometry_msgs::msg::TransformStamped tr =
        tf_->lookupTransform(cfg_->map_frame, obstacles_frame_id, tf2::TimePointZero);
      tf_map_from_obs = tf2::transformToEigen(tr);
      have_tf = true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, *(clock_), 2000,
        "vehicle grid: TF %s->map: %s", obstacles_frame_id.c_str(), ex.what());
    }
  } else if (obstacles_frame_id == cfg_->map_frame) {
    have_tf = true;
  }

  const int fill_thr = std::max(1, cfg_->obstacles.vehicle_scan_polygon_fill_threshold);

  for (std::size_t oi = 0; oi < obstacles->obstacles.size(); ++oi) {
    const auto& poly = obstacles->obstacles[oi].polygon;
    if (poly.points.size() < 3) {
      continue;
    }

    std::vector<Eigen::Vector2d> verts;
    verts.reserve(poly.points.size());
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    for (const auto& p : poly.points) {
      Eigen::Vector3d v3(p.x, p.y, p.z);
      if (have_tf && obstacles_frame_id != cfg_->map_frame) {
        v3 = tf_map_from_obs * v3;
      }
      Eigen::Vector2d v2(v3.x(), v3.y());
      verts.push_back(v2);
      min_x = std::min(min_x, v2.x());
      min_y = std::min(min_y, v2.y());
      max_x = std::max(max_x, v2.x());
      max_y = std::max(max_y, v2.y());
    }

    if (!aabbIntersectsGridWindow(
          min_x, min_y, max_x, max_y,
          vehicle_scan_origin_ix_, vehicle_scan_origin_iy_, vehicle_scan_nx_, vehicle_scan_ny_, res)) {
      continue;
    }

    bool maybe_marked = false;
    const int i0 = std::max(0, static_cast<int>(std::floor(min_x / res)) - vehicle_scan_origin_ix_);
    const int i1 = std::min(vehicle_scan_nx_ - 1, static_cast<int>(std::ceil(max_x / res)) - vehicle_scan_origin_ix_);
    const int j0 = std::max(0, static_cast<int>(std::floor(min_y / res)) - vehicle_scan_origin_iy_);
    const int j1 = std::min(vehicle_scan_ny_ - 1, static_cast<int>(std::ceil(max_y / res)) - vehicle_scan_origin_iy_);
    for (int jj = j0; jj <= j1 && !maybe_marked; ++jj) {
      for (int ii = i0; ii <= i1; ++ii) {
        const size_t id = static_cast<size_t>(ii + jj * vehicle_scan_nx_);
        if (vehicle_scan_grid_[id] == kOcc) {
          const double cx = (vehicle_scan_origin_ix_ + ii + 0.5) * res;
          const double cy = (vehicle_scan_origin_iy_ + jj + 0.5) * res;
          if (pointInPolygonXY(cx, cy, verts)) {
            maybe_marked = true;
            break;
          }
        }
      }
    }
    if (!maybe_marked) {
      continue;
    }

    int marked_inside = 0;
    for (int jj = j0; jj <= j1; ++jj) {
      for (int ii = i0; ii <= i1; ++ii) {
        const size_t id = static_cast<size_t>(ii + jj * vehicle_scan_nx_);
        if (vehicle_scan_grid_[id] != kOcc) {
          continue;
        }
        const double cx = (vehicle_scan_origin_ix_ + ii + 0.5) * res;
        const double cy = (vehicle_scan_origin_iy_ + jj + 0.5) * res;
        if (pointInPolygonXY(cx, cy, verts)) {
          ++marked_inside;
        }
      }
    }

    const bool fill_all = (marked_inside >= fill_thr);
    for (int jj = j0; jj <= j1; ++jj) {
      for (int ii = i0; ii <= i1; ++ii) {
        const double cx = (vehicle_scan_origin_ix_ + ii + 0.5) * res;
        const double cy = (vehicle_scan_origin_iy_ + jj + 0.5) * res;
        if (!pointInPolygonXY(cx, cy, verts)) {
          continue;
        }
        const size_t id = static_cast<size_t>(ii + jj * vehicle_scan_nx_);
        if (fill_all) {
          vehicle_scan_grid_[id] = kOcc;
          vehicle_scan_stale_[id] = 0;
          vehicle_scan_touched_[id] = 1;
        } else {
          if (vehicle_scan_grid_[id] == kOcc) {
            vehicle_scan_stale_[id] = 0;
            vehicle_scan_touched_[id] = 1;
          }
        }
      }
    }
  }
  }  // if (obstacles)

  // --- Decay ---
  const int max_stale = std::max(1, cfg_->obstacles.vehicle_scan_max_stale_cycles);
  for (size_t i = 0; i < vehicle_scan_grid_.size(); ++i) {
    if (vehicle_scan_grid_[i] != kOcc) {
      continue;
    }
    if (vehicle_scan_touched_[i] == 0) {
      if (vehicle_scan_stale_[i] < 255) {
        vehicle_scan_stale_[i]++;
      }
      if (vehicle_scan_stale_[i] > static_cast<uint8_t>(max_stale)) {
        vehicle_scan_grid_[i] = kFree;
      }
    } else {
      vehicle_scan_stale_[i] = 0;
    }
  }
}

void TebLocalPlannerROS::appendVehicleScanInflatedHullObstacles()
{
  if (!vehicle_scan_grid_ready_ || vehicle_scan_nx_ <= 0 || vehicle_scan_ny_ <= 0) {
    return;
  }
  const double res = cfg_->obstacles.vehicle_scan_grid_resolution;
  const double inflation = cfg_->obstacles.vehicle_scan_hull_inflation;
  const int nx = vehicle_scan_nx_;
  const int ny = vehicle_scan_ny_;

  const int di[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dj[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

  std::vector<uint8_t> visited(static_cast<size_t>(nx * ny), 0);

  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const size_t sid = static_cast<size_t>(i + j * nx);
      if (visited[sid] != 0 || vehicle_scan_grid_[sid] != kOcc) {
        continue;
      }

      std::vector<Eigen::Vector2d> cluster_pts;
      std::queue<std::pair<int, int>> q;
      q.push({i, j});
      visited[sid] = 1;

      while (!q.empty()) {
        const auto cur = q.front();
        q.pop();
        const int ci = cur.first;
        const int cj = cur.second;
        const double cx = (vehicle_scan_origin_ix_ + ci + 0.5) * res;
        const double cy = (vehicle_scan_origin_iy_ + cj + 0.5) * res;
        cluster_pts.emplace_back(cx, cy);

        for (int k = 0; k < 8; ++k) {
          const int ni = ci + di[k];
          const int nj = cj + dj[k];
          if (ni < 0 || ni >= nx || nj < 0 || nj >= ny) {
            continue;
          }
          const size_t nid = static_cast<size_t>(ni + nj * nx);
          if (visited[nid] != 0 || vehicle_scan_grid_[nid] != kOcc) {
            continue;
          }
          visited[nid] = 1;
          q.push({ni, nj});
        }
      }

      if (cluster_pts.empty()) {
        continue;
      }
      if (cluster_pts.size() == 1) {
        const double r = std::max(res * 0.5 + inflation, inflation + 0.01);
        obstacles_.push_back(ObstaclePtr(
          new CircularObstacle(cluster_pts[0].x(), cluster_pts[0].y(), r)));
        continue;
      }
      if (cluster_pts.size() == 2) {
        const Eigen::Vector2d a = cluster_pts[0];
        const Eigen::Vector2d b = cluster_pts[1];
        const double midx = 0.5 * (a.x() + b.x());
        const double midy = 0.5 * (a.y() + b.y());
        const double half_len = 0.5 * (a - b).norm();
        const double r = half_len + inflation;
        obstacles_.push_back(ObstaclePtr(new CircularObstacle(midx, midy, r)));
        continue;
      }

      std::vector<Eigen::Vector2d> hull = convexHullMonotone(cluster_pts);
      if (hull.size() < 3) {
        continue;
      }
      inflateHullRadially(hull, inflation);

      PolygonObstacle* polyobst = new PolygonObstacle();
      for (const auto& v : hull) {
        polyobst->pushBackVertex(v.x(), v.y());
      }
      polyobst->finalizePolygon();
      obstacles_.push_back(ObstaclePtr(polyobst));
    }
  }
}

void TebLocalPlannerROS::publishVehicleScanOccupancyGrid()
{
  if (!vehicle_scan_grid_pub_) {
    return;
  }
  if (!vehicle_scan_grid_ready_ || vehicle_scan_nx_ <= 0 || vehicle_scan_ny_ <= 0) {
    return;
  }
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = clock_->now();
  msg.header.frame_id = cfg_->map_frame;
  msg.info.resolution = cfg_->obstacles.vehicle_scan_grid_resolution;
  msg.info.width = static_cast<uint32_t>(vehicle_scan_nx_);
  msg.info.height = static_cast<uint32_t>(vehicle_scan_ny_);
  msg.info.origin.position.x = vehicle_scan_origin_ix_ * msg.info.resolution;
  msg.info.origin.position.y = vehicle_scan_origin_iy_ * msg.info.resolution;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(vehicle_scan_grid_.size());
  for (size_t i = 0; i < vehicle_scan_grid_.size(); ++i) {
    msg.data[i] = (vehicle_scan_grid_[i] == kOcc) ? 100 : 0;
  }

  vehicle_scan_grid_pub_->publish(msg);
}

}  // namespace teb_local_planner
