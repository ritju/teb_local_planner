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

// ros stuff
#include "teb_local_planner/visualization.h"
#include "teb_local_planner/optimal_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace teb_local_planner
{

void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                 rclcpp::Publisher<nav_msgs::msg::Path> *pub) {
    if(path.empty())
        return;

    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub->publish(gui_path);
}

TebVisualization::TebVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh, const TebConfig& cfg) : nh_(nh), cfg_(&cfg), initialized_(false)
{
}

void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  publishPlan(global_plan, global_plan_pub_.get());
}

void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& local_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  publishPlan(local_plan, local_plan_pub_.get());
}

void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  
    // create path msg
    nav_msgs::msg::Path teb_path;
    teb_path.header.frame_id = cfg_->map_frame;
    teb_path.header.stamp = nh_->now();
    
    // create pose_array (along trajectory)
    geometry_msgs::msg::PoseArray teb_poses;
    teb_poses.header.frame_id = teb_path.header.frame_id;
    teb_poses.header.stamp = teb_path.header.stamp;
    
    // fill path msgs with teb configurations
    for (int i=0; i < teb.sizePoses(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      teb.Pose(i).toPoseMsg(pose.pose);
      pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);
    }
    local_plan_pub_->publish(teb_path);
    teb_poses_pub_->publish(teb_poses);
}



void TebVisualization::publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns,
                                                  const std_msgs::msg::ColorRGBA &color)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  std::vector<visualization_msgs::msg::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers, color);
  if (markers.empty())
    return;
  
  int idx = 1000000;  // avoid overshadowing by obstacles
  for (std::vector<visualization_msgs::msg::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  {
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = nh_->now();
    marker_it->action = visualization_msgs::msg::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = rclcpp::Duration(2, 0);
    teb_marker_pub_->publish(*marker_it);
  }
  
}

void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
{
  publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
}


void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  // 收集所有障碍物的 marker，最后一次性发布
  std::vector<visualization_msgs::msg::Marker> markers;
  rclcpp::Time current_time = nh_->now();
  
  // Visualize point obstacles
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = current_time;
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(2, 0);
    marker.pose.orientation.w = 1.0;
    
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(*obst);
      if (!pobst)
        continue;

      if (cfg_->hcp.visualize_with_time_as_z_axis_scale < 0.001)
      {
        geometry_msgs::msg::Point point;
        point.x = pobst->x();
        point.y = pobst->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      else // Spatiotemporally point obstacles become a line
      {
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        geometry_msgs::msg::Point start;
        start.x = pobst->x();
        start.y = pobst->y();
        start.z = 0;
        marker.points.push_back(start);

        geometry_msgs::msg::Point end;
        double t = 20;
        Eigen::Vector2d pred;
        pobst->predictCentroidConstantVelocity(t, pred);
        end.x = pred[0];
        end.y = pred[1];
        end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*t;
        marker.points.push_back(end);
      }
    }
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 只有当有点障碍物时才添加到数组
    if (!marker.points.empty())
    {
      markers.push_back(marker);
    }
  }
  
  // Visualize circular obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      std::shared_ptr<CircularObstacle> pobst = std::dynamic_pointer_cast<CircularObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = current_time;
      marker.ns = "CircularObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      geometry_msgs::msg::Point point;
      point.x = pobst->x();
      point.y = pobst->y();
      point.z = 0;
      marker.points.push_back(point);

      marker.scale.x = pobst->radius();
      marker.scale.y = pobst->radius();
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      markers.push_back(marker);
    }
  }

  // Visualize line obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      std::shared_ptr<LineObstacle> pobst = std::dynamic_pointer_cast<LineObstacle>(*obst);
      if (!pobst)
        continue;
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = current_time;
      marker.ns = "LineObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      marker.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point start;
      start.x = pobst->start().x();
      start.y = pobst->start().y();
      start.z = 0;
      marker.points.push_back(start);
      geometry_msgs::msg::Point end;
      end.x = pobst->end().x();
      end.y = pobst->end().y();
      end.z = 0;
      marker.points.push_back(end);
  
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      
      markers.push_back(marker);
    }
  }
  

  // Visualize polygon obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      std::shared_ptr<PolygonObstacle> pobst = std::dynamic_pointer_cast<PolygonObstacle>(*obst);
      if (!pobst)
        continue;
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = current_time;
      marker.ns = "PolyObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      marker.pose.orientation.w = 1.0;
      
      for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
      {
        geometry_msgs::msg::Point point;
        point.x = vertex->x();
        point.y = vertex->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      
      // Also add last point to close the polygon
      // but only if polygon has more than 2 points (it is not a line)
      if (pobst->vertices().size() > 2)
      {
        geometry_msgs::msg::Point point;
        point.x = pobst->vertices().front().x();
        point.y = pobst->vertices().front().y();
        point.z = 0;
        marker.points.push_back(point);
      }
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      
      markers.push_back(marker);
    }
  }
  
  // 一次性发布所有障碍物 marker，使用 MarkerArray
  if (!markers.empty())
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = markers;
    teb_marker_array_pub_->publish(marker_array);
  }
}

namespace
{

geometry_msgs::msg::Point toPoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void appendCirclePoints(double cx, double cy, double radius, int segments,
                        std::vector<geometry_msgs::msg::Point>& points, double z = 0.0)
{
  if (radius <= 0.0 || segments < 3)
  {
    points.push_back(toPoint(cx, cy, z));
    return;
  }
  for (int k = 0; k <= segments; ++k)
  {
    const double ang = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(segments);
    points.push_back(toPoint(cx + radius * std::cos(ang), cy + radius * std::sin(ang), z));
  }
}

void predictObstacleContour(const Obstacle& obst, double t, std::vector<geometry_msgs::msg::Point>& points,
                            double* line_width, double z = 0.0)
{
  points.clear();
  if (line_width)
    *line_width = 0.03;

  if (const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obst))
  {
    Eigen::Vector2d pred;
    pobst->predictCentroidConstantVelocity(t, pred);
    points.push_back(toPoint(pred.x(), pred.y(), z));
    return;
  }
  if (const CircularObstacle* cobst = dynamic_cast<const CircularObstacle*>(&obst))
  {
    Eigen::Vector2d pred;
    cobst->predictCentroidConstantVelocity(t, pred);
    appendCirclePoints(pred.x(), pred.y(), cobst->radius(), 24, points, z);
    return;
  }
  if (const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obst))
  {
    const Eigen::Vector2d offset = t * lobst->getCentroidVelocity();
    const Eigen::Vector2d s = lobst->start() + offset;
    const Eigen::Vector2d e = lobst->end() + offset;
    points.push_back(toPoint(s.x(), s.y(), z));
    points.push_back(toPoint(e.x(), e.y(), z));
    return;
  }
  if (const PillObstacle* pill = dynamic_cast<const PillObstacle*>(&obst))
  {
    const Eigen::Vector2d offset = t * pill->getCentroidVelocity();
    const Eigen::Vector2d s = pill->start() + offset;
    const Eigen::Vector2d e = pill->end() + offset;
    points.push_back(toPoint(s.x(), s.y(), z));
    points.push_back(toPoint(e.x(), e.y(), z));
    if (line_width)
      *line_width = 0.06;
    return;
  }
  if (const PolygonObstacle* poly = dynamic_cast<const PolygonObstacle*>(&obst))
  {
    Point2dContainer pred;
    poly->predictVertices(t, pred);
    for (const Eigen::Vector2d& v : pred)
      points.push_back(toPoint(v.x(), v.y(), z));
    if (!points.empty())
      points.push_back(points.front());
    return;
  }

  Eigen::Vector2d pred;
  obst.predictCentroidConstantVelocity(t, pred);
  points.push_back(toPoint(pred.x(), pred.y(), z));
}

std_msgs::msg::ColorRGBA colorBySpatioTemporalDist(double dist, double min_dist, double inflation_dist,
                                                   double alpha)
{
  if (dist < min_dist)
    return TebVisualization::toColorMsg(alpha, 1.0, 0.15, 0.15);
  if (dist < inflation_dist)
    return TebVisualization::toColorMsg(alpha, 1.0, 0.85, 0.1);
  return TebVisualization::toColorMsg(alpha, 0.15, 0.85, 0.2);
}

TebPoseTimeSnapshot buildPoseTimeSamplesFromTeb(const TimedElasticBand& teb, int stride)
{
  TebPoseTimeSnapshot samples;
  if (teb.sizePoses() < 3)
    return samples;

  stride = std::max(1, stride);
  double time = teb.TimeDiff(0);
  for (int i = 1; i < teb.sizePoses() - 1; ++i)
  {
    if (((i - 1) % stride) == 0)
    {
      TebPoseTimeSample sample;
      sample.pose_idx = i;
      sample.t = time;
      sample.pose = teb.Pose(i);
      samples.push_back(sample);
    }
    time += teb.TimeDiff(i);
  }
  return samples;
}

TebPoseTimeSnapshot filterSnapshotByStride(const TebPoseTimeSnapshot& snapshot, int stride)
{
  if (stride <= 1)
    return snapshot;
  TebPoseTimeSnapshot filtered;
  for (std::size_t i = 0; i < snapshot.size(); ++i)
  {
    if ((static_cast<int>(i) % stride) == 0)
      filtered.push_back(snapshot[i]);
  }
  return filtered;
}

void appendDynamicObstacleDebugLayer(visualization_msgs::msg::MarkerArray& marker_array,
                                     const TebPoseTimeSnapshot& samples,
                                     const ObstContainer& obstacles,
                                     const BaseRobotFootprintModel& robot_model,
                                     const TebConfig& cfg,
                                     const std::string& ns_prefix,
                                     double alpha,
                                     int& marker_id,
                                     const std_msgs::msg::Header& header)
{
  const double min_dist = cfg.obstacles.min_obstacle_dist;
  const double inflation_dist = cfg.obstacles.dynamic_obstacle_inflation_dist;
  const double z_scale = cfg.trajectory.dynamic_obstacle_debug_time_z_scale;

  for (std::size_t obst_i = 0; obst_i < obstacles.size(); ++obst_i)
  {
    const ObstaclePtr& obst = obstacles[obst_i];
    if (!obst || !obst->isDynamic())
      continue;

    for (const TebPoseTimeSample& sample : samples)
    {
      // Same t for robot pose and obstacle prediction — lift both to z=scale*t so they share a "time plane"
      const double z = z_scale * sample.t;
      const double dist = robot_model.estimateSpatioTemporalDistance(sample.pose, obst.get(), sample.t);
      const std_msgs::msg::ColorRGBA color = colorBySpatioTemporalDist(dist, min_dist, inflation_dist, alpha);

      std::vector<geometry_msgs::msg::Point> contour;
      double line_width = 0.03;
      predictObstacleContour(*obst, sample.t, contour, &line_width, z);

      visualization_msgs::msg::Marker obst_marker;
      obst_marker.header = header;
      obst_marker.ns = ns_prefix + "/obstacle";
      obst_marker.id = marker_id++;
      obst_marker.action = visualization_msgs::msg::Marker::ADD;
      obst_marker.lifetime = rclcpp::Duration(2, 0);
      obst_marker.pose.orientation.w = 1.0;
      obst_marker.color = color;
      obst_marker.scale.x = line_width;
      obst_marker.scale.y = line_width;
      obst_marker.scale.z = line_width;

      if (contour.size() <= 1)
      {
        obst_marker.type = visualization_msgs::msg::Marker::SPHERE;
        obst_marker.pose.position = contour.empty() ? toPoint(0, 0, z) : contour.front();
        obst_marker.scale.x = obst_marker.scale.y = obst_marker.scale.z = 0.08;
      }
      else
      {
        obst_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        obst_marker.points = contour;
      }
      marker_array.markers.push_back(obst_marker);

      std::vector<visualization_msgs::msg::Marker> robot_markers;
      robot_model.visualizeRobot(sample.pose, robot_markers, color);
      for (visualization_msgs::msg::Marker& rm : robot_markers)
      {
        rm.header = header;
        rm.ns = ns_prefix + "/robot";
        rm.id = marker_id++;
        rm.action = visualization_msgs::msg::Marker::ADD;
        rm.lifetime = rclcpp::Duration(2, 0);
        rm.color = color;
        rm.pose.position.z += z;
        for (geometry_msgs::msg::Point& pt : rm.points)
          pt.z += z;
        marker_array.markers.push_back(rm);
      }

      Eigen::Vector2d obst_centroid;
      obst->predictCentroidConstantVelocity(sample.t, obst_centroid);
      visualization_msgs::msg::Marker link;
      link.header = header;
      link.ns = ns_prefix + "/pair";
      link.id = marker_id++;
      link.type = visualization_msgs::msg::Marker::LINE_LIST;
      link.action = visualization_msgs::msg::Marker::ADD;
      link.lifetime = rclcpp::Duration(2, 0);
      link.pose.orientation.w = 1.0;
      link.scale.x = 0.015;
      link.color = color;
      link.points.push_back(toPoint(sample.pose.x(), sample.pose.y(), z));
      link.points.push_back(toPoint(obst_centroid.x(), obst_centroid.y(), z));
      marker_array.markers.push_back(link);

      // Label shared by this (pose_idx, t) pair for easy RViz identification
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = ns_prefix + "/label";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.lifetime = rclcpp::Duration(2, 0);
      text.pose.orientation.w = 1.0;
      text.pose.position.x = 0.5 * (sample.pose.x() + obst_centroid.x());
      text.pose.position.y = 0.5 * (sample.pose.y() + obst_centroid.y());
      text.pose.position.z = z + 0.05;
      text.scale.z = 0.08;
      text.color = color;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "i=%d t=%.2f", sample.pose_idx, sample.t);
      text.text = buf;
      marker_array.markers.push_back(text);
    }
  }
}

} // namespace

void TebVisualization::publishDynamicObstacleDebug(const TimedElasticBand& teb_optimized,
                                                   const TebPoseTimeSnapshot* edge_build_snapshot,
                                                   const ObstContainer& obstacles,
                                                   const BaseRobotFootprintModel& robot_model) const
{
  if (printErrorWhenNotInitialized() || !teb_dyn_obst_debug_pub_)
    return;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = cfg_->map_frame;
  del.header.stamp = nh_->now();
  del.ns = "";
  del.id = 0;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(del);

  if (!cfg_->trajectory.publish_dynamic_obstacle_debug ||
      !cfg_->obstacles.include_dynamic_obstacles ||
      obstacles.empty() ||
      teb_optimized.sizePoses() < 3)
  {
    teb_dyn_obst_debug_pub_->publish(marker_array);
    return;
  }

  bool has_dynamic = false;
  for (const ObstaclePtr& obst : obstacles)
  {
    if (obst && obst->isDynamic())
    {
      has_dynamic = true;
      break;
    }
  }
  if (!has_dynamic)
  {
    teb_dyn_obst_debug_pub_->publish(marker_array);
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = cfg_->map_frame;
  header.stamp = nh_->now();

  const int stride = std::max(1, cfg_->trajectory.dynamic_obstacle_debug_pose_stride);
  const TebPoseTimeSnapshot opt_samples = buildPoseTimeSamplesFromTeb(teb_optimized, stride);

  int marker_id = 1;
  appendDynamicObstacleDebugLayer(marker_array, opt_samples, obstacles, robot_model, *cfg_,
                                  "DynObstOpt", 0.95, marker_id, header);

  if (edge_build_snapshot && !edge_build_snapshot->empty())
  {
    const TebPoseTimeSnapshot build_samples = filterSnapshotByStride(*edge_build_snapshot, stride);
    appendDynamicObstacleDebugLayer(marker_array, build_samples, obstacles, robot_model, *cfg_,
                                    "DynObstBuild", 0.35, marker_id, header);
  }

  teb_dyn_obst_debug_pub_->publish(marker_array);
}


void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
  if ( via_points.empty() || printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = nh_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(2, 0);
  marker.pose.orientation.w = 1.0;
  
  for (std::size_t i=0; i < via_points.size(); ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  teb_marker_pub_->publish( marker );
}

void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
{
if ( printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = nh_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  
  // Iterate through teb pose sequence
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb )
  {	  
    // iterate single poses
    PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
    TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
    PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
    std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one.
    double time = 0;

    while (it_pose != it_pose_end)
    {
      geometry_msgs::msg::Point point_start;
      point_start.x = (*it_pose)->x();
      point_start.y = (*it_pose)->y();
      point_start.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
      marker.points.push_back(point_start);

      time += (*it_timediff)->dt();

      geometry_msgs::msg::Point point_end;
      point_end.x = (*boost::next(it_pose))->x();
      point_end.y = (*boost::next(it_pose))->y();
      point_end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
      marker.points.push_back(point_end);
      ++it_pose;
      ++it_timediff;
    }
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_->publish( marker );
}

void TebVisualization::publishFeedbackMessage(const std::vector< std::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                              unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
{
  teb_msgs::msg::FeedbackMsg msg;
  msg.header.stamp = nh_->now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = selected_trajectory_idx;
  
  
  msg.trajectories.resize(teb_planners.size());
  
  // Iterate through teb pose sequence
  std::size_t idx_traj = 0;
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj )
  {   
    msg.trajectories[idx_traj].header = msg.header;
    it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  }
  
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_->publish(msg);
}

void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
{
  teb_msgs::msg::FeedbackMsg msg;
  msg.header.stamp = nh_->now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = 0;
  
  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);
 
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_->publish(msg);
}

std_msgs::msg::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
{
  std_msgs::msg::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool TebVisualization::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TebVisualization class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

nav2_util::CallbackReturn TebVisualization::on_configure()
{
  // register topics
  global_plan_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);;
  local_plan_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("local_plan",1);
  teb_poses_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>("teb_poses", 1);
  teb_marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("teb_markers", 1);
  teb_marker_array_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_marker_array", 1);
  teb_dyn_obst_debug_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_dynamic_obstacle_debug", 1);
  feedback_pub_ = nh_->create_publisher<teb_msgs::msg::FeedbackMsg>("teb_feedback", 1);

  initialized_ = true;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_activate()
{
  global_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  teb_poses_pub_->on_activate();
  teb_marker_pub_->on_activate();
  teb_marker_array_pub_->on_activate();
  teb_dyn_obst_debug_pub_->on_activate();
  feedback_pub_->on_activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_deactivate()
{
  global_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  teb_poses_pub_->on_deactivate();
  teb_marker_pub_->on_deactivate();
  teb_marker_array_pub_->on_deactivate();
  teb_dyn_obst_debug_pub_->on_deactivate();
  feedback_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  teb_poses_pub_.reset();
  teb_marker_pub_.reset();
  teb_marker_array_pub_.reset();
  teb_dyn_obst_debug_pub_.reset();
  feedback_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace teb_local_planner
