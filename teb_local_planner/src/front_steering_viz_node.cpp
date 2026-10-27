#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/front_steering_plot.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace
{

nav_msgs::msg::Path makePath(const std::vector<teb_local_planner::PoseSE2> & poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & pose : poses)
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = "map";
    pose.toPoseMsg(stamped.pose);
    path.poses.push_back(stamped);
  }
  return path;
}

visualization_msgs::msg::Marker lineMarker(
  const std::string & name, int id, float r, float g, float b,
  const std::vector<teb_local_planner::PoseSE2> & poses)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.04;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  for (const auto & pose : poses)
  {
    geometry_msgs::msg::Point point;
    point.x = pose.x();
    point.y = pose.y();
    point.z = 0.02;
    marker.points.push_back(point);
  }
  return marker;
}

visualization_msgs::msg::Marker headingMarker(
  const std::string & name, int id, const teb_local_planner::FrontSteeringSample & sample)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = std::max(0.15, std::abs(sample.v) * 0.4);
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.pose.position.x = sample.x;
  marker.pose.position.y = sample.y;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.z = std::sin(sample.theta * 0.5);
  marker.pose.orientation.w = std::cos(sample.theta * 0.5);
  const double span = teb_local_planner::kFrontSteeringMaxAngle;
  const double ratio = span > 0.0 ? std::min(1.0, std::abs(sample.steering_angle) / span) : 0.0;
  marker.color.r = static_cast<float>(ratio);
  marker.color.g = static_cast<float>(1.0 - ratio);
  marker.color.b = 0.2f;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::msg::Marker errorMarker(
  int id, float r, float g, float b,
  const teb_local_planner::PlanPathSimulation & simulation)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "full_error";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.015;
  marker.pose.orientation.w = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.85f;
  const size_t count = std::min(simulation.simulated.size(), simulation.nearest_reference.size());
  for (size_t index = 1; index < count; index += 4)
  {
    geometry_msgs::msg::Point simulated;
    simulated.x = simulation.simulated[index].x();
    simulated.y = simulation.simulated[index].y();
    simulated.z = 0.08;
    geometry_msgs::msg::Point reference;
    reference.x = simulation.nearest_reference[index].x();
    reference.y = simulation.nearest_reference[index].y();
    reference.z = 0.02;
    marker.points.push_back(simulated);
    marker.points.push_back(reference);
  }
  return marker;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("front_steering_viz");
  const auto plot = teb_local_planner::buildFrontSteeringVerificationPlot();
  RCLCPP_INFO(node->get_logger(), "%s", plot.summary.c_str());

  auto straight_pub = node->create_publisher<nav_msgs::msg::Path>("front_steering/straight", 1);
  auto arc_pub = node->create_publisher<nav_msgs::msg::Path>("front_steering/feasible_arc", 1);
  auto tight_pub = node->create_publisher<nav_msgs::msg::Path>("front_steering/tight_arc", 1);
  auto optimized_pub = node->create_publisher<nav_msgs::msg::Path>("front_steering/optimized_arc", 1);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("front_steering/markers", 1);

  const auto straight = makePath(plot.straight);
  const auto arc = makePath(plot.feasible_arc);
  const auto tight = makePath(plot.tight_arc);
  const auto optimized = makePath(plot.optimized_arc);

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(lineMarker("straight", 0, 0.1f, 0.8f, 0.2f, plot.straight));
  markers.markers.push_back(lineMarker("feasible_arc", 1, 0.2f, 0.4f, 0.9f, plot.feasible_arc));
  markers.markers.push_back(lineMarker("tight_arc", 2, 0.9f, 0.2f, 0.2f, plot.tight_arc));
  markers.markers.push_back(lineMarker("optimized_arc", 3, 0.95f, 0.75f, 0.1f, plot.optimized_arc));
  int arrow_id = 10;
  for (const auto & command : plot.optimized_commands)
  {
    markers.markers.push_back(headingMarker("optimized_velocity", arrow_id++, command));
  }
  for (const auto & command : plot.time_stretched_commands)
  {
    markers.markers.push_back(headingMarker("time_stretched_velocity", arrow_id++, command));
  }
  visualization_msgs::msg::Marker text;
  text.header.frame_id = "map";
  text.ns = "summary";
  text.id = 100;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position.x = 0.2;
  text.pose.position.y = -0.6;
  text.pose.position.z = 0.3;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.12;
  text.color.r = 1.0f;
  text.color.g = 1.0f;
  text.color.b = 1.0f;
  text.color.a = 1.0f;
  text.text = plot.summary;
  markers.markers.push_back(text);

  struct PublishedPath
  {
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;
    nav_msgs::msg::Path path;
  };
  std::vector<PublishedPath> plan_paths;
  std::vector<teb_local_planner::PlanPathSimulation> simulations;
  visualization_msgs::msg::MarkerArray plan_markers;
  const float bend_colors[5][3] = {
    {0.95f, 0.35f, 0.25f},
    {0.25f, 0.75f, 0.95f},
    {0.35f, 0.85f, 0.35f},
    {0.95f, 0.75f, 0.2f},
    {0.75f, 0.45f, 0.95f},
  };
  for (int index = 0; index < 5; ++index)
  {
    const auto simulation = teb_local_planner::simulateFullPlanPath(index);
    simulations.push_back(simulation);
    RCLCPP_INFO(
      node->get_logger(),
      "%s poses=%d 长度=%.2f m 最大横向=%.4f m 均方根=%.4f m 终点=%.4f m 航向=%.3e rad 转角=%.3f rad 转角速度=%.3f 角加速度=%.3f 饱和步数=%d",
      simulation.name.c_str(), simulation.pose_count, simulation.path_length,
      simulation.max_cross_track, simulation.rms_cross_track, simulation.final_position_error,
      simulation.max_heading_error, simulation.max_abs_delta, simulation.max_abs_rate,
      simulation.max_abs_acceleration, simulation.saturated_steps);

    PublishedPath reference;
    reference.publisher = node->create_publisher<nav_msgs::msg::Path>(
      std::string("front_steering/full_reference_") + std::to_string(index + 1), 1);
    reference.path = makePath(simulation.reference);
    plan_paths.push_back(reference);
    PublishedPath simulated;
    simulated.publisher = node->create_publisher<nav_msgs::msg::Path>(
      std::string("front_steering/full_simulated_") + std::to_string(index + 1), 1);
    simulated.path = makePath(simulation.simulated);
    plan_paths.push_back(simulated);

    auto reference_line = lineMarker(
      "full_reference", index, bend_colors[index][0] * 0.55f, bend_colors[index][1] * 0.55f,
      bend_colors[index][2] * 0.55f, simulation.reference);
    reference_line.scale.x = 0.04;
    plan_markers.markers.push_back(reference_line);
    auto simulated_line = lineMarker(
      "full_simulated", index, bend_colors[index][0], bend_colors[index][1], bend_colors[index][2],
      simulation.simulated);
    simulated_line.scale.x = 0.07;
    for (auto & point : simulated_line.points)
    {
      point.z = 0.08;
    }
    plan_markers.markers.push_back(simulated_line);
    plan_markers.markers.push_back(errorMarker(
      index, bend_colors[index][0], bend_colors[index][1], bend_colors[index][2], simulation));

    int arrow_index = 0;
    for (size_t command_index = 0; command_index < simulation.commands.size(); command_index += 8)
    {
      auto arrow = headingMarker(
        std::string("full_pose_") + std::to_string(index + 1), 1000 * index + arrow_index,
        simulation.commands[command_index]);
      arrow.scale.x = 0.35;
      arrow.scale.y = 0.06;
      plan_markers.markers.push_back(arrow);
      ++arrow_index;
    }
    if (!simulation.simulated.empty())
    {
      visualization_msgs::msg::Marker label;
      label.header.frame_id = "map";
      label.ns = "full_label";
      label.id = index;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = simulation.reference.front().x();
      label.pose.position.y = simulation.reference.front().y();
      label.pose.position.z = 0.45;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.28;
      label.color.r = bend_colors[index][0];
      label.color.g = bend_colors[index][1];
      label.color.b = bend_colors[index][2];
      label.color.a = 1.0f;
      std::ostringstream label_text;
      label_text.setf(std::ios::fixed);
      label_text.precision(3);
      label_text << simulation.name << "\n横向 " << simulation.max_cross_track
                 << " m\n均方根 " << simulation.rms_cross_track << " m";
      label.text = label_text.str();
      plan_markers.markers.push_back(label);
    }
  }
  auto plan_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "front_steering/plan_segments", 1);
  auto playback_path_pub = node->create_publisher<nav_msgs::msg::Path>(
    "front_steering/playback", 1);
  auto steering_rate_pub = node->create_publisher<std_msgs::msg::Float64>(
    "front_steering/steering_angular_velocity", 10);
  auto steering_acceleration_pub = node->create_publisher<std_msgs::msg::Float64>(
    "front_steering/steering_angular_acceleration", 10);

  size_t path_cursor = 0;
  size_t segment_cursor = 0;
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok())
  {
    const auto stamp = node->now();
    auto stamp_path = [&](nav_msgs::msg::Path path) {
      path.header.stamp = stamp;
      for (auto & pose : path.poses)
      {
        pose.header.stamp = stamp;
      }
      return path;
    };
    straight_pub->publish(stamp_path(straight));
    arc_pub->publish(stamp_path(arc));
    tight_pub->publish(stamp_path(tight));
    optimized_pub->publish(stamp_path(optimized));
    for (auto & marker : markers.markers)
    {
      marker.header.stamp = stamp;
    }
    marker_pub->publish(markers);
    for (const auto & path_pub : plan_paths)
    {
      path_pub.publisher->publish(stamp_path(path_pub.path));
    }

    visualization_msgs::msg::MarkerArray frame = plan_markers;
    if (!simulations.empty() && !simulations[path_cursor].commands.empty())
    {
      const auto & simulation = simulations[path_cursor];
      const auto & command = simulation.commands[segment_cursor];
      std_msgs::msg::Float64 rate_message;
      rate_message.data = command.steering_rate;
      std_msgs::msg::Float64 acceleration_message;
      acceleration_message.data = command.steering_acceleration;
      steering_rate_pub->publish(rate_message);
      steering_acceleration_pub->publish(acceleration_message);

      std::vector<teb_local_planner::PoseSE2> played;
      played.reserve(segment_cursor + 2);
      for (size_t pose_index = 0; pose_index <= segment_cursor && pose_index < simulation.simulated.size();
        ++pose_index)
      {
        played.push_back(simulation.simulated[pose_index]);
      }
      if (segment_cursor + 1 < simulation.simulated.size())
      {
        played.push_back(simulation.simulated[segment_cursor + 1]);
      }
      auto trail = lineMarker("playback_trail", 0, 1.0f, 1.0f, 1.0f, played);
      trail.scale.x = 0.09;
      for (auto & point : trail.points)
      {
        point.z = 0.12;
      }
      frame.markers.push_back(trail);
      auto playhead = headingMarker("playback_pose", 0, command);
      playhead.scale.x = 0.55;
      playhead.scale.y = 0.12;
      playhead.color.r = 1.0f;
      playhead.color.g = 1.0f;
      playhead.color.b = 1.0f;
      frame.markers.push_back(playhead);

      visualization_msgs::msg::Marker label;
      label.header.frame_id = "map";
      label.ns = "playback_label";
      label.id = 0;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = command.x;
      label.pose.position.y = command.y;
      label.pose.position.z = 0.7;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.28;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      std::ostringstream label_text;
      label_text.setf(std::ios::fixed);
      label_text.precision(3);
      label_text << simulation.name << "  段 " << (segment_cursor + 1) << "/"
                 << simulation.commands.size()
                 << "\n前轮角速度 " << command.steering_rate << " rad/s"
                 << "\n前轮角加速度 " << command.steering_acceleration << " rad/s^2";
      label.text = label_text.str();
      frame.markers.push_back(label);
      playback_path_pub->publish(stamp_path(makePath(played)));

      ++segment_cursor;
      if (segment_cursor >= simulation.commands.size())
      {
        segment_cursor = 0;
        path_cursor = (path_cursor + 1) % simulations.size();
      }
    }
    for (auto & marker : frame.markers)
    {
      marker.header.stamp = stamp;
    }
    plan_marker_pub->publish(frame);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
