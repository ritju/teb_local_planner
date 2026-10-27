#include "teb_local_planner/front_steering_plot.hpp"
#include "teb_local_planner/plan_curvature_segments.hpp"
#include "teb_local_planner/plan_full_paths.hpp"

#include "teb_local_planner/front_steering_model.hpp"
#include "teb_local_planner/g2o_types/edge_front_steering.h"
#include "teb_local_planner/g2o_types/edge_kinematics.h"
#include "teb_local_planner/g2o_types/edge_time_optimal.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

namespace teb_local_planner
{
namespace
{

class EdgePosePositionAnchor : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
  void computeError() override
  {
    const VertexPose * pose = static_cast<const VertexPose *>(_vertices[0]);
    _error[0] = pose->x() - _measurement.x();
    _error[1] = pose->y() - _measurement.y();
  }

  bool read(std::istream &) override
  {
    return true;
  }

  bool write(std::ostream & os) const override
  {
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgePoseHeadingAnchor : public g2o::BaseUnaryEdge<1, double, VertexPose>
{
public:
  void computeError() override
  {
    const VertexPose * pose = static_cast<const VertexPose *>(_vertices[0]);
    _error[0] = g2o::normalize_theta(pose->theta() - _measurement);
  }

  bool read(std::istream &) override
  {
    return true;
  }

  bool write(std::ostream & os) const override
  {
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void fillFrontSteeringConfig(TebConfig & cfg)
{
  cfg.robot.front_steering_rear_drive = true;
  cfg.robot.wheelbase = kFrontSteeringWheelbase;
  cfg.robot.rear_track = kFrontSteeringRearTrack;
  cfg.robot.max_steering_angle = kFrontSteeringMaxAngle;
  cfg.robot.max_steering_rate = kFrontSteeringMaxRate;
  cfg.robot.max_steering_acceleration = kFrontSteeringMaxAcceleration;
  cfg.robot.steering_min_speed = kFrontSteeringMinSpeed;
  cfg.robot.steering_omega_epsilon = kFrontSteeringOmegaEpsilon;
  cfg.robot.no_inplace_rotation = true;
  cfg.robot.cmd_angle_instead_rotvel = false;
  cfg.robot.min_turning_radius = minimumTurningRadius(kFrontSteeringWheelbase, kFrontSteeringMaxAngle);
  cfg.robot.max_vel_y = 0.0;
  cfg.robot.max_vel_theta = 3.0;
  cfg.optim.penalty_epsilon = 0.0;
  cfg.optim.weight_kinematics_nh = 1000.0;
  cfg.optim.weight_kinematics_turning_radius = 100.0;
  cfg.optim.weight_steering_kinematics = 500.0;
  cfg.optim.weight_steering_rate = 300.0;
  cfg.optim.weight_steering_acceleration = 200.0;
  cfg.optim.weight_optimaltime = 1.0;
}

std::vector<PoseSE2> circularArc(double radius, double step, int segments)
{
  std::vector<PoseSE2> poses;
  poses.reserve(static_cast<size_t>(segments) + 1);
  for (int index = 0; index <= segments; ++index)
  {
    const double phi = index * step;
    poses.push_back(PoseSE2(radius * std::sin(phi), radius * (1.0 - std::cos(phi)), phi));
  }
  return poses;
}

void configureOptimizer(g2o::SparseOptimizer & optimizer)
{
  static bool threading_ready = false;
  if (!threading_ready)
  {
    optimizer.initMultiThreading();
    threading_ready = true;
  }
  auto linear = std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
  linear->setBlockOrdering(true);
  auto block = std::make_unique<g2o::BlockSolverX>(std::move(linear));
  optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(block)));
}

double kinematicsError(const TebConfig & cfg, const PoseSE2 & first, const PoseSE2 & second, double dt)
{
  VertexPose pose1(first, true);
  VertexPose pose2(second, true);
  VertexTimeDiff timediff(dt, true);
  EdgeSteeringKinematics edge;
  edge.setTebConfig(cfg);
  edge.setVertex(0, &pose1);
  edge.setVertex(1, &pose2);
  edge.setVertex(2, &timediff);
  Eigen::Matrix<double, 1, 1> information;
  information << 1.0;
  edge.setInformation(information);
  return edge.getError()[0];
}

FrontSteeringSample sampleFromSegment(
  const PoseSE2 & pose, const SegmentKinematics & segment, double wheelbase, double rear_track)
{
  FrontSteeringSample sample;
  sample.x = pose.x();
  sample.y = pose.y();
  sample.theta = pose.theta();
  sample.v = segment.velocity;
  sample.omega = segment.omega;
  sample.steering_angle = segment.steering_angle;
  sample.rear_left = segment.velocity - 0.5 * rear_track * segment.omega;
  sample.rear_right = segment.velocity + 0.5 * rear_track * segment.omega;
  (void)wheelbase;
  return sample;
}

struct ReferenceProjection
{
  double distance{std::numeric_limits<double>::infinity()};
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

ReferenceProjection projectOntoReference(const PoseSE2 & pose, const std::vector<PoseSE2> & reference)
{
  ReferenceProjection best;
  for (size_t index = 0; index + 1 < reference.size(); ++index)
  {
    const double start_x = reference[index].x();
    const double start_y = reference[index].y();
    const double end_x = reference[index + 1].x();
    const double end_y = reference[index + 1].y();
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;
    const double length_squared = dx * dx + dy * dy;
    double ratio = 0.0;
    if (length_squared > 1e-12)
    {
      ratio = ((pose.x() - start_x) * dx + (pose.y() - start_y) * dy) / length_squared;
      ratio = clampDouble(ratio, 0.0, 1.0);
    }
    const double projected_x = start_x + ratio * dx;
    const double projected_y = start_y + ratio * dy;
    const double distance = std::hypot(pose.x() - projected_x, pose.y() - projected_y);
    if (distance < best.distance)
    {
      const double delta_theta = g2o::normalize_theta(
        reference[index + 1].theta() - reference[index].theta());
      best.distance = distance;
      best.x = projected_x;
      best.y = projected_y;
      best.theta = g2o::normalize_theta(reference[index].theta() + ratio * delta_theta);
    }
  }
  return best;
}

}  // namespace

FrontSteeringVerificationPlot buildFrontSteeringVerificationPlot()
{
  FrontSteeringVerificationPlot plot;
  TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  plot.straight.push_back(PoseSE2(0.0, 0.0, 0.0));
  plot.straight.push_back(PoseSE2(1.0, 0.0, 0.0));
  plot.straight.push_back(PoseSE2(2.0, 0.0, 0.0));
  plot.straight.push_back(PoseSE2(3.0, 0.0, 0.0));
  plot.feasible_arc = circularArc(1.0, 0.25, 5);
  plot.tight_arc = circularArc(0.4, 0.2, 4);

  {
    g2o::SparseOptimizer optimizer;
    configureOptimizer(optimizer);
    std::vector<VertexPose *> vertices;
    const double dt_ref = 0.25;
    for (size_t index = 0; index < plot.feasible_arc.size(); ++index)
    {
      const bool fixed = index == 0 || index + 1 == plot.feasible_arc.size();
      const PoseSE2 & reference = plot.feasible_arc[index];
      const double theta = fixed ? reference.theta() : 0.0;
      VertexPose * vertex = new VertexPose(reference.x(), reference.y(), theta, fixed);
      vertex->setId(static_cast<int>(index));
      optimizer.addVertex(vertex);
      vertices.push_back(vertex);
      if (!fixed)
      {
        EdgePosePositionAnchor * anchor = new EdgePosePositionAnchor();
        anchor->setId(1000 + static_cast<int>(index));
        anchor->setVertex(0, vertex);
        anchor->setMeasurement(reference.position());
        Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * 800.0;
        anchor->setInformation(information);
        optimizer.addEdge(anchor);

        EdgePoseHeadingAnchor * heading = new EdgePoseHeadingAnchor();
        heading->setId(1100 + static_cast<int>(index));
        heading->setVertex(0, vertex);
        heading->setMeasurement(reference.theta());
        Eigen::Matrix<double, 1, 1> heading_information;
        heading_information << 400.0;
        heading->setInformation(heading_information);
        optimizer.addEdge(heading);
      }
    }
    std::vector<VertexTimeDiff *> times;
    for (size_t index = 0; index + 1 < vertices.size(); ++index)
    {
      VertexTimeDiff * timediff = new VertexTimeDiff(dt_ref, true);
      timediff->setId(100 + static_cast<int>(index));
      optimizer.addVertex(timediff);
      times.push_back(timediff);
    }
    Eigen::Matrix<double, 2, 2> carlike_information;
    carlike_information.setZero();
    carlike_information(0, 0) = cfg.optim.weight_kinematics_nh;
    carlike_information(1, 1) = cfg.optim.weight_kinematics_turning_radius;
    Eigen::Matrix<double, 1, 1> steering_information;
    steering_information << cfg.optim.weight_steering_kinematics;
    Eigen::Matrix<double, 1, 1> rate_information;
    rate_information << cfg.optim.weight_steering_rate;
    Eigen::Matrix<double, 1, 1> acceleration_information;
    acceleration_information << cfg.optim.weight_steering_acceleration;
    int edge_id = 2000;
    for (size_t index = 0; index + 1 < vertices.size(); ++index)
    {
      EdgeKinematicsCarlike * carlike = new EdgeKinematicsCarlike();
      carlike->setId(edge_id++);
      carlike->setVertex(0, vertices[index]);
      carlike->setVertex(1, vertices[index + 1]);
      carlike->setInformation(carlike_information);
      carlike->setTebConfig(cfg);
      optimizer.addEdge(carlike);

      EdgeSteeringKinematics * steering = new EdgeSteeringKinematics();
      steering->setId(edge_id++);
      steering->setVertex(0, vertices[index]);
      steering->setVertex(1, vertices[index + 1]);
      steering->setVertex(2, times[index]);
      steering->setInformation(steering_information);
      steering->setTebConfig(cfg);
      optimizer.addEdge(steering);
    }
    for (size_t index = 0; index + 2 < vertices.size(); ++index)
    {
      EdgeSteeringRate * rate = new EdgeSteeringRate();
      rate->setId(edge_id++);
      rate->setVertex(0, vertices[index]);
      rate->setVertex(1, vertices[index + 1]);
      rate->setVertex(2, vertices[index + 2]);
      rate->setVertex(3, times[index]);
      rate->setVertex(4, times[index + 1]);
      rate->setInformation(rate_information);
      rate->setTebConfig(cfg);
      optimizer.addEdge(rate);
    }
    for (size_t index = 0; index + 3 < vertices.size(); ++index)
    {
      EdgeSteeringAcceleration * acceleration = new EdgeSteeringAcceleration();
      acceleration->setId(edge_id++);
      acceleration->setVertex(0, vertices[index]);
      acceleration->setVertex(1, vertices[index + 1]);
      acceleration->setVertex(2, vertices[index + 2]);
      acceleration->setVertex(3, vertices[index + 3]);
      acceleration->setVertex(4, times[index]);
      acceleration->setVertex(5, times[index + 1]);
      acceleration->setVertex(6, times[index + 2]);
      acceleration->setInformation(acceleration_information);
      acceleration->setTebConfig(cfg);
      optimizer.addEdge(acceleration);
    }
    optimizer.initializeOptimization();
    const int iterations = optimizer.optimize(60);
    plot.arc_optimized = iterations > 0;
    plot.optimized_arc.reserve(vertices.size());
    for (size_t index = 0; index < vertices.size(); ++index)
    {
      const PoseSE2 pose = vertices[index]->pose();
      plot.optimized_arc.push_back(pose);
      const double heading_error = std::abs(
        g2o::normalize_theta(pose.theta() - plot.feasible_arc[index].theta()));
      const double position_error = (pose.position() - plot.feasible_arc[index].position()).norm();
      plot.max_heading_error = std::max(plot.max_heading_error, heading_error);
      plot.max_position_error = std::max(plot.max_position_error, position_error);
    }
    for (size_t index = 0; index + 1 < plot.optimized_arc.size(); ++index)
    {
      SegmentKinematics segment;
      computeSteeringSegment(
        plot.optimized_arc[index], plot.optimized_arc[index + 1], times[index]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, segment);
      plot.optimized_commands.push_back(
        sampleFromSegment(plot.optimized_arc[index], segment, cfg.robot.wheelbase, cfg.robot.rear_track));
      plot.max_abs_delta = std::max(plot.max_abs_delta, std::abs(segment.steering_angle));
      if (std::abs(segment.velocity) > cfg.robot.steering_min_speed)
      {
        const double model_omega =
          segment.velocity * std::tan(segment.steering_angle) / cfg.robot.wheelbase;
        plot.max_model_omega_error = std::max(
          plot.max_model_omega_error, std::abs(segment.omega - model_omega));
      }
    }
    for (size_t index = 0; index + 1 < plot.optimized_commands.size(); ++index)
    {
      SegmentKinematics first;
      SegmentKinematics second;
      computeSteeringSegment(
        plot.optimized_arc[index], plot.optimized_arc[index + 1], times[index]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, first);
      computeSteeringSegment(
        plot.optimized_arc[index + 1], plot.optimized_arc[index + 2], times[index + 1]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, second);
      plot.max_abs_rate = std::max(plot.max_abs_rate, std::abs(steeringRate(first, second)));
    }
    for (size_t index = 0; index + 2 < plot.optimized_commands.size(); ++index)
    {
      SegmentKinematics first;
      SegmentKinematics second;
      SegmentKinematics third;
      computeSteeringSegment(
        plot.optimized_arc[index], plot.optimized_arc[index + 1], times[index]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, first);
      computeSteeringSegment(
        plot.optimized_arc[index + 1], plot.optimized_arc[index + 2], times[index + 1]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, second);
      computeSteeringSegment(
        plot.optimized_arc[index + 2], plot.optimized_arc[index + 3], times[index + 2]->dt(),
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, third);
      plot.max_abs_acceleration = std::max(
        plot.max_abs_acceleration, std::abs(steeringAcceleration(first, second, third)));
    }
  }

  {
    PoseSE2 first(0.0, 0.0, 0.0);
    PoseSE2 second = poseAfterLongitudinalSegment(first, 0.4, 0.0, cfg.robot.wheelbase);
    PoseSE2 third = poseAfterLongitudinalSegment(second, 0.4, 0.1, cfg.robot.wheelbase);
    const double initial_dt = 0.1;
    SegmentKinematics before_first;
    SegmentKinematics before_second;
    computeSteeringSegment(
      first, second, initial_dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
      cfg.robot.steering_omega_epsilon, before_first);
    computeSteeringSegment(
      second, third, initial_dt, cfg.robot.wheelbase, cfg.robot.steering_min_speed,
      cfg.robot.steering_omega_epsilon, before_second);
    plot.rate_before = steeringRate(before_first, before_second);

    g2o::SparseOptimizer optimizer;
    configureOptimizer(optimizer);
    VertexPose * pose1 = new VertexPose(first, true);
    VertexPose * pose2 = new VertexPose(second, true);
    VertexPose * pose3 = new VertexPose(third, true);
    pose1->setId(0);
    pose2->setId(1);
    pose3->setId(2);
    optimizer.addVertex(pose1);
    optimizer.addVertex(pose2);
    optimizer.addVertex(pose3);
    VertexTimeDiff * dt1 = new VertexTimeDiff(initial_dt, false);
    VertexTimeDiff * dt2 = new VertexTimeDiff(initial_dt, false);
    dt1->setId(3);
    dt2->setId(4);
    dt1->setMinDt(0.05);
    dt2->setMinDt(0.05);
    optimizer.addVertex(dt1);
    optimizer.addVertex(dt2);
    EdgeSteeringRate * rate = new EdgeSteeringRate();
    rate->setId(5);
    rate->setVertex(0, pose1);
    rate->setVertex(1, pose2);
    rate->setVertex(2, pose3);
    rate->setVertex(3, dt1);
    rate->setVertex(4, dt2);
    Eigen::Matrix<double, 1, 1> rate_information;
    rate_information << 300.0;
    rate->setInformation(rate_information);
    rate->setTebConfig(cfg);
    optimizer.addEdge(rate);
    Eigen::Matrix<double, 1, 1> time_information;
    time_information << 1.0;
    EdgeTimeOptimal * time1 = new EdgeTimeOptimal();
    time1->setId(6);
    time1->setVertex(0, dt1);
    time1->setInformation(time_information);
    time1->setTebConfig(cfg);
    optimizer.addEdge(time1);
    EdgeTimeOptimal * time2 = new EdgeTimeOptimal();
    time2->setId(7);
    time2->setVertex(0, dt2);
    time2->setInformation(time_information);
    time2->setTebConfig(cfg);
    optimizer.addEdge(time2);
    optimizer.initializeOptimization();
    const int iterations = optimizer.optimize(30);
    plot.rate_optimized = iterations > 0;
    SegmentKinematics after_first;
    SegmentKinematics after_second;
    computeSteeringSegment(
      pose1->pose(), pose2->pose(), dt1->dt(), cfg.robot.wheelbase, cfg.robot.steering_min_speed,
      cfg.robot.steering_omega_epsilon, after_first);
    computeSteeringSegment(
      pose2->pose(), pose3->pose(), dt2->dt(), cfg.robot.wheelbase, cfg.robot.steering_min_speed,
      cfg.robot.steering_omega_epsilon, after_second);
    plot.rate_after = steeringRate(after_first, after_second);
    plot.time_stretched_commands.push_back(
      sampleFromSegment(pose1->pose(), after_first, cfg.robot.wheelbase, cfg.robot.rear_track));
    plot.time_stretched_commands.push_back(
      sampleFromSegment(pose2->pose(), after_second, cfg.robot.wheelbase, cfg.robot.rear_track));
  }

  plot.tight_error_before = kinematicsError(cfg, plot.tight_arc.front(), plot.tight_arc[1], 0.2);
  {
    g2o::SparseOptimizer optimizer;
    configureOptimizer(optimizer);
    std::vector<VertexPose *> vertices;
    std::vector<VertexTimeDiff *> times;
    for (size_t index = 0; index < plot.tight_arc.size(); ++index)
    {
      VertexPose * vertex = new VertexPose(plot.tight_arc[index], true);
      vertex->setId(static_cast<int>(index));
      optimizer.addVertex(vertex);
      vertices.push_back(vertex);
    }
    for (size_t index = 0; index + 1 < vertices.size(); ++index)
    {
      VertexTimeDiff * timediff = new VertexTimeDiff(0.1, false);
      timediff->setId(50 + static_cast<int>(index));
      timediff->setMinDt(0.02);
      optimizer.addVertex(timediff);
      times.push_back(timediff);
      EdgeSteeringKinematics * steering = new EdgeSteeringKinematics();
      steering->setId(80 + static_cast<int>(index));
      steering->setVertex(0, vertices[index]);
      steering->setVertex(1, vertices[index + 1]);
      steering->setVertex(2, timediff);
      Eigen::Matrix<double, 1, 1> information;
      information << 500.0;
      steering->setInformation(information);
      steering->setTebConfig(cfg);
      optimizer.addEdge(steering);
      EdgeTimeOptimal * time = new EdgeTimeOptimal();
      time->setId(90 + static_cast<int>(index));
      time->setVertex(0, timediff);
      Eigen::Matrix<double, 1, 1> time_information;
      time_information << 1.0;
      time->setInformation(time_information);
      time->setTebConfig(cfg);
      optimizer.addEdge(time);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    plot.tight_error_after = kinematicsError(cfg, vertices[0]->pose(), vertices[1]->pose(), times[0]->dt());
  }

  std::ostringstream text;
  text << "圆弧优化航向误差 " << plot.max_heading_error
       << " rad，转角 " << plot.max_abs_delta
       << " rad，模型角速度误差 " << plot.max_model_omega_error
       << "；转角速度 " << plot.rate_before << " -> " << plot.rate_after
       << " rad/s；过小半径误差 " << plot.tight_error_before
       << " -> " << plot.tight_error_after;
  plot.summary = text.str();
  return plot;
}

PlanCurvatureOptimization optimizePlanCurvatureSegment(int index)
{
  PlanCurvatureOptimization result;
  const PlanLogSegment & segment = planLogHighCurvatureSegment(index);
  result.name = segment.name;
  result.path_index = segment.path_index;
  result.start_index = segment.start_index;
  result.logged_peak_curvature = segment.logged_peak_curvature;
  result.turn_angle = segment.turn_angle;
  result.reference.reserve(static_cast<size_t>(segment.count));
  for (int pose_index = 0; pose_index < segment.count; ++pose_index)
  {
    const PlanLogPose & pose = segment.poses[pose_index];
    result.reference.push_back(PoseSE2(pose.x, pose.y, pose.theta));
  }
  if (result.reference.size() < 4)
  {
    return result;
  }

  TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  g2o::SparseOptimizer optimizer;
  configureOptimizer(optimizer);
  std::vector<VertexPose *> vertices;
  vertices.reserve(result.reference.size());
  for (size_t pose_index = 0; pose_index < result.reference.size(); ++pose_index)
  {
    const bool fixed = pose_index == 0 || pose_index + 1 == result.reference.size();
    const PoseSE2 & reference = result.reference[pose_index];
    VertexPose * vertex = new VertexPose(reference, fixed);
    vertex->setId(static_cast<int>(pose_index));
    optimizer.addVertex(vertex);
    vertices.push_back(vertex);
    if (!fixed)
    {
      EdgePosePositionAnchor * anchor = new EdgePosePositionAnchor();
      anchor->setId(1000 + static_cast<int>(pose_index));
      anchor->setVertex(0, vertex);
      anchor->setMeasurement(reference.position());
      Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * 80.0;
      anchor->setInformation(information);
      optimizer.addEdge(anchor);

      EdgePoseHeadingAnchor * heading = new EdgePoseHeadingAnchor();
      heading->setId(2000 + static_cast<int>(pose_index));
      heading->setVertex(0, vertex);
      heading->setMeasurement(reference.theta());
      Eigen::Matrix<double, 1, 1> heading_information;
      heading_information << 20.0;
      heading->setInformation(heading_information);
      optimizer.addEdge(heading);
    }
  }

  std::vector<VertexTimeDiff *> times;
  for (size_t pose_index = 0; pose_index + 1 < vertices.size(); ++pose_index)
  {
    const double distance = (result.reference[pose_index + 1].position() -
      result.reference[pose_index].position()).norm();
    const double dt = std::max(0.08, distance / 0.25);
    VertexTimeDiff * timediff = new VertexTimeDiff(dt, false);
    timediff->setId(3000 + static_cast<int>(pose_index));
    timediff->setMinDt(0.04);
    optimizer.addVertex(timediff);
    times.push_back(timediff);
  }

  Eigen::Matrix<double, 2, 2> carlike_information;
  carlike_information.setZero();
  carlike_information(0, 0) = cfg.optim.weight_kinematics_nh;
  carlike_information(1, 1) = cfg.optim.weight_kinematics_turning_radius;
  Eigen::Matrix<double, 1, 1> steering_information;
  steering_information << cfg.optim.weight_steering_kinematics;
  Eigen::Matrix<double, 1, 1> rate_information;
  rate_information << cfg.optim.weight_steering_rate;
  Eigen::Matrix<double, 1, 1> acceleration_information;
  acceleration_information << cfg.optim.weight_steering_acceleration;
  Eigen::Matrix<double, 1, 1> time_information;
  time_information << 0.2;
  int edge_id = 4000;
  for (size_t pose_index = 0; pose_index + 1 < vertices.size(); ++pose_index)
  {
    EdgeKinematicsCarlike * carlike = new EdgeKinematicsCarlike();
    carlike->setId(edge_id++);
    carlike->setVertex(0, vertices[pose_index]);
    carlike->setVertex(1, vertices[pose_index + 1]);
    carlike->setInformation(carlike_information);
    carlike->setTebConfig(cfg);
    optimizer.addEdge(carlike);

    EdgeSteeringKinematics * steering = new EdgeSteeringKinematics();
    steering->setId(edge_id++);
    steering->setVertex(0, vertices[pose_index]);
    steering->setVertex(1, vertices[pose_index + 1]);
    steering->setVertex(2, times[pose_index]);
    steering->setInformation(steering_information);
    steering->setTebConfig(cfg);
    optimizer.addEdge(steering);

    EdgeTimeOptimal * time_edge = new EdgeTimeOptimal();
    time_edge->setId(edge_id++);
    time_edge->setVertex(0, times[pose_index]);
    time_edge->setInformation(time_information);
    time_edge->setTebConfig(cfg);
    optimizer.addEdge(time_edge);
  }
  for (size_t pose_index = 0; pose_index + 2 < vertices.size(); ++pose_index)
  {
    EdgeSteeringRate * rate = new EdgeSteeringRate();
    rate->setId(edge_id++);
    rate->setVertex(0, vertices[pose_index]);
    rate->setVertex(1, vertices[pose_index + 1]);
    rate->setVertex(2, vertices[pose_index + 2]);
    rate->setVertex(3, times[pose_index]);
    rate->setVertex(4, times[pose_index + 1]);
    rate->setInformation(rate_information);
    rate->setTebConfig(cfg);
    optimizer.addEdge(rate);
  }
  for (size_t pose_index = 0; pose_index + 3 < vertices.size(); ++pose_index)
  {
    EdgeSteeringAcceleration * acceleration = new EdgeSteeringAcceleration();
    acceleration->setId(edge_id++);
    acceleration->setVertex(0, vertices[pose_index]);
    acceleration->setVertex(1, vertices[pose_index + 1]);
    acceleration->setVertex(2, vertices[pose_index + 2]);
    acceleration->setVertex(3, vertices[pose_index + 3]);
    acceleration->setVertex(4, times[pose_index]);
    acceleration->setVertex(5, times[pose_index + 1]);
    acceleration->setVertex(6, times[pose_index + 2]);
    acceleration->setInformation(acceleration_information);
    acceleration->setTebConfig(cfg);
    optimizer.addEdge(acceleration);
  }

  optimizer.initializeOptimization();
  const int iterations = optimizer.optimize(25);
  result.succeeded = iterations > 0;
  result.optimized.reserve(vertices.size());
  for (size_t pose_index = 0; pose_index < vertices.size(); ++pose_index)
  {
    const PoseSE2 pose = vertices[pose_index]->pose();
    result.optimized.push_back(pose);
    const double position_error =
      (pose.position() - result.reference[pose_index].position()).norm();
    result.max_position_error = std::max(result.max_position_error, position_error);
  }
  for (size_t pose_index = 0; pose_index + 1 < result.optimized.size(); ++pose_index)
  {
    SegmentKinematics reference_segment;
    SegmentKinematics optimized_segment;
    computeSteeringSegment(
      result.reference[pose_index], result.reference[pose_index + 1], times[pose_index]->dt(),
      cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon,
      reference_segment);
    computeSteeringSegment(
      result.optimized[pose_index], result.optimized[pose_index + 1], times[pose_index]->dt(),
      cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon,
      optimized_segment);
    result.peak_reference_curvature = std::max(
      result.peak_reference_curvature, std::abs(reference_segment.curvature));
    result.max_optimized_delta = std::max(
      result.max_optimized_delta, std::abs(optimized_segment.steering_angle));
    result.max_abs_lateral = std::max(
      result.max_abs_lateral, std::abs(optimized_segment.lateral_displacement));
    result.commands.push_back(sampleFromSegment(
      result.optimized[pose_index], optimized_segment, cfg.robot.wheelbase, cfg.robot.rear_track));
    if (std::abs(optimized_segment.velocity) > cfg.robot.steering_min_speed)
    {
      const double model_omega = optimized_segment.velocity *
        std::tan(optimized_segment.steering_angle) / cfg.robot.wheelbase;
      result.max_model_omega_error = std::max(
        result.max_model_omega_error, std::abs(optimized_segment.omega - model_omega));
    }
  }
  for (size_t pose_index = 0; pose_index + 1 < result.commands.size(); ++pose_index)
  {
    SegmentKinematics first;
    SegmentKinematics second;
    computeSteeringSegment(
      result.optimized[pose_index], result.optimized[pose_index + 1], times[pose_index]->dt(),
      cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, first);
    computeSteeringSegment(
      result.optimized[pose_index + 1], result.optimized[pose_index + 2], times[pose_index + 1]->dt(),
      cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, second);
    result.max_abs_rate = std::max(result.max_abs_rate, std::abs(steeringRate(first, second)));
  }
  return result;
}

PlanPathSimulation simulateFullPlanPath(int index)
{
  PlanPathSimulation result;
  const PlanLogSegment & segment = planLogFullPath(index);
  result.name = segment.name;
  result.path_index = segment.path_index;
  result.pose_count = segment.count;
  result.reference.reserve(static_cast<size_t>(segment.count));
  for (int pose_index = 0; pose_index < segment.count; ++pose_index)
  {
    const PlanLogPose & pose = segment.poses[pose_index];
    result.reference.push_back(PoseSE2(pose.x, pose.y, pose.theta));
  }
  if (result.reference.size() < 2)
  {
    return result;
  }

  TebConfig cfg;
  fillFrontSteeringConfig(cfg);
  std::vector<double> time_steps;
  time_steps.reserve(result.reference.size() - 1);
  for (size_t pose_index = 0; pose_index + 1 < result.reference.size(); ++pose_index)
  {
    const double spacing = (result.reference[pose_index + 1].position() -
      result.reference[pose_index].position()).norm();
    result.path_length += spacing;
    result.max_sample_spacing = std::max(result.max_sample_spacing, spacing);
    time_steps.push_back(std::max(spacing / kPlanPathReferenceSpeed, 1e-3));
  }

  SegmentKinematics seed;
  if (!computeSteeringSegment(
      result.reference[0], result.reference[1], time_steps[0], cfg.robot.wheelbase,
      cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon, seed))
  {
    return result;
  }
  FrontSteeringCommandState previous;
  previous.valid = true;
  previous.steering_angle = seed.steering_angle;
  previous.steering_rate = 0.0;

  PoseSE2 simulated = result.reference.front();
  result.simulated.push_back(simulated);
  result.nearest_reference.push_back(result.reference.front());
  std::vector<SegmentKinematics> executed;
  executed.reserve(time_steps.size());
  double cross_track_square_sum = 0.0;
  bool model_ok = true;

  for (size_t pose_index = 0; pose_index + 1 < result.reference.size(); ++pose_index)
  {
    SegmentKinematics geometric;
    if (!computeSteeringSegment(
        result.reference[pose_index], result.reference[pose_index + 1], time_steps[pose_index],
        cfg.robot.wheelbase, cfg.robot.steering_min_speed, cfg.robot.steering_omega_epsilon,
        geometric))
    {
      return result;
    }
    result.max_reference_lateral = std::max(
      result.max_reference_lateral, std::abs(geometric.lateral_displacement));

    const FrontSteeringCommand command = projectFrontSteeringCommand(
      geometric.velocity, 0.0, geometric.omega, time_steps[pose_index], cfg.robot, previous);
    previous.valid = true;
    previous.steering_angle = command.steering_angle;
    previous.steering_rate = command.steering_rate;

    if (std::abs(command.vy) > 1e-12)
    {
      model_ok = false;
    }
    if (std::abs(command.vx) > cfg.robot.steering_min_speed)
    {
      const double model_omega = command.vx * std::tan(command.steering_angle) / cfg.robot.wheelbase;
      result.max_model_omega_error = std::max(
        result.max_model_omega_error, std::abs(command.omega - model_omega));
    }
    else
    {
      result.max_model_omega_error = std::max(result.max_model_omega_error, std::abs(command.omega));
    }
    result.max_abs_delta = std::max(result.max_abs_delta, std::abs(command.steering_angle));
    if (std::abs(command.steering_angle - geometric.steering_angle) > 1e-4 ||
      std::abs(command.omega - geometric.omega) > 1e-4)
    {
      ++result.saturated_steps;
    }

    FrontSteeringSample sample;
    sample.x = simulated.x();
    sample.y = simulated.y();
    sample.theta = simulated.theta();
    sample.v = command.vx;
    sample.omega = command.omega;
    sample.steering_angle = command.steering_angle;
    sample.segment_dt = time_steps[pose_index];
    sample.rear_left = command.rear_left;
    sample.rear_right = command.rear_right;
    result.commands.push_back(sample);

    SegmentKinematics step;
    step.dt = time_steps[pose_index];
    step.steering_angle = command.steering_angle;
    step.velocity = command.vx;
    step.omega = command.omega;
    step.valid = true;
    executed.push_back(step);

    simulated = poseAfterLongitudinalSegment(
      simulated, command.vx * time_steps[pose_index], command.steering_angle, cfg.robot.wheelbase);
    result.simulated.push_back(simulated);
    const ReferenceProjection projection = projectOntoReference(simulated, result.reference);
    result.nearest_reference.push_back(PoseSE2(projection.x, projection.y, projection.theta));
    result.max_cross_track = std::max(result.max_cross_track, projection.distance);
    cross_track_square_sum += projection.distance * projection.distance;
    result.max_heading_error = std::max(
      result.max_heading_error,
      std::abs(g2o::normalize_theta(simulated.theta() - result.reference[pose_index + 1].theta())));
  }

  const double sample_count = static_cast<double>(result.simulated.size() - 1);
  result.rms_cross_track = std::sqrt(cross_track_square_sum / sample_count);
  result.final_position_error =
    (result.simulated.back().position() - result.reference.back().position()).norm();
  for (size_t step_index = 0; step_index + 1 < executed.size(); ++step_index)
  {
    const double rate = steeringRate(executed[step_index], executed[step_index + 1]);
    result.commands[step_index].steering_rate = rate;
    result.max_abs_rate = std::max(result.max_abs_rate, std::abs(rate));
  }
  if (result.commands.size() >= 2)
  {
    result.commands.back().steering_rate = result.commands[result.commands.size() - 2].steering_rate;
  }
  for (size_t step_index = 0; step_index + 2 < executed.size(); ++step_index)
  {
    const double acceleration = steeringAcceleration(
      executed[step_index], executed[step_index + 1], executed[step_index + 2]);
    result.commands[step_index].steering_acceleration = acceleration;
    result.max_abs_acceleration = std::max(result.max_abs_acceleration, std::abs(acceleration));
  }
  if (result.commands.size() >= 3)
  {
    const double last_acceleration = result.commands[result.commands.size() - 3].steering_acceleration;
    result.commands[result.commands.size() - 2].steering_acceleration = last_acceleration;
    result.commands.back().steering_acceleration = last_acceleration;
  }
  const bool within_angle = result.max_abs_delta <= cfg.robot.max_steering_angle + 1e-9;
  const bool within_rate = result.max_abs_rate <= cfg.robot.max_steering_rate + 1e-6;
  const bool within_acceleration =
    result.max_abs_acceleration <= cfg.robot.max_steering_acceleration + 1e-6;
  result.commands_satisfy_model = model_ok && within_angle && within_rate && within_acceleration &&
    result.max_model_omega_error <= 1e-9;
  result.succeeded = result.simulated.size() == result.reference.size() &&
    result.commands.size() + 1 == result.reference.size();
  return result;
}

}  // namespace teb_local_planner
