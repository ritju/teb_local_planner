#ifndef FRONT_STEERING_PLOT_HPP_
#define FRONT_STEERING_PLOT_HPP_

#include "teb_local_planner/pose_se2.h"

#include <string>
#include <vector>

namespace teb_local_planner
{

struct FrontSteeringSample
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v{0.0};
  double omega{0.0};
  double steering_angle{0.0};
  double steering_rate{0.0};
  double steering_acceleration{0.0};
  double segment_dt{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

struct FrontSteeringVerificationPlot
{
  std::vector<PoseSE2> straight;
  std::vector<PoseSE2> feasible_arc;
  std::vector<PoseSE2> tight_arc;
  std::vector<PoseSE2> optimized_arc;
  std::vector<FrontSteeringSample> optimized_commands;
  std::vector<FrontSteeringSample> time_stretched_commands;
  bool arc_optimized{false};
  bool rate_optimized{false};
  double max_abs_delta{0.0};
  double max_abs_rate{0.0};
  double max_abs_acceleration{0.0};
  double max_heading_error{0.0};
  double max_position_error{0.0};
  double max_model_omega_error{0.0};
  double rate_before{0.0};
  double rate_after{0.0};
  double tight_error_before{0.0};
  double tight_error_after{0.0};
  std::string summary;
};

FrontSteeringVerificationPlot buildFrontSteeringVerificationPlot();

struct PlanCurvatureOptimization
{
  std::string name;
  int path_index{0};
  int start_index{0};
  double logged_peak_curvature{0.0};
  double turn_angle{0.0};
  std::vector<PoseSE2> reference;
  std::vector<PoseSE2> optimized;
  std::vector<FrontSteeringSample> commands;
  double peak_reference_curvature{0.0};
  double max_optimized_delta{0.0};
  double max_abs_rate{0.0};
  double max_position_error{0.0};
  double max_model_omega_error{0.0};
  double max_abs_lateral{0.0};
  bool succeeded{false};
};

PlanCurvatureOptimization optimizePlanCurvatureSegment(int index);

constexpr double kPlanPathReferenceSpeed = 0.25;

struct PlanPathSimulation
{
  std::string name;
  int path_index{0};
  int pose_count{0};
  double reference_speed{kPlanPathReferenceSpeed};
  double path_length{0.0};
  double max_sample_spacing{0.0};
  std::vector<PoseSE2> reference;
  std::vector<PoseSE2> simulated;
  std::vector<PoseSE2> nearest_reference;
  std::vector<FrontSteeringSample> commands;
  double max_cross_track{0.0};
  double rms_cross_track{0.0};
  double max_heading_error{0.0};
  double final_position_error{0.0};
  double max_reference_lateral{0.0};
  double max_abs_delta{0.0};
  double max_abs_rate{0.0};
  double max_abs_acceleration{0.0};
  double max_model_omega_error{0.0};
  int saturated_steps{0};
  bool commands_satisfy_model{false};
  bool succeeded{false};
};

PlanPathSimulation simulateFullPlanPath(int index);

}  // namespace teb_local_planner

#endif
