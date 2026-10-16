#pragma once

#include "teb_local_planner/teb_config.h"

#include <nav2_util/lifecycle_node.hpp>

#include <mutex>
#include <stdexcept>
#include <string>

namespace teb_local_planner
{
namespace test_data
{

inline std::string tebParamsYamlPath()
{
#ifdef TEB_PARAMS_YAML
  return TEB_PARAMS_YAML;
#else
  return "params/teb_params.yaml";
#endif
}

inline nav2_util::LifecycleNode::SharedPtr followPathParamNode()
{
  static nav2_util::LifecycleNode::SharedPtr node = []() {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.automatically_declare_parameters_from_overrides(true);
    options.arguments(
      {"--ros-args", "--params-file", tebParamsYamlPath()});
    return std::make_shared<nav2_util::LifecycleNode>("controller_server", "", options);
  }();
  return node;
}

inline void loadFollowPathParams(TebConfig & config)
{
  auto node = followPathParamNode();
  if (!node) {
    throw std::runtime_error("无法创建 controller_server 以加载 " + tebParamsYamlPath());
  }
  config.declareParameters(node, "FollowPath");
  config.loadRosParamFromNodeHandle(node, "FollowPath");
}

inline TebConfig & followPathConfig()
{
  static TebConfig config;
  static std::once_flag once;
  std::call_once(once, []() { loadFollowPathParams(config); });
  return config;
}

}  // namespace test_data
}  // namespace teb_local_planner
