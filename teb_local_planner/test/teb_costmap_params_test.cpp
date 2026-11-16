#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/async_wait.hpp"

#include <atomic>
#include <future>
#include <thread>
#include <rcutils/logging.h>

TEST(ControllerDebugMode, loadsRootOverrideAndUpdatesEveryPluginWithoutChangingSeverity)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.parameter_overrides({rclcpp::Parameter("debug_mode", true)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("controller_debug_mode_test", "", options);
  teb_local_planner::TebConfig first;
  teb_local_planner::TebConfig second;
  first.declareParameters(node, "FollowPath");
  second.declareParameters(node, "OtherController");
  first.loadRosParamFromNodeHandle(node, "FollowPath");
  second.loadRosParamFromNodeHandle(node, "OtherController");
  ASSERT_TRUE(first.debug_mode.load());
  ASSERT_TRUE(second.debug_mode.load());
  const auto severity = rcutils_logging_get_logger_effective_level(node->get_logger().get_name());
  auto first_callback = node->add_on_set_parameters_callback(
    [&first](const std::vector<rclcpp::Parameter> & params) {
      return first.dynamicParametersCallback(params);
    });
  auto second_callback = node->add_on_set_parameters_callback(
    [&second](const std::vector<rclcpp::Parameter> & params) {
      return second.dynamicParametersCallback(params);
    });
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("debug_mode", false)).successful);
  EXPECT_FALSE(first.debug_mode.load());
  EXPECT_FALSE(second.debug_mode.load());
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("debug_mode", true)).successful);
  EXPECT_TRUE(first.debug_mode.load());
  EXPECT_TRUE(second.debug_mode.load());
  EXPECT_EQ(rcutils_logging_get_logger_effective_level(node->get_logger().get_name()), severity);
}

TEST(AsyncWait, joinAndReadyFuture)
{
  std::thread idle;
  teb_local_planner::detail::joinThreadQuietly(idle);
  teb_local_planner::detail::detachThreadQuietly(idle);

  std::promise<void> promise;
  auto future = promise.get_future();
  promise.set_value();
  const auto data = teb_local_planner::test_data::costmap();
  EXPECT_TRUE(teb_local_planner::detail::waitForFutureInterruptible(
    future, std::chrono::milliseconds(data.join_wait_ms), []() { return true; }));
}

TEST_F(SplitModuleFixture, backgroundTaskRunsAndNullServiceWaitsFail)
{
  EXPECT_TRUE(teb_local_planner::TebSplitModuleTestAccess::isBackgroundWorkAllowed(*planner_));
  const auto data = teb_local_planner::test_data::costmap();
  EXPECT_FALSE(teb_local_planner::TebSplitModuleTestAccess::waitForServiceInterruptible(
    *planner_, nullptr, std::chrono::milliseconds(data.service_wait_ms)));

  std::atomic<int> ran{0};
  for (int i = 0; i < data.task_count; ++i) {
    teb_local_planner::TebSplitModuleTestAccess::spawnBackgroundTask(
      *planner_, [&ran]() { ran.fetch_add(1); });
  }
  teb_local_planner::TebSplitModuleTestAccess::joinBackgroundTasks(
    *planner_, std::chrono::milliseconds(data.join_wait_ms));
  EXPECT_EQ(ran.load(), data.task_count);
}
