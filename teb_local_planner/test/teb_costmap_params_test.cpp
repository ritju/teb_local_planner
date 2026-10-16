#include "split_module_fixture.hpp"
#include "split_module_scenarios.hpp"

#include "teb_local_planner/detail/async_wait.hpp"

#include <atomic>
#include <future>
#include <thread>

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
