#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <system_error>
#include <thread>

namespace teb_local_planner
{
namespace detail
{
constexpr auto kBackgroundPollSlice = std::chrono::milliseconds(100);

inline void joinThreadQuietly(std::thread & thread)
{
  if (!thread.joinable()) {
    return;
  }
  try {
    thread.join();
  } catch (const std::system_error &) {
  } catch (...) {
  }
}

inline void detachThreadQuietly(std::thread & thread)
{
  if (!thread.joinable()) {
    return;
  }
  try {
    thread.detach();
  } catch (const std::system_error &) {
  } catch (...) {
  }
}

template<typename FutureT>
inline bool waitForFutureInterruptible(
  FutureT & future,
  const std::chrono::milliseconds total_timeout,
  const std::function<bool()> & still_allowed)
{
  const auto deadline = std::chrono::steady_clock::now() + total_timeout;
  while (still_allowed()) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      return false;
    }
    const auto slice =
      remaining < kBackgroundPollSlice ? remaining : kBackgroundPollSlice;
    try {
      if (future.wait_for(slice) == std::future_status::ready) {
        return true;
      }
    } catch (const std::exception &) {
      return false;
    }
  }
  return false;
}

}  // namespace detail
}  // namespace teb_local_planner
