/*********************************************************************
 * teb_costmap_params.cpp
 * Split from teb_local_planner_ros.cpp. Behavior unchanged.
 *********************************************************************/
#include "teb_local_planner/teb_local_planner_ros.h"

#include <boost/algorithm/string.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <future>
#include <thread>
#include <system_error>
#include <chrono>
#include <functional>
#include <string>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include "dwb_core/exceptions.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_critics/line_iterator.hpp"
#include "nav2_util/robot_utils.hpp"
#include "teb_local_planner/misc.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <exception>
#include <limits>

#include "teb_local_planner/detail/async_wait.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
using detail::kBackgroundPollSlice;
using detail::joinThreadQuietly;
using detail::detachThreadQuietly;
using detail::waitForFutureInterruptible;

 void TebLocalPlannerROS::setStaticLayerEnabled(bool enabled, bool is_delayed)
 {
   
   if (!isBackgroundWorkAllowed()) {
     return;
   }
   if (!static_layer_client_) {
     RCLCPP_ERROR(logger_, "static_layer_client_ 未初始化，无法设置 static_layer.enabled");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   if (enabled != desired_static_layer_state_.load()) {
     RCLCPP_DEBUG(logger_, "跳过 setStaticLayerEnabled(%s): 当前目标状态已变为 %s",
                  enabled ? "true" : "false", desired_static_layer_state_.load() ? "true" : "false");
     return;
   }

   // Check service availability
   if (!waitForServiceInterruptible(static_layer_client_, std::chrono::seconds(2))) {
     if (isBackgroundWorkAllowed()) {
       RCLCPP_ERROR(logger_, "服务 /local_costmap/local_costmap/set_parameters 不可用");
     }
     return;
   }
   if (!isBackgroundWorkAllowed()) {
     return;
   }

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("static_layer.enabled", enabled).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "调用 /local_costmap/local_costmap/set_parameters 设置 static_layer.enabled -> %s%s",
               enabled ? "true" : "false", is_delayed ? " (延迟)" : "");

   // Use a separate thread to avoid blocking the executor
   spawnBackgroundTask([this, request, enabled]() {
     auto future = static_layer_client_->async_send_request(request);
     try {
       if (!waitForFutureInterruptible(
             future, std::chrono::seconds(2),
             [this]() { return isBackgroundWorkAllowed(); })) {
         return;
       }
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "设置 /local_costmap/local_costmap static_layer.enabled -> %s，结果: 成功",
                     enabled ? "true" : "false");
         current_static_layer_state_.store(enabled);
       } else {
         RCLCPP_ERROR(logger_, "设置 /local_costmap/local_costmap static_layer.enabled -> %s，结果: 失败。原因: %s",
                      enabled ? "true" : "false", failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       if (isBackgroundWorkAllowed()) {
         RCLCPP_ERROR(logger_, "调用 local_costmap set_parameters 服务失败: %s", e.what());
       }
     }
   });
 }

 void TebLocalPlannerROS::armPendingNormalFootprintRestore()
 {
   if (!pending_normal_footprint_restore_) {
     pending_normal_footprint_restore_ = true;
     footprint_restore_start_time_ = clock_->now();
     footprint_restore_safe_count_ = 0;
     RCLCPP_INFO(
       logger_,
       "等待恢复正常 footprint: 需连续 %d 帧安全，或 %.1f s 后强制恢复",
       std::max(1, cfg_->wall_line.footprint_restore_safe_frames),
       cfg_->wall_line.footprint_restore_max_wait_sec);
   }
 }

 void TebLocalPlannerROS::clearPendingNormalFootprintRestore()
 {
   if (pending_normal_footprint_restore_) {
     RCLCPP_DEBUG(logger_, "已清除等待中的正常 footprint 恢复");
   }
   pending_normal_footprint_restore_ = false;
   footprint_restore_safe_count_ = 0;
 }

 bool TebLocalPlannerROS::isNormalFootprintFreeAtRobot() const
 {
   if (!costmap_model_) {
     return false;
   }

   std::string footprint_str = local_costmap_footprint_;
   if (footprint_str.empty()) {
     footprint_str = normal_footprint_vertices_;
   }
   if (footprint_str.empty()) {
     return false;
   }

   std::vector<geometry_msgs::msg::Point> footprint;
   if (!nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint) ||
       footprint.size() < 3)
   {
     return false;
   }

   // 向左右两侧扩展，恢复判定更保守
   const double side_inflate =
     std::max(0.0, cfg_->wall_line.footprint_restore_side_inflate);
   if (side_inflate > 1e-9) {
     for (auto & p : footprint) {
       if (p.y >= 0.0) {
         p.y += side_inflate;
       } else {
         p.y -= side_inflate;
       }
     }
   }

   geometry_msgs::msg::Pose2D pose2d;
   robot_pose_.toPoseMsg(pose2d);
   try {
     (void)costmap_model_->scorePose(
       pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint));
     return true;
   } catch (const dwb_core::IllegalTrajectoryException & e) {
     const char * em = e.what();
     if (!std::strcmp(em, "Trajectory Hits Obstacle.") ||
         !std::strcmp(em, "Trajectory Hits Unknown Region."))
     {
       return false;
     }
     // 出格等无法可靠判定时不视为安全，等待超时强制恢复
     return false;
   } catch (...) {
     return false;
   }
 }

 void TebLocalPlannerROS::updatePendingNormalFootprintRestore()
 {
   if (!pending_normal_footprint_restore_ || !initialized_) {
     return;
   }

   const bool want_local_normal =
     cfg_->wall_line.switch_local_footprint && !desired_local_footprint_state_.load();
   const bool want_global_normal =
     cfg_->wall_line.switch_global_footprint && !desired_global_footprint_state_.load();
   const bool need_local =
     want_local_normal && current_local_footprint_state_.load();
   const bool need_global =
     want_global_normal && current_global_footprint_state_.load();

   if (!need_local && !need_global) {
     clearPendingNormalFootprintRestore();
     return;
   }

   const double elapsed =
     std::max(0.0, (clock_->now() - footprint_restore_start_time_).seconds());
   const double max_wait = std::max(0.0, cfg_->wall_line.footprint_restore_max_wait_sec);
   const bool force_timeout = elapsed >= max_wait;

   const bool safe = isNormalFootprintFreeAtRobot();
   if (safe) {
     ++footprint_restore_safe_count_;
   } else {
     footprint_restore_safe_count_ = 0;
   }

   const int need_frames = std::max(1, cfg_->wall_line.footprint_restore_safe_frames);
   if (!force_timeout && footprint_restore_safe_count_ < need_frames) {
     RCLCPP_INFO_THROTTLE(
       logger_, *(clock_), 2000,
       "等待恢复正常 footprint: 安全帧=%d/%d 已等待=%.1f/%.1fs 无碰撞=%d",
       footprint_restore_safe_count_, need_frames, elapsed, max_wait, safe ? 1 : 0);
     return;
   }

   if (force_timeout) {
     RCLCPP_WARN(
       logger_,
       "等待 %.1f s 后强制恢复正常 footprint (安全帧=%d/%d)",
       elapsed, footprint_restore_safe_count_, need_frames);
   } else {
     RCLCPP_INFO(
       logger_,
       "连续 %d 帧安全，恢复正常 footprint (已等待=%.1fs)",
       footprint_restore_safe_count_, elapsed);
   }

   if (need_local) {
     setLocalFootprintEnabled(false, true);
   }
   if (need_global) {
     setGlobalFootprintEnabled(false, true);
   }
   clearPendingNormalFootprintRestore();
 }

 void TebLocalPlannerROS::setLocalFootprintEnabled(bool enable, bool is_delayed)
 {
   
   if (!isBackgroundWorkAllowed()) {
     return;
   }
   if (!local_footprint_client_) {
     RCLCPP_ERROR(logger_, "local_footprint_client_ 未初始化，无法设置 footprint");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   // enable=true means edge footprint, enable=false means normal footprint
   if (enable == current_local_footprint_state_.load()) {
     RCLCPP_INFO(logger_, "跳过 setLocalFootprintEnabled(%s): 当前状态已是 %s",
                  enable ? "edge" : "normal", current_local_footprint_state_.load() ? "edge" : "normal");
     return;
   }

   // Check service availability
   if (!waitForServiceInterruptible(local_footprint_client_, std::chrono::seconds(2))) {
     if (isBackgroundWorkAllowed()) {
       RCLCPP_ERROR(logger_, "服务 /local_costmap/local_costmap/set_parameters 不可用");
     }
     return;
   }
   if (!isBackgroundWorkAllowed()) {
     return;
   }

   // Determine footprint to set
   std::string footprint_vertices = enable ? cfg_->wall_line.edge_footprint_vertices : local_costmap_footprint_;

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("footprint", footprint_vertices).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "调用 /local_costmap/local_costmap/set_parameters 设置 footprint -> %s%s",
               footprint_vertices.c_str(), is_delayed ? " (延迟)" : "");

   // Use a separate thread to avoid blocking the executor
   spawnBackgroundTask([this, request, footprint_vertices, enable]() {
     auto future = local_footprint_client_->async_send_request(request);
     try {
       if (!waitForFutureInterruptible(
             future, std::chrono::seconds(2),
             [this]() { return isBackgroundWorkAllowed(); })) {
         return;
       }
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "设置 /local_costmap/local_costmap footprint -> %s，结果: 成功",
                     footprint_vertices.c_str());
         current_local_footprint_state_.store(enable);
       } else {
         RCLCPP_ERROR(logger_, "设置 /local_costmap/local_costmap footprint -> %s，结果: 失败。原因: %s",
                      footprint_vertices.c_str(), failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       if (isBackgroundWorkAllowed()) {
         RCLCPP_ERROR(logger_, "调用 local_costmap set_parameters 服务失败: %s", e.what());
       }
     }
   });
 }

 void TebLocalPlannerROS::setGlobalFootprintEnabled(bool enable, bool is_delayed)
 {
   
   if (!isBackgroundWorkAllowed()) {
     return;
   }
   if (!global_footprint_client_) {
     RCLCPP_ERROR(logger_, "global_footprint_client_ 未初始化，无法设置 footprint");
     return;
   }

   // Check if the requested state matches the desired state
   // This prevents outdated timer callbacks from overriding newer state changes
   // enable=true means edge footprint, enable=false means normal footprint
   if (enable == current_global_footprint_state_.load()) {
     RCLCPP_INFO(logger_, "跳过 setGlobalFootprintEnabled(%s): 当前状态已是 %s",
                  enable ? "edge" : "normal", current_global_footprint_state_.load() ? "edge" : "normal");
     return;
   }

   // Check service availability
   if (!waitForServiceInterruptible(global_footprint_client_, std::chrono::seconds(2))) {
     if (isBackgroundWorkAllowed()) {
       RCLCPP_ERROR(logger_, "服务 /global_costmap/global_costmap/set_parameters 不可用");
     }
     return;
   }
   if (!isBackgroundWorkAllowed()) {
     return;
   }

   // Determine footprint to set
   std::string footprint_vertices = enable ? cfg_->wall_line.edge_footprint_vertices : global_costmap_footprint_;

   // Create request
   auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
   rcl_interfaces::msg::Parameter param_msg = rclcpp::Parameter("footprint", footprint_vertices).to_parameter_msg();
   request->parameters.push_back(param_msg);

   RCLCPP_INFO(logger_, "调用 /global_costmap/global_costmap/set_parameters 设置 footprint -> %s%s",
               footprint_vertices.c_str(), is_delayed ? " (延迟)" : "");

   // Use a separate thread to avoid blocking the executor
   spawnBackgroundTask([this, request, footprint_vertices, enable]() {
     auto future = global_footprint_client_->async_send_request(request);
     try {
       if (!waitForFutureInterruptible(
             future, std::chrono::seconds(2),
             [this]() { return isBackgroundWorkAllowed(); })) {
         return;
       }
       auto response = future.get();
       bool all_successful = true;
       std::string failed_reason;
       for (const auto& result : response->results) {
         if (!result.successful) {
           all_successful = false;
           failed_reason = result.reason;
           break;
         }
       }
       if (all_successful) {
         RCLCPP_INFO(logger_, "设置 /global_costmap/global_costmap footprint -> %s，结果: 成功",
                     footprint_vertices.c_str());
         current_global_footprint_state_.store(enable);
       } else {
         RCLCPP_ERROR(logger_, "设置 /global_costmap/global_costmap footprint -> %s，结果: 失败。原因: %s",
                      footprint_vertices.c_str(), failed_reason.empty() ? "unknown" : failed_reason.c_str());
       }
     } catch (const std::exception& e) {
       if (isBackgroundWorkAllowed()) {
         RCLCPP_ERROR(logger_, "调用 global_costmap set_parameters 服务失败: %s", e.what());
       }
     }
   });
 }

bool TebLocalPlannerROS::isBackgroundWorkAllowed() const
{
  return !shutting_down_.load(std::memory_order_acquire) && rclcpp::ok();
}

void TebLocalPlannerROS::spawnBackgroundTask(std::function<void()> task)
{
  if (!isBackgroundWorkAllowed() || !task) {
    return;
  }

  auto finished = std::make_shared<std::atomic<bool>>(false);
  std::thread worker([this, task, finished]() {
    try {
      if (isBackgroundWorkAllowed()) {
        task();
      }
    } catch (const std::exception & ex) {
      if (isBackgroundWorkAllowed()) {
        RCLCPP_WARN(logger_, "后台任务异常: %s", ex.what());
      }
    } catch (...) {
    }
    finished->store(true, std::memory_order_release);
  });

  std::vector<std::thread> finished_threads;
  {
    std::lock_guard<std::mutex> lock(background_threads_mutex_);
    for (size_t i = 0; i < background_jobs_.size();) {
      if (background_jobs_[i].finished &&
          background_jobs_[i].finished->load(std::memory_order_acquire)) {
        finished_threads.push_back(std::move(background_jobs_[i].thread));
        background_jobs_.erase(background_jobs_.begin() + static_cast<long>(i));
      } else {
        ++i;
      }
    }
    BackgroundJob job;
    job.thread = std::move(worker);
    job.finished = finished;
    background_jobs_.push_back(std::move(job));
  }
  for (auto & t : finished_threads) {
    joinThreadQuietly(t);
  }
}

void TebLocalPlannerROS::joinBackgroundTasks(const std::chrono::milliseconds timeout)
{
  std::vector<BackgroundJob> jobs;
  {
    std::lock_guard<std::mutex> lock(background_threads_mutex_);
    jobs.swap(background_jobs_);
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (auto & job : jobs) {
    if (!job.thread.joinable()) {
      continue;
    }
    while (job.finished &&
           !job.finished->load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!job.finished || job.finished->load(std::memory_order_acquire)) {
      joinThreadQuietly(job.thread);
    } else {
      RCLCPP_WARN(
        logger_,
        "关闭时后台任务 %ld ms 内未结束，改为分离 (detach)",
        static_cast<long>(timeout.count()));
      detachThreadQuietly(job.thread);
    }
  }
}

void TebLocalPlannerROS::stopCostmapConverterWorker(const std::chrono::milliseconds timeout)
{
  if (!costmap_converter_) {
    return;
  }

  auto converter = costmap_converter_;
  costmap_converter_.reset();
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread stopper([converter, done]() {
    try {
      converter->stopWorker();
    } catch (const std::exception &) {
    } catch (...) {
    }
    done->store(true, std::memory_order_release);
  });

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done->load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (done->load(std::memory_order_acquire)) {
    joinThreadQuietly(stopper);
  } else {
    RCLCPP_WARN(
      logger_,
      "costmap_converter stopWorker 在 %ld ms 内未结束，改为分离 (detach)",
      static_cast<long>(timeout.count()));
    detachThreadQuietly(stopper);
  }
}

bool TebLocalPlannerROS::waitForServiceInterruptible(
  const rclcpp::ClientBase::SharedPtr & client,
  const std::chrono::milliseconds total_timeout)
{
  if (!client) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + total_timeout;
  while (isBackgroundWorkAllowed()) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      return client->service_is_ready();
    }
    const auto slice =
      remaining < kBackgroundPollSlice ? remaining : kBackgroundPollSlice;
    try {
      if (client->wait_for_service(slice)) {
        return true;
      }
    } catch (const std::exception &) {
      return false;
    }
  }
  return false;
}

}  // namespace teb_local_planner
