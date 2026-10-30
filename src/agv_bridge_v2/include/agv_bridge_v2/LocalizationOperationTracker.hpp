#pragma once

#include <agv_bridge_v2_interfaces/msg/localization_operation.hpp>
#include <agv_bridge_v2_interfaces/srv/get_localization_operation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace agv_bridge {
// Only tracks explicitly requested load_map/relocalize operations. No process
// control lives here. All updates are ID-scoped so late workers cannot rewrite
// terminal results or a newer operation. History is bounded and process-local.
class LocalizationOperationTracker {
 public:
  using Status = agv_bridge_v2_interfaces::msg::LocalizationOperation;
  using Query = agv_bridge_v2_interfaces::srv::GetLocalizationOperation;
  using Clock = std::chrono::steady_clock;

  explicit LocalizationOperationTracker(rclcpp::Node *node) : node_(node) {
    timeout_ = node_->declare_parameter("localization_operation_timeout_sec", 600.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0)
      throw std::invalid_argument("localization_operation_timeout_sec must be positive and finite");
    publisher_ = node_->create_publisher<Status>("/agv/localization_operation",
        rclcpp::QoS(1).reliable().transient_local());
    service_ = node_->create_service<Query>("/agv/get_localization_operation",
      [this](const Query::Request::SharedPtr req, Query::Response::SharedPtr res) {
        res->found = lookup(req->operation_id, res->operation);
        res->message = res->found ? "操作记录已找到" :
            "未找到操作：ID不存在、超过最近128条记录或bridge已重启；请勿据此推断成功";
      });
  }

  bool active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !history_.empty() && !history_.back().finished;
  }

  bool allowsModeCompletion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    // The independent observer must commit SUCCEEDED before mode can reopen;
    // otherwise a ready TF arriving after the deadline could race the timeout.
    return history_.empty() || history_.back().success;
  }

  void failActive(const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!history_.empty() && !history_.back().finished) finishLocked("FAILED", reason);
  }

  std::string begin(const std::string &type, const std::string &map) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!history_.empty() && !history_.back().finished)
      throw std::logic_error("localization operation already running");
    Status status;
    status.operation_id = std::to_string(::getpid()) + "-" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
        "-" + std::to_string(++sequence_);
    status.operation_type = type;
    status.map_name = map;
    status.state = "RUNNING";
    status.stage = "ACCEPTED";
    status.message = "请求已受理，尚未完成定位";
    started_ = Clock::now();
    history_.push_back(status);
    if (history_.size() > 128) history_.pop_front();
    publishLocked();
    return status.operation_id;
  }

  void stage(const std::string &id, const std::string &stage, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!matchesLocked(id)) return;
    auto &status = history_.back();
    status.stage = stage;
    status.message = message;
    publishLocked();
  }

  void fail(const std::string &id, const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (matchesLocked(id)) finishLocked("FAILED", reason);
  }

  // Called from the independent wall-clock heartbeat. Success requires the
  // worker's explicit WAITING_LOCALIZATION stage, not merely a fresh old TF.
  void observe(bool pose_ready, bool navigation_ready, bool transitioning) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (history_.empty()) return;
    auto &status = history_.back();
    status.pose_initialized = pose_ready;
    status.navigation_ready = navigation_ready;
    if (!status.finished) {
      const auto elapsed = std::chrono::duration<double>(Clock::now() - started_).count();
      if (elapsed >= timeout_) {
        finishLocked("TIMED_OUT", "定位操作超过 " + std::to_string(timeout_) +
          " 秒，最后阶段=" + status.stage +
          "；请检查定位日志、传感器与TF。超时不代表后台已停止，也不会自动重放请求");
        return;
      }
      if (!transitioning && status.stage == "WAITING_LOCALIZATION" && pose_ready) {
        finishLocked("SUCCEEDED", "目标地图定位检查已通过；导航能否接任务请另查 navigation_ready 或导航就绪检查");
        return;
      }
    }
    // Re-publish the latest terminal result too: reconnecting clients do not
    // have to rely on receiving a short-lived mode transition.
    publishLocked();
  }

  bool lookup(const std::string &id, Status &result) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
      if (id.empty() || it->operation_id == id) {
        result = *it;
        if (!result.finished)
          result.elapsed_sec = std::chrono::duration<double>(Clock::now() - started_).count();
        return true;
      }
    }
    return false;
  }

 private:
  bool matchesLocked(const std::string &id) const {
    return !history_.empty() && history_.back().operation_id == id && !history_.back().finished;
  }
  void finishLocked(const std::string &state, const std::string &message) {
    auto &status = history_.back();
    status.state = state;
    status.finished = true;
    status.success = state == "SUCCEEDED";
    status.message = message;
    // Keep the last progress stage for failure/timeout diagnosis.
    if (status.success) status.stage = "FINISHED";
    status.elapsed_sec = std::chrono::duration<double>(Clock::now() - started_).count();
    publishLocked();
  }
  void publishLocked() {
    auto &status = history_.back();
    status.header.stamp = node_->now();
    if (!status.finished)
      status.elapsed_sec = std::chrono::duration<double>(Clock::now() - started_).count();
    publisher_->publish(status);
  }
  rclcpp::Node *node_;
  double timeout_;
  mutable std::mutex mutex_;
  std::deque<Status> history_;
  Clock::time_point started_;
  unsigned long long sequence_ = 0;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Service<Query>::SharedPtr service_;
};
}  // namespace agv_bridge
