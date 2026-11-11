#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <agv_bridge_v2_interfaces/msg/agv_status.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>

namespace agv_bridge {
// Backend contract: success=true means the charging connection has actually
// changed, not merely that an asynchronous hardware command was queued.
class BatteryGateway {
 public:
  BatteryGateway(rclcpp::Node *node, std::function<bool()> busy,
                 std::function<void()> empty)
      : node_(node), busy_(std::move(busy)), empty_(std::move(empty)) {
    required_ = node_->declare_parameter("require_battery_state", false);
    timeout_ = node_->declare_parameter("battery_state_timeout_sec", 5.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0) throw std::invalid_argument("battery_state_timeout_sec must be positive");
    // 证据自愈：uncertain_ 时若接触器真值+电池状态均证明“未连接”，自动解除。
    // 真实硬件若要求严格回执语义可置 false（仿真默认开启）。
    evidence_heal_ = node_->declare_parameter("charging_evidence_heal", true);
    const auto backend = node_->declare_parameter<std::string>("charging_backend_service", "/battery/set_charging");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = node_->create_client<std_srvs::srv::SetBool>(backend, rmw_qos_profile_services_default, group_);
    rclcpp::SubscriptionOptions opts; opts.callback_group = group_;
    battery_ = node_->create_subscription<sensor_msgs::msg::BatteryState>("/battery_state", 10,
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
        bool depleted;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          seen_ = true; received_ = std::chrono::steady_clock::now();
          // Follow the standard 0..1 fraction. Invalid readings never become 0%.
          valid_ = msg->present && std::isfinite(msg->percentage) &&
            msg->percentage >= 0 && msg->percentage <= 1;
          if (valid_) level_ = msg->percentage * 100;
          status_ = msg->power_supply_status;
          depleted = valid_ && level_ <= 0;
        }
        if (depleted) empty_();
      }, opts);
    contact_ = node_->create_subscription<std_msgs::msg::Bool>("/battery/charging_connected", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        seen_contact_ = true; received_contact_ = std::chrono::steady_clock::now();
        connected_ = msg->data;
      }, opts);
    service_ = node_->create_service<std_srvs::srv::SetBool>("/agv/set_charging",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (req->data) {
          if (busy_()) { res->message = "不能开始充电：存在移动任务；请等待导航到充电点成功并停车后重试"; return; }
          std::lock_guard<std::mutex> lock(mutex_);
          if (!fresh()) { res->message = "不能开始充电：电池数据无效或过期，请检查 /battery_state"; return; }
        }
        res->success = control(req->data, res->message);
      });
    // 自愈定时器（专用回调组：healTick 内的 control() 会阻塞至多2秒，
    // 不得与 /agv/set_charging 服务或其它默认组回调互锁）
    heal_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    heal_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500),
                                           [this] { healTick(); }, heal_group_);
  }

  bool prepareMotion(std::string &error) {
    bool disconnect;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if ((required_ || seen_) && !fresh()) {
        error = "电池数据无效或超过有效期；请恢复 /battery_state 后重新下发任务"; return false;
      }
      if (fresh() && level_ <= 0) { error = "电量已耗尽；请先充电再下发移动任务"; return false; }
      if (required_ && fresh() && status_ == 0) { error = "电池充电状态 UNKNOWN；请检查电池驱动状态上报后重试"; return false; }
      disconnect = connected_ || status_ == 1 || status_ == 4;
      if (uncertain_) { error = "上次充电控制结果未确认；系统正按接触器/电池状态自动确认（约1秒），"
                                "若持续出现请检查充电后端；也可手动调用 /agv/set_charging(data=false)"; return false; }
    }
    if (disconnect && !control(false, error)) {
      error = "移动前停止充电失败，未发送导航目标：" + error; return false;
    }
    return true;
  }

  void fill(agv_bridge_v2_interfaces::msg::AgvStatus &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.battery_data_valid = fresh();
    if (seen_) msg.battery = level_;
    msg.charging = msg.battery_data_valid && status_ == 1;
    msg.charge_connected = connected_;
    msg.battery_state = "UNKNOWN";
    if (msg.battery_data_valid && !uncertain_) {
      switch(status_) {
        case 1: msg.battery_state = "CHARGING"; break;
        case 2: msg.battery_state = "DISCHARGING"; break;
        case 3: msg.battery_state = "NOT_CHARGING"; break;
        case 4: msg.battery_state = "FULL"; break;
      }
    }
  }
 private:
  bool fresh() const {
    return valid_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count() <= timeout_;
  }
  bool contactFresh() const {
    return seen_contact_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-received_contact_).count() <= timeout_;
  }
  // “用事实替代回执”：接触器真值新鲜且为未连接，电池状态新鲜且明确不在充电。
  // FULL 不算证据——满电插在充电桩上 status 同样可能是 FULL。
  bool evidenceDisconnected() const {
    return contactFresh() && !connected_ && fresh() &&
           (status_ == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING ||
            status_ == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
  }
  void armRetry() { stage_ = 0; next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(1); }
  void bumpRetry() {
    static const int secs[] = {1, 5, 15, 30};
    if (stage_ < 3) ++stage_;
    next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(secs[stage_]);
  }
  // 周期自愈：uncertain_ 时优先按证据解除；无论是否解除，都按退避计划补发一次
  // control(false) 同步后端（清除后端可能残留的充电标志）。证据不足（真插着/
  // 数据断流）时保持阻塞，仅由重试+事实在恢复的瞬间自动解开。
  void healTick() {
    bool fire = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!uncertain_) return;
      if (evidence_heal_ && evidenceDisconnected()) uncertain_ = false;
      if (std::chrono::steady_clock::now() >= next_retry_) { fire = true; bumpRetry(); }
    }
    if (!fire) return;
    std::string error;
    control(false, error);
  }
  bool control(bool enabled, std::string &error) {
    if (!client_->service_is_ready()) { error = "充电后端服务不可用，请检查 /battery/set_charging 或 charging_backend_service 配置"; return false; }
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = enabled;
    try {
      auto future = client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        client_->remove_pending_request(future);
        std::lock_guard<std::mutex> lock(mutex_); uncertain_ = true; armRetry();
        error = "充电后端2秒内未确认，当前连接状态未知；系统将按接触器状态自动确认并重试断开"; return false;
      }
      auto result = future.get(); error = result->message;
      if (!result->success) return false;
      std::lock_guard<std::mutex> lock(mutex_);
      connected_ = enabled; uncertain_ = false;
      return true;
    } catch(const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_); uncertain_ = true; armRetry();
      error = std::string("充电后端通信异常：") + e.what(); return false;
    }
  }
  rclcpp::Node *node_;
  std::function<bool()> busy_;
  std::function<void()> empty_;
  bool required_, seen_ = false, valid_ = false, connected_ = false, uncertain_ = false;
  bool evidence_heal_ = true, seen_contact_ = false;
  double timeout_, level_ = 100.0;
  uint8_t status_ = 0;
  std::chrono::steady_clock::time_point received_{}, received_contact_{}, next_retry_{};
  int stage_ = 0;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::CallbackGroup::SharedPtr heal_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr heal_timer_;
};
} // namespace agv_bridge
