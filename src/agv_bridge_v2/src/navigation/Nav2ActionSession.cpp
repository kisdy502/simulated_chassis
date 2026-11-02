#include "agv_bridge_v2/navigation/Nav2ActionSession.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace agv_bridge {
Nav2ActionSession::Nav2ActionSession(rclcpp::Node *node) {
    ros_clock_ = node->get_clock();
    transport_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = transport_group_;
    // Capture independent state: an in-flight callback cannot access a destroyed session.
    rosout_ = node->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(100).best_effort().durability_volatile(),
        [state = diagnostics_](rcl_interfaces::msg::Log::ConstSharedPtr message) {
            recordDiagnostic(state, *message);
        }, subscription_options);
    navigate_ = rclcpp_action::create_client<Navigate>(node, "navigate_to_pose", transport_group_);
    path_ = rclcpp_action::create_client<Path>(node, "follow_path", transport_group_);
    spin_ = rclcpp_action::create_client<Spin>(node, "spin", transport_group_);
}
void Nav2ActionSession::beginDiagnostics(Kind kind) {
    std::lock_guard<std::mutex> lock(diagnostics_->mutex);
    diagnostics_->enabled = true;
    diagnostics_->kind = kind;
    diagnostics_->started_ros_ns = ros_clock_->now().nanoseconds();
    diagnostics_->entries.clear();
}
void Nav2ActionSession::recordDiagnostic(const std::shared_ptr<Diagnostics> &state,
                                        const rcl_interfaces::msg::Log &message) {
    if (message.level < rcl_interfaces::msg::Log::ERROR) return;
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->enabled || message.msg.empty()) return;
    const auto stamp = std::int64_t(message.stamp.sec) * 1000000000LL + message.stamp.nanosec;
    if (stamp < state->started_ros_ns) return; // Discard queued logs from a previous action.
    const auto separator = message.name.find_last_of("./");
    const auto source = message.name.substr(separator == std::string::npos ? 0 : separator + 1);
    const bool allowed = state->kind == Kind::PATH ? source == "controller_server" :
        state->kind == Kind::SPIN ? source == "behavior_server" :
        (source == "controller_server" || source == "planner_server" ||
         source == "bt_navigator" || source == "behavior_server");
    if (!allowed) return;
    auto end = std::min<std::size_t>(message.msg.size(), 900);
    while (end < message.msg.size() && end > 0 &&
           (static_cast<unsigned char>(message.msg[end]) & 0xc0) == 0x80) --end;
    auto text = message.msg.substr(0, end);
    for (auto &c : text) if (c == '\n' || c == '\r' || c == '\t') c = ' ';
    if (!state->entries.empty() && state->entries.back().source == source &&
        state->entries.back().text == text) return;
    state->entries.push_back({Clock::now(), source, std::move(text)});
    while (state->entries.size() > 3) state->entries.pop_front();
}
std::string Nav2ActionSession::failureDetails() const {
    std::lock_guard<std::mutex> lock(diagnostics_->mutex);
    std::string details;
    for (const auto &entry : diagnostics_->entries) {
        if (Clock::now() - entry.received > std::chrono::seconds(5)) continue;
        if (!details.empty()) details += " | ";
        details += "[" + entry.source + "] " + entry.text;
    }
    return details.empty() ? "；Nav2 Humble 未提供错误文本，未捕获本次动作的近期错误日志，请检查 Nav2 日志" :
        "；Nav2 关联错误日志（非动作结果字段）：" + details;
}
Nav2ActionSession::~Nav2ActionSession() { shutdown(); }
void Nav2ActionSession::waitForServers() {
    spin_->wait_for_action_server(std::chrono::seconds(10));
    navigate_->wait_for_action_server(std::chrono::seconds(5));
    path_->wait_for_action_server(std::chrono::seconds(5));
}
bool Nav2ActionSession::navigateReady() const { return navigate_->action_server_is_ready(); }
bool Nav2ActionSession::pathReady() const { return path_->action_server_is_ready(); }
bool Nav2ActionSession::spinReady() const { return spin_->action_server_is_ready(); }
bool Nav2ActionSession::sendNavigate(const Navigate::Goal &goal, rclcpp_action::Client<Navigate>::SendGoalOptions options) {
    return send<Navigate>(navigate_, goal, std::move(options), Kind::NAVIGATE);
}
bool Nav2ActionSession::sendPath(const Path::Goal &goal, rclcpp_action::Client<Path>::SendGoalOptions options) {
    return send<Path>(path_, goal, std::move(options), Kind::PATH);
}
bool Nav2ActionSession::sendSpin(const Spin::Goal &goal, rclcpp_action::Client<Spin>::SendGoalOptions options) {
    return send<Spin>(spin_, goal, std::move(options), Kind::SPIN);
}

Nav2ActionSession::Snapshot Nav2ActionSession::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {generation_, unsafe_generation_, pending_, canceling_, accepted_, active_, kind_};
}
bool Nav2ActionSession::isCurrent(std::uint64_t generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !closed_ && generation == generation_;
}

void Nav2ActionSession::runCancels(std::vector<DeferredCancel> commands) {
    for (auto &command : commands) {
        try { command.send(); }
        catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            // An exception is not evidence of stopping. Never poison a newer
            // action with an older cancel command after reset/terminal proof.
            if ((command.generation == generation_ && active_) || command.generation == unsafe_generation_)
                unsafe_generation_ = command.generation;
        }
    }
}

void Nav2ActionSession::queueCancelLocked(std::uint64_t generation, std::function<void()> command) {
    for (const auto &pending : deferred_cancels_) if (pending.generation == generation) return;
    deferred_cancels_.push_back({generation, std::move(command)});
}

bool Nav2ActionSession::cancel(double timeout_sec) {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0) throw std::invalid_argument("cancel timeout must be positive");
    std::vector<DeferredCancel> commands;
    bool wait;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wait = accepted_;
        if (closed_ || canceling_) return wait;
        canceling_ = true;
        timeout_reported_ = false;
        deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_sec));
        if (active_ && cancel_goal_) commands.push_back({generation_, cancel_goal_});
    }
    runCancels(std::move(commands));
    return wait;
}

void Nav2ActionSession::retire() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Acceptance may race a timeout event between poll() and Manager cleanup.
    // Do not drop a handle accepted in that window without targeting it for cancel.
    if (active_ && cancel_goal_) queueCancelLocked(generation_, cancel_goal_);
    if (active_ || pending_) unsafe_generation_ = generation_;
    { std::lock_guard<std::mutex> diagnostic_lock(diagnostics_->mutex); diagnostics_->enabled = false; }
    ++generation_;
    active_ = pending_ = accepted_ = canceling_ = timeout_reported_ = false;
    cancel_goal_ = {};
    events_.clear();
    // Keep deferred cancels for retired goals until poll sends them.
}

void Nav2ActionSession::resetAfterServerShutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    unsafe_generation_ = 0;
    { std::lock_guard<std::mutex> diagnostic_lock(diagnostics_->mutex); diagnostics_->enabled = false; diagnostics_->entries.clear(); }
    active_ = pending_ = accepted_ = canceling_ = timeout_reported_ = false;
    cancel_goal_ = {};
    events_.clear();
    deferred_cancels_.clear();
}

std::vector<Nav2ActionSession::Event> Nav2ActionSession::poll() {
    std::vector<DeferredCancel> cancels;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cancels.swap(deferred_cancels_);
    }
    runCancels(std::move(cancels));
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_ && active_ && (pending_ || canceling_) && !timeout_reported_ && Clock::now() >= deadline_) {
        unsafe_generation_ = generation_;
        timeout_reported_ = true;
        events_.push_back({generation_, kind_, canceling_ ? Type::CANCEL_TIMEOUT : Type::RESPONSE_TIMEOUT, {}});
    }
    std::vector<Event> events;
    while (!events_.empty()) { events.push_back(std::move(events_.front())); events_.pop_front(); }
    return events;
}

void Nav2ActionSession::shutdown() {
    std::vector<DeferredCancel> cancels;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return;
        closed_ = true;
        if (active_ && cancel_goal_) cancels.push_back({generation_, cancel_goal_});
        for (auto &cancel : deferred_cancels_) cancels.push_back(std::move(cancel));
        deferred_cancels_.clear(); events_.clear(); cancel_goal_ = {};
    }
    runCancels(std::move(cancels));
}
} // namespace agv_bridge
