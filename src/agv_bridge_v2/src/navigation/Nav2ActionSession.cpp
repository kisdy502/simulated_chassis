#include "agv_bridge_v2/navigation/Nav2ActionSession.hpp"
#include <cmath>
#include <stdexcept>

namespace agv_bridge {
Nav2ActionSession::Nav2ActionSession(rclcpp::Node *node) {
    transport_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    navigate_ = rclcpp_action::create_client<Navigate>(node, "navigate_to_pose", transport_group_);
    path_ = rclcpp_action::create_client<Path>(node, "follow_path", transport_group_);
    spin_ = rclcpp_action::create_client<Spin>(node, "spin", transport_group_);
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
