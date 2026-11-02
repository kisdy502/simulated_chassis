#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace agv_bridge {
// Transport callbacks only enqueue events; they NEVER call Manager. Client API
// calls NEVER run with mutex_ held. These two rules remove the client/Manager
// lock inversion and also keep late callbacks independent of Manager lifetime.
class Nav2ActionSession : public std::enable_shared_from_this<Nav2ActionSession> {
public:
    using Navigate = nav2_msgs::action::NavigateToPose;
    using Path = nav2_msgs::action::FollowPath;
    using Spin = nav2_msgs::action::Spin;
    using Clock = std::chrono::steady_clock;
    enum class Kind { NAVIGATE, PATH, SPIN };
    enum class Type { ACCEPTED, REJECTED, FEEDBACK, RESULT, RESPONSE_TIMEOUT, CANCEL_TIMEOUT, UNKNOWN_RESULT };
    struct Event {
        std::uint64_t generation;
        Kind kind;
        Type type;
        std::function<void()> deliver;
    };
    struct Snapshot {
        std::uint64_t generation = 0, unsafe_generation = 0;
        bool pending = false, canceling = false, accepted = false, active = false;
        Kind kind = Kind::NAVIGATE;
    };

    explicit Nav2ActionSession(rclcpp::Node *node);
    ~Nav2ActionSession();
    bool sendNavigate(const Navigate::Goal &, rclcpp_action::Client<Navigate>::SendGoalOptions);
    bool sendPath(const Path::Goal &, rclcpp_action::Client<Path>::SendGoalOptions);
    bool sendSpin(const Spin::Goal &, rclcpp_action::Client<Spin>::SendGoalOptions);
    bool navigateReady() const;
    bool pathReady() const;
    bool spinReady() const;
    void waitForServers();
    // True means there is an accepted/pending terminal to wait for. Pending
    // sends with no accepted handle are retired by the caller, preserving unsafe.
    bool cancel(double timeout_sec = 5.0);
    void retire();
    void resetAfterServerShutdown();
    void shutdown();
    Snapshot snapshot() const;
    bool isCurrent(std::uint64_t generation) const;
    std::vector<Event> poll();
    // Correlated logs are best-effort evidence, not an action-provided error.
    std::string failureDetails() const;

private:
    friend struct NavigationManagerTestPeer;
    friend struct Nav2ActionSessionTestPeer;
    template<class Action>
    bool send(const typename rclcpp_action::Client<Action>::SharedPtr &client,
              const typename Action::Goal &goal,
              typename rclcpp_action::Client<Action>::SendGoalOptions options, Kind kind) {
        std::uint64_t generation;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_ || active_ || unsafe_generation_ != 0) return false;
            beginDiagnostics(kind);
            generation = ++generation_;
            kind_ = kind;
            active_ = pending_ = true;
            accepted_ = canceling_ = timeout_reported_ = false;
            deadline_ = Clock::now() + std::chrono::seconds(5);
            cancel_goal_ = {};
        }
        auto weak = weak_from_this();
        std::weak_ptr<rclcpp_action::Client<Action>> weak_client = client;
        auto response = options.goal_response_callback;
        auto feedback = options.feedback_callback;
        auto result = options.result_callback;
        options.goal_response_callback = [weak, weak_client, generation, kind, response](auto handle) {
            auto self = weak.lock(); if (!self) return;
            std::lock_guard<std::mutex> lock(self->mutex_);
            if (self->closed_) return;
            if (!handle && generation == self->unsafe_generation_) self->unsafe_generation_ = 0;
            auto client = weak_client.lock();
            auto cancel = [client, handle]() { if (client && handle) client->async_cancel_goal(handle); };
            if (generation != self->generation_ || !self->active_) {
                if (handle) self->queueCancelLocked(generation, cancel);
                else if (generation == self->unsafe_generation_) self->unsafe_generation_ = 0;
                return;
            }
            self->pending_ = false;
            self->accepted_ = bool(handle);
            self->active_ = bool(handle);
            if (handle) {
                self->cancel_goal_ = cancel;
                if (self->canceling_) self->queueCancelLocked(generation, cancel);
            }
            self->events_.push_back({generation, kind, handle ? Type::ACCEPTED : Type::REJECTED,
                [response, handle]() { if (response) response(handle); }});
        };
        options.feedback_callback = [weak, generation, kind, feedback](auto handle, const auto &value) {
            auto self = weak.lock(); if (!self) return;
            std::lock_guard<std::mutex> lock(self->mutex_);
            if (self->closed_ || generation != self->generation_ || !self->active_ || self->canceling_) return;
            Event event{generation, kind, Type::FEEDBACK,
                [feedback, handle, value]() { if (feedback) feedback(handle, value); }};
            // Keep latest feedback without losing acceptance/result ordering.
            if (!self->events_.empty() && self->events_.back().generation == generation &&
                self->events_.back().type == Type::FEEDBACK) self->events_.back() = std::move(event);
            else self->events_.push_back(std::move(event));
        };
        options.result_callback = [weak, generation, kind, result](const auto &value) {
            auto self = weak.lock(); if (!self) return;
            std::lock_guard<std::mutex> lock(self->mutex_);
            if (self->closed_) return;
            const bool known = value.code == rclcpp_action::ResultCode::SUCCEEDED ||
                value.code == rclcpp_action::ResultCode::ABORTED || value.code == rclcpp_action::ResultCode::CANCELED;
            if (known && generation == self->unsafe_generation_) self->unsafe_generation_ = 0;
            if (generation != self->generation_ || !self->active_) return;
            self->pending_ = false;
            if (!known) {
                self->unsafe_generation_ = generation;
                self->canceling_ = true;
                self->deadline_ = Clock::now();
                self->events_.push_back({generation, kind, Type::UNKNOWN_RESULT, {}});
                return;
            }
            self->active_ = false;
            self->cancel_goal_ = {};
            self->events_.push_back({generation, kind, Type::RESULT,
                [result, value]() { if (result) result(value); }});
        };
        try {
            // No session lock here; callbacks can acquire mutex_ while the
            // ROS client is holding its own internal callback lock.
            client->async_send_goal(goal, options);
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            unsafe_generation_ = generation;
            throw;
        }
        return true;
    }
    struct DiagnosticEntry { Clock::time_point received; std::string source, text; };
    struct Diagnostics {
        std::mutex mutex;
        bool enabled = false;
        Kind kind = Kind::NAVIGATE;
        std::int64_t started_ros_ns = 0;
        std::deque<DiagnosticEntry> entries;
    };
    void beginDiagnostics(Kind kind);
    static void recordDiagnostic(const std::shared_ptr<Diagnostics> &state,
                                 const rcl_interfaces::msg::Log &message);
    std::shared_ptr<Diagnostics> diagnostics_ = std::make_shared<Diagnostics>();
    rclcpp::Clock::SharedPtr ros_clock_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_;
    struct DeferredCancel { std::uint64_t generation; std::function<void()> send; };
    void queueCancelLocked(std::uint64_t generation, std::function<void()> command);
    void runCancels(std::vector<DeferredCancel> commands);
    mutable std::mutex mutex_;
    bool closed_ = false, active_ = false, pending_ = false, accepted_ = false;
    bool canceling_ = false, timeout_reported_ = false;
    std::uint64_t generation_ = 0, unsafe_generation_ = 0;
    Kind kind_ = Kind::NAVIGATE;
    Clock::time_point deadline_;
    std::function<void()> cancel_goal_;
    std::deque<Event> events_;
    std::vector<DeferredCancel> deferred_cancels_;
    rclcpp::CallbackGroup::SharedPtr transport_group_;
    rclcpp_action::Client<Navigate>::SharedPtr navigate_;
    rclcpp_action::Client<Path>::SharedPtr path_;
    rclcpp_action::Client<Spin>::SharedPtr spin_;
};
} // namespace agv_bridge
