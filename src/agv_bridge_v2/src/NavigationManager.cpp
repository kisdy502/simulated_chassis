#include "agv_bridge_v2/NavigationManager.hpp"
#include "agv_bridge_v2/NavigationValidation.hpp"

#include "agv_bridge_v2/navigation/FreeNavigationExecutor.hpp"
#include "agv_bridge_v2/navigation/PathNavigationExecutor.hpp"

#include <cmath>
using namespace agv_bridge;
namespace agv_bridge
{

    NavigationManager::NavigationManager(rclcpp::Node *node, NavigationCallbacks *callbacks, std::shared_ptr<agv_bridge::LocalizationMonitor> pose_provider)
        : node_(node), 
          logger_(node->get_logger().get_child("NavigationManager")),
          callbacks_(callbacks), pose_provider_(pose_provider), current_state_(NavigationState::IDLE), initialized_(false)
    {
        speed_limit_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);

        if (!node_)
        {
            throw std::invalid_argument("NavigationManager: node cannot be null");
        }
        if (!callbacks_)
        {
            throw std::invalid_argument("NavigationManager: callbacks cannot be null");
        }
        action_session_ = std::make_shared<Nav2ActionSession>(node_);
        auto &context = static_cast<NavigationExecutionContext &>(*this);
        free_executor_ = std::make_unique<FreeNavigationExecutor>(context);
        path_executor_ = std::make_unique<PathNavigationExecutor>(context);
    }

    NavigationManager::~NavigationManager()
    {
        if (action_watchdog_) action_watchdog_->cancel();
        if (action_session_) action_session_->shutdown();
    }

    void NavigationManager::setSpeedLimit(double speed)
    {
        nav2_msgs::msg::SpeedLimit msg;
        msg.header.stamp = node_->now();
        msg.percentage = false;
        msg.speed_limit = speed;
        speed_limit_pub_->publish(msg);
        current_speed_limit_ = speed;

        RCLCPP_INFO(logger_, "Speed limit set to: %.2f m/s", speed);
    }

    bool NavigationManager::initialize()
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (initialized_)
        {
            return true;
        }

        // 倒车朝向偏差阈值：超过则不执行倒车（参见 backUpNavigationInternal）
        if (!node_->has_parameter("back_up_max_heading_error_deg"))
        {
            node_->declare_parameter<double>("back_up_max_heading_error_deg", 20.0);
        }
        node_->get_parameter("back_up_max_heading_error_deg", back_up_max_heading_error_deg_);

        for (auto entry : {std::make_pair("spin_timeout_sec", &spin_timeout_sec_),
                           std::make_pair("spin_progress_timeout_sec", &spin_progress_timeout_sec_),
                           std::make_pair("spin_progress_min_angle_rad", &spin_progress_min_angle_rad_),
                           std::make_pair("spin_cancel_timeout_sec", &spin_cancel_timeout_sec_)}) {
            if (!node_->has_parameter(entry.first)) node_->declare_parameter<double>(entry.first, *entry.second);
            node_->get_parameter(entry.first, *entry.second);
            if (!std::isfinite(*entry.second) || *entry.second <= 0.0)
                throw std::invalid_argument(std::string(entry.first) + " 必须是大于0的有限数值");
        }

        action_session_->waitForServers();

        for (const auto &name : {"controller_server", "smoother_server", "planner_server", "behavior_server",
                                 "bt_navigator", "waypoint_follower", "velocity_smoother"}) {
            LifecycleCheck check;
            check.name = name;
            check.client = node_->create_client<lifecycle_msgs::srv::GetState>(std::string(name) + "/get_state");
            lifecycle_checks_.push_back(std::move(check));
        }
        auto weak = weak_from_this();
        action_watchdog_ = node_->create_wall_timer(std::chrono::milliseconds(100),
            [weak]() { if (auto self = weak.lock()) self->checkActionProgress(); });
        initialized_ = true;
        RCLCPP_INFO(logger_, "Navigation manager initialized");
        return true;
    }

    /***********nav2 目标点点 自由导航过去 api START***************/

    bool NavigationManager::startNavigation(const std::string &command_id, const std::string &node_id, double x, double y, double theta)
    {
        MoveToMessage task;
        task.commandId = command_id;
        task.nodeId = node_id;
        task.x = x; task.y = y; task.theta = theta;
        task.edgeInfo.type = EdgeType::FREE;
        return startNavigation(task);
    }

    bool NavigationManager::startNavigation(const agv_bridge::MoveToMessage &task)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!isReady() || current_task_) return false;
        current_task_ = task;
        return free_executor_->start(task);
    }

    void NavigationManager::goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (!goal_handle)
        {
            RCLCPP_ERROR(logger_, "Goal was rejected by server");
            failTask("FAILED", "到点导航请求被拒绝!");
            return;
        }

        RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");

        // 检查任务有效性
        if (!current_task_)
        {
            RCLCPP_ERROR(logger_, "goalResponseCallback: current_task_ is null");
            return;
        }

        free_executor_->onAccepted();
    }

    void NavigationManager::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        free_executor_->onFeedback(*feedback);
    }

    void NavigationManager::resultCallback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (result.code == rclcpp_action::ResultCode::ABORTED) {
            failTask("ABORTED", "自由导航被 Nav2 终止" + action_session_->failureDetails());
            return;
        }
        free_executor_->onResult(result.code);
    }

    /***********nav2 目标点点 自由导航过去 api END***************/

    /***********nav2 旋转 api START***************/
    // 🔴 新增：发送 Spin 命令
    bool NavigationManager::sendSpinCommand(const double spin_angle)
    {
        auto goal_msg = nav2_msgs::action::Spin::Goal();
        goal_msg.target_yaw = spin_angle; // 正数逆时针，负数顺时针
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(spin_timeout_sec_);
        spin_target_angle_ = spin_angle;
        spin_watch_active_ = false;
        spin_failure_reason_.clear();
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigationManager::spinGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavigationManager::spinResultCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = [this](auto, const auto &feedback) {
            observeSpinProgress(feedback->angular_distance_traveled);
        };
        const auto error = readinessError();
        if (!error.empty()) { failTask("FAILED", "Nav2 未就绪: " + error); return false; }
        try {
            if (!action_session_->sendSpin(goal_msg, send_goal_options)) {
                failTask("FAILED", "底层动作会话仍被占用或尚未确认停止"); return false;
            }
        } catch (const std::exception &e) {
            failTask("FAILED", std::string("提交导航动作失败: ") + e.what()); return false;
        }
        return true;
    }

    // 🔴 新增：Spin 结果回调
    void NavigationManager::spinResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult &result)
    {
        spin_watch_active_ = false;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(logger_, "旋转完成");
            auto current_pose_opt = executionPose(); // 获取当前位姿（旋转后的）
            if (!current_pose_opt.has_value())
            {
                failTask("FAILED", "旋转结束但无法取得新鲜定位位姿，导航已停止");
                return;
            }

            // 检查任务是否仍然有效
            if (!current_task_)
            {
                RCLCPP_WARN(logger_, "spinResultCallback: current_task_ is null, cannot continue navigation");
                return;
            }

            const auto &current_pose = current_pose_opt.value().pose;

            // 缓存任务数据,避免多次访问 value()
            const auto &task = current_task_.value();

            const auto task_copy = task;
            path_executor_->onSpinSucceeded(current_state_, current_pose, task_copy);

            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转被终止");
            failTask("FAILED", reason + action_session_->failureDetails());
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转被取消");
            finishTask(false, "CANCELED", reason.c_str());
            break;
        }
        default:
            break;
        }
    }

    // 🔴 新增：Spin 响应回调
    void NavigationManager::spinGoalResponseCallback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>> goal_handle)
    {
        if (!goal_handle)
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转目标请求被拒绝");
            failTask("FAILED", reason.c_str());
        }
        else
        {
            spin_started_at_ = spin_last_progress_at_ = std::chrono::steady_clock::now();
            spin_progress_angle_ = spin_reported_angle_ = 0.0;
            spin_watch_active_ = true;
            RCLCPP_INFO(logger_, "旋转目标已接受");
        }
    }

    /***********nav2 旋转 api END***************/

    // 🔴 倒车导航内部实现(被followPathNavigation根据backUp标志调用)


    // 直线路点导航，public放，外部调用
    bool NavigationManager::followPathNavigation(const MoveToMessage task)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!isReady() || current_task_) return false;
        current_task_ = task;
        return path_executor_->startStraight(task);
    }

    // 贝塞尔曲线路点导航，public放，外部调用
    bool NavigationManager::followBezierPathNavigation(const MoveToMessage task)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!isReady() || current_task_) return false;
        current_task_ = task;
        return path_executor_->startCurve(task);
    }

    void NavigationManager::stopNavigation()
    {
        cancelNavigation();
    }

    bool NavigationManager::cancelNavigation()
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return requestCancel(5.0);
    }

    bool NavigationManager::requestCancel(double timeout_sec)
    {
        if (!current_task_) return false;
        if (action_session_->snapshot().canceling) return true;
        if (!action_session_->cancel(timeout_sec)) {
            // A send without an accepted handle is retired. The session keeps
            // unsafe until the late goal is rejected/terminated or servers stop.
            finishTask(false, "CANCELED", "导航已取消（底层目标尚未接受）");
        }
        return true;
    }

    bool NavigationManager::isReady() const
    {
        return readinessError().empty();
    }

    std::string NavigationManager::readinessError() const
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!initialized_) return "导航管理器尚未初始化";
        if (action_session_->snapshot().unsafe_generation != 0) return "上一条底层动作尚未确认停止";
        const auto now = std::chrono::steady_clock::now();
        if (lifecycle_checks_.empty()) return "Nav2 生命周期状态尚未取得";
        for (const auto &check : lifecycle_checks_) {
            if (!check.active) return check.name + " 未激活或状态查询失败";
            if (now - check.responded > std::chrono::seconds(3))
                return check.name + " 生命周期状态已过期";
        }
        if (!action_session_->spinReady() || !action_session_->pathReady())
            return "Nav2 动作服务尚未就绪";
        return {};
    }

    bool NavigationManager::requiresServerRestart() const
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return action_session_->snapshot().unsafe_generation != 0;
    }

    bool NavigationManager::isStackActive() const
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (!initialized_ || lifecycle_checks_.empty()) return false;
        for (const auto &check : lifecycle_checks_)
            if (!check.active || now - check.responded > std::chrono::seconds(3)) return false;
        return action_session_->spinReady() && action_session_->pathReady();
    }

    void NavigationManager::resetAfterServerShutdown()
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        // Only call after the owned Nav2 process group has exited.
        action_session_->resetAfterServerShutdown();
        for (auto &check : lifecycle_checks_) {
            ++check.request_generation;
            check.client->prune_pending_requests();
            check.pending = check.active = false;
            check.responded = {};
        }
        if (current_task_) {
            const std::string reason = spin_failure_reason_.empty()
                ? "Nav2 生命周期异常，导航已停止；恢复后请重新下发任务"
                : spin_failure_reason_ + "；取消未确认，现已确认托管 Nav2 进程组停止；请重新下发任务";
            failTask("FAILED", reason);
        }
    }

    void NavigationManager::observeSpinProgress(double angle)
    {
        if (!spin_watch_active_ || !std::isfinite(angle)) return;
        // Spin反馈为相对起始朝向的累计角度；只认距起点更远的新进展，
        // 重复反馈或来回抖动不能无限续期。
        spin_reported_angle_ = angle;
        const double directed = std::abs(angle);
        if (directed - spin_progress_angle_ >= spin_progress_min_angle_rad_) {
            spin_progress_angle_ = directed;
            spin_last_progress_at_ = std::chrono::steady_clock::now();
        }
    }

    void NavigationManager::stopFailedSpin(const std::string &reason)
    {
        spin_watch_active_ = false;
        spin_failure_reason_ = reason;
        RCLCPP_ERROR(logger_, "%s；请求取消旋转，等待底层终态", reason.c_str());
        requestCancel(spin_cancel_timeout_sec_);
    }

    void NavigationManager::checkActionProgress()
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        processActionEvents();
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < lifecycle_checks_.size(); ++i) {
            auto &check = lifecycle_checks_[i];
            if (check.pending && now - check.checked > std::chrono::seconds(3)) {
                ++check.request_generation;
                check.client->prune_pending_requests();
                check.pending = false;
                check.active = false;
            }
            if (check.pending || now - check.checked < std::chrono::seconds(1)) continue;
            check.checked = now;
            if (!check.client->service_is_ready()) { check.active = false; continue; }
            check.pending = true;
            const auto request_generation = ++check.request_generation;
            check.client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
                [weak = weak_from_this(), i, request_generation](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                    auto self = weak.lock();
                    if (!self) return;
                    std::lock_guard<std::recursive_mutex> lock(self->mutex_);
                    auto &entry = self->lifecycle_checks_[i];
                    if (entry.request_generation != request_generation) return;
                    entry.pending = false;
                    try { entry.active = future.get()->current_state.id == 3; }
                    catch (const std::exception &e) {
                        entry.active = false;
                        RCLCPP_WARN(self->logger_, "%s 状态查询失败: %s", entry.name.c_str(), e.what());
                    }
                    entry.responded = std::chrono::steady_clock::now();
                });
        }
        if (current_task_ && spin_watch_active_ && !action_session_->snapshot().canceling) {
            const double elapsed = std::chrono::duration<double>(now - spin_started_at_).count();
            const double stalled = std::chrono::duration<double>(now - spin_last_progress_at_).count();
            if (elapsed >= spin_timeout_sec_ || stalled >= spin_progress_timeout_sec_) {
                const std::string reason = elapsed >= spin_timeout_sec_
                    ? "旋转总时限超时（限制 " + std::to_string(spin_timeout_sec_) + " 秒）"
                    : "旋转无进展（连续 " + std::to_string(spin_progress_timeout_sec_) + " 秒未产生有效转动）";
                stopFailedSpin(reason + "；command_id=" + current_task_->commandId +
                    "，阶段=" + navigationStateToString(current_state_) +
                    "，目标角度=" + std::to_string(spin_target_angle_) +
                    " rad，已转角度=" + std::to_string(spin_reported_angle_) + " rad");
            }
        }

    }

    NavigationState NavigationManager::getCurrentState() const
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return current_state_;
    }

    bool NavigationManager::isNavigating() const
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return current_state_ != NavigationState::IDLE;
    }

    // 1. 执行路点导航和贝塞尔曲线路点导航，都到这三个回调
    void NavigationManager::followPathGoalResponseCallback(const GoalHandleFollowPath::SharedPtr &goal_handle)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (!goal_handle)
        {
            failTask("FAILED", "路点导航请求被拒绝!");
            return;
        }

        // 检查任务有效性
        if (!current_task_)
        {
            RCLCPP_ERROR(logger_, "followPathGoalResponseCallback: current_task_ is null");
            return;
        }

        path_executor_->onAccepted();
    }

    // 2. 反馈回调
    void NavigationManager::followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        path_executor_->onFeedback(*feedback);
    }

    // 3. 结果回调
    void NavigationManager::followPathResultCallback(const GoalHandleFollowPath::WrappedResult &result)
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!current_task_) return;
        const auto task = *current_task_;
        if (result.code == rclcpp_action::ResultCode::ABORTED) {
            failTask("FAILED", "路径跟随被 Nav2 终止" + action_session_->failureDetails());
            return;
        }
        path_executor_->onResult(result.code, task);
    }

    /***其他辅助的方法 */
    void NavigationManager::setState(NavigationState new_state)
    {
        if (current_state_ != new_state)
        {
            NavigationState old_state = current_state_;
            current_state_ = new_state;

            RCLCPP_INFO(logger_, "导航状态中%s 转换到 %s", navigationStateToString(old_state).c_str(), navigationStateToString(new_state).c_str());
            if (callbacks_)
            {
                // 🔴 如果 current_task_ 为空（如 stopNavigation 中已 reset），传递空字符串
                std::string command_id = current_task_.has_value() ? current_task_.value().commandId : "";
                callbacks_->onNavigationStateChanged(new_state, command_id);
            }
        }
    }

    void NavigationManager::finishTask(bool success, const std::string status, const std::string message)
    {
        // 缓存任务数据,避免多次访问 value() (防止在多线程环境下current_task_被重置)
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!current_task_) return;
        agv_bridge::MoveToMessage task = current_task_.value();
        action_session_->retire();
        spin_watch_active_ = false;
        spin_failure_reason_.clear();

        if (callbacks_)
        {
            // std::string status = success ? "SUCCESS" : "FAILED";
            callbacks_->onCommandAck(task.commandId, task.nodeId, status, message);
            callbacks_->onNavigationResult(success, message, task.commandId, task.nodeId);
        }

        if (success)
        {
            setState(status == "SUCCESS" ? NavigationState::COMPLETED : NavigationState::CANCELLED);
        }
        else
        {
            setState(NavigationState::FAILED);
        }
        current_task_.reset();
        setState(NavigationState::IDLE);
    }
    void NavigationManager::failTask(const std::string status, const std::string message)
    {
        finishTask(false, status, message);
    }

    const rclcpp::Logger &NavigationManager::executionLogger() const { return logger_; }
    rclcpp::Time NavigationManager::executionNow() const { return node_->now(); }
    std::optional<geometry_msgs::msg::PoseStamped> NavigationManager::executionPose() const
    {
        return pose_provider_ ? pose_provider_->getCurrentPose() : std::nullopt;
    }
    double NavigationManager::backUpHeadingLimit() const { return back_up_max_heading_error_deg_; }
    bool NavigationManager::navigateServerReady() const { return action_session_->navigateReady(); }
    void NavigationManager::reportAccepted(const std::string &message)
    {
        if (callbacks_ && current_task_) {
            const auto task = *current_task_;
            callbacks_->onCommandAck(task.commandId, task.nodeId, "ACCEPTED", message);
        }
    }
    void NavigationManager::reportFeedback(double x, double y, double yaw)
    {
        if (callbacks_) callbacks_->onNavigationFeedback(x, y, yaw);
    }
    bool NavigationManager::sendNavigateGoal(const NavigateToPose::Goal &goal)
    {
        NavigationActionClient::SendGoalOptions options;
        options.goal_response_callback = std::bind(&NavigationManager::goalResponseCallback, this, std::placeholders::_1);
        options.feedback_callback = std::bind(&NavigationManager::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        options.result_callback = std::bind(&NavigationManager::resultCallback, this, std::placeholders::_1);
        const auto error = readinessError();
        if (!error.empty()) { failTask("FAILED", "Nav2 未就绪: " + error); return false; }
        try {
            if (action_session_->sendNavigate(goal, options)) return true;
            failTask("FAILED", "底层动作会话仍被占用或尚未确认停止");
        } catch (const std::exception &e) { failTask("FAILED", std::string("提交导航动作失败: ") + e.what()); }
        return false;
    }
    bool NavigationManager::sendPathGoal(const FollowPath::Goal &goal)
    {
        rclcpp_action::Client<FollowPath>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&NavigationManager::followPathGoalResponseCallback, this, std::placeholders::_1);
        options.feedback_callback = std::bind(&NavigationManager::followPathFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        options.result_callback = std::bind(&NavigationManager::followPathResultCallback, this, std::placeholders::_1);
        const auto error = readinessError();
        if (!error.empty()) { failTask("FAILED", "Nav2 未就绪: " + error); return false; }
        try {
            if (action_session_->sendPath(goal, options)) return true;
            failTask("FAILED", "底层动作会话仍被占用或尚未确认停止");
        } catch (const std::exception &e) { failTask("FAILED", std::string("提交导航动作失败: ") + e.what()); }
        return false;
    }

    void NavigationManager::processActionEvents()
    {
        using Type = Nav2ActionSession::Type;
        for (auto &event : action_session_->poll()) {
            if (!action_session_->isCurrent(event.generation) || !current_task_) continue;
            if (event.type == Type::UNKNOWN_RESULT) {
                spin_watch_active_ = false;
                spin_failure_reason_ = "底层动作返回未知终态；command_id=" + current_task_->commandId +
                    "，阶段=" + navigationStateToString(current_state_) + "；底层尚未确认停止";
                RCLCPP_ERROR(logger_, "%s", spin_failure_reason_.c_str());
            } else if (event.type == Type::RESPONSE_TIMEOUT) {
                failTask("FAILED", "底层导航目标响应超时，请等待 Nav2 就绪后重试");
            } else if (event.type == Type::CANCEL_TIMEOUT) {
                if (!spin_failure_reason_.empty()) {
                    RCLCPP_ERROR(logger_, "%s；取消等待超时，等待托管 Nav2 停止确认", spin_failure_reason_.c_str());
                } else {
                    finishTask(false, "CANCELED", "导航取消结果等待超时，已释放任务");
                }
            } else if (event.type == Type::RESULT && action_session_->snapshot().canceling) {
                if (!spin_failure_reason_.empty()) finishTask(false, "FAILED", spin_failure_reason_);
                else finishTask(false, "CANCELED", "导航已取消");
            } else if (event.type != Type::FEEDBACK || !action_session_->snapshot().canceling) {
                if (event.deliver) event.deliver();
            }
        }
    }
} // namespace agv_bridge
