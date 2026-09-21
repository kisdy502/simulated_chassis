#include "agv_bridge_v2/AgvNavServerNode.hpp"

#include "agv_bridge_v2/utils/TransformUtils.hpp"

#include <chrono>
#include <cmath>
#include <utility>

using namespace std::chrono_literals;
using namespace agv_bridge;

namespace agv_bridge
{

    // ============================ 构造 / 析构 ============================

    AgvNavServerNode::AgvNavServerNode(const rclcpp::NodeOptions &options)
        : Node("agv_nav_server", options)
    {
        initialize_parameters();
        initialize_components();
        create_interfaces();
        create_timers();

        RCLCPP_INFO(this->get_logger(),
                    "AGV Nav Server 已启动 (agv_id=%s)。上位机请通过 rosbridge 访问 "
                    "/agv/follow_edge、/agv/set_control、/agv/status",
                    agv_id_.c_str());
    }

    AgvNavServerNode::~AgvNavServerNode() noexcept
    {
        try
        {
            // 退出时取消正在执行的导航，避免 nav2 控制器停留在最后一条路径
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "析构中取消导航失败: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "析构中发生未知异常");
        }
    }

    // ============================ 初始化 ============================

    void AgvNavServerNode::initialize_parameters()
    {
        this->declare_parameter<std::string>("agv_id", "AGV001");
        this->declare_parameter<double>("battery_level", 100.0);
        this->declare_parameter<int>("feedback_interval_ms", 400);
        this->declare_parameter<bool>("enable_tf_broadcast", true);

        this->get_parameter("agv_id", agv_id_);
        this->get_parameter("battery_level", battery_level_);
        this->get_parameter("feedback_interval_ms", feedback_interval_ms_);
        this->get_parameter("enable_tf_broadcast", enable_tf_broadcast_);

        if (feedback_interval_ms_ <= 0)
        {
            feedback_interval_ms_ = 400;
        }
    }

    void AgvNavServerNode::initialize_components()
    {
        LocalizationMonitor::Config loc_config;
        loc_config.particle_spread_threshold = 1.0;
        loc_config.required_stable_count = 5;
        loc_config.quality_threshold = 0.5;

        localization_monitor_ =
            std::make_shared<LocalizationMonitor>(static_cast<rclcpp::Node *>(this), loc_config);

        navigation_manager_ = std::make_shared<agv_bridge::NavigationManager>(
            static_cast<rclcpp::Node *>(this), this, localization_monitor_);

        if (!navigation_manager_->initialize())
        {
            RCLCPP_WARN(this->get_logger(),
                        "NavigationManager 初始化未完全成功（nav2 动作服务可能尚未起来）");
        }
    }

    void AgvNavServerNode::create_interfaces()
    {
        // 注意：NavigationManager 内部使用相对名创建 nav2 客户端（follow_path / spin /
        // navigate_to_pose），因此本节点必须运行在根命名空间下。为保证上位机侧名字稳定，
        // 这里的接口一律使用显式绝对名。
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // ---- action: /agv/follow_edge ----
        rclcpp::Node *self = static_cast<rclcpp::Node *>(this);
        follow_edge_server_ = rclcpp_action::create_server<FollowEdge>(
            self,
            "/agv/follow_edge",
            std::bind(&AgvNavServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AgvNavServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&AgvNavServerNode::handle_accepted, this, std::placeholders::_1));

        // ---- service: /agv/set_control ----
        set_control_srv_ = this->create_service<SetControl>(
            "/agv/set_control",
            std::bind(&AgvNavServerNode::handle_set_control, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- topic: /agv/status (transient_local，新客户端一接入即可拿到最后一帧) ----
        rclcpp::QoS status_qos(1);
        status_qos.transient_local();
        status_pub_ = this->create_publisher<AgvStatus>("/agv/status", status_qos);

        // ---- topic: /agv/pose (map 系 PoseStamped，10Hz，上位机直读位姿专用) ----
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/agv/pose", 10);

        // ---- 可选：真实电池数据，无发布者时保持 battery_level 参数值 ----
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
            {
                if (std::isfinite(msg->percentage) && msg->percentage >= 0.0)
                {
                    // 标准约定：0.0 ~ 1.0；部分驱动直接给 0~100
                    battery_level_ = msg->percentage <= 1.0 ? msg->percentage * 100.0
                                                             : msg->percentage;
                    if (battery_level_ > 100.0) battery_level_ = 100.0;
                }
            });
    }

    void AgvNavServerNode::create_timers()
    {
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&AgvNavServerNode::publish_status, this));

        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(feedback_interval_ms_),
            std::bind(&AgvNavServerNode::publish_feedback, this));

        localization_timer_ = this->create_wall_timer(
            1s, std::bind(&AgvNavServerNode::update_localization_monitor, this));

        tf_timer_ = this->create_wall_timer(
            100ms, std::bind(&AgvNavServerNode::broadcast_tf, this));
    }

    // ============================ action 回调 ============================

    rclcpp_action::GoalResponse AgvNavServerNode::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowEdge::Goal> goal)
    {
        (void)uuid;

        // 不在这里拒绝「定位未初始化 / 启动失败」，而是先 ACCEPT 再 ABORT。
        // 这样上位机拿到的是一条带 message 的 action_result，等价于旧的 command_ack FAILED。
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (current_goal_handle_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "已有导航任务在执行 (command_id=%s)，拒绝新的 move_to (command_id=%s)",
                            active_command_id_.c_str(), goal->command_id.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (control_stopped_.load())
        {
            RCLCPP_WARN(this->get_logger(),
                        "AGV 处于 stop 状态，拒绝导航任务 (command_id=%s)", goal->command_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
                    "接受导航任务: command_id=%s, node_id=%s, edge=%s(%s), 目标(%.2f, %.2f, %.3f), "
                    "max_speed=%.2f, back_up=%d, reverse=%d, control_points=%zu",
                    goal->command_id.c_str(), goal->node_id.c_str(),
                    goal->edge_id.c_str(), goal->edge_type.c_str(),
                    goal->x, goal->y, goal->theta,
                    goal->max_speed, static_cast<int>(goal->back_up),
                    static_cast<int>(goal->reverse), goal->control_points.size());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AgvNavServerNode::handle_cancel(
        const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
    {
        cancel_requested_ = true;

        if (navigation_manager_ && navigation_manager_->isNavigating())
        {
            // 让 NavigationManager 去取消 nav2 的 follow_path / spin / navigate_to_pose，
            // 结果会通过 onNavigationResult 回到当前 goal，此处不做终结。
            RCLCPP_INFO(this->get_logger(), "收到取消请求，正在取消底层 nav2 动作...");
            navigation_manager_->stopNavigation();
        }
        else
        {
            // 底层没有在执行的动作，直接终结，避免上位机一直等 cancel 响应
            RCLCPP_INFO(this->get_logger(), "收到取消请求，但当前没有正在执行的导航，直接结束");

            auto result = std::make_shared<FollowEdge::Result>();
            result->success = false;
            result->message = "导航已取消";
            result->command_id = active_command_id_;
            result->node_id = active_node_id_;

            if (goal_handle && goal_handle->is_active())
            {
                goal_handle->canceled(result);
            }
            reset_goal();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AgvNavServerNode::handle_accepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            current_goal_handle_ = goal_handle;
        }
        cancel_requested_ = false;

        auto goal = goal_handle->get_goal();
        active_command_id_ = goal->command_id;
        active_node_id_ = goal->node_id;

        // 定位未收敛时不允许下发导航（与原 handle_move_command 的检查一致）
        if (!localization_monitor_->isInitialized())
        {
            abort_current_goal("无法获取机器人位姿（定位未初始化），拒绝执行导航命令");
            return;
        }

        MoveToMessage move_msg = to_move_to_message(*goal);

        bool started = false;
        if (move_msg.edgeInfo.isBezierCurve())
        {
            RCLCPP_INFO(this->get_logger(), "贝塞尔曲线阶数: %d, 控制点数量: %zu",
                        move_msg.edgeInfo.getBezierOrder(),
                        move_msg.edgeInfo.controlPoints.size());
            started = navigation_manager_->followBezierPathNavigation(move_msg);
        }
        else
        {
            started = navigation_manager_->followPathNavigation(move_msg);
        }

        if (!started)
        {
            abort_current_goal("启动导航失败（路径生成失败或位姿不可用）");
        }
    }

    // ============================ service 回调 ============================

    void AgvNavServerNode::handle_set_control(
        std::shared_ptr<SetControl::Request> request,
        std::shared_ptr<SetControl::Response> response)
    {
        const std::string &action = request->action;

        if (action == "start")
        {
            control_stopped_ = false;
            response->success = true;
            response->message = "已恢复任务接收";
        }
        else if (action == "stop")
        {
            control_stopped_ = true;
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
            response->success = true;
            response->message = "已停止当前导航并暂停接收任务";
        }
        else if (action == "reset")
        {
            control_stopped_ = false;
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
            response->success = true;
            response->message = "已复位";
        }
        else
        {
            response->success = false;
            response->message = "未知 action: " + action + "（仅支持 start | stop | reset）";
        }

        response->state = control_stopped_.load() ? "STOPPED" : agv_state_;

        RCLCPP_INFO(this->get_logger(), "set_control(%s) -> success=%d, state=%s",
                    action.c_str(), static_cast<int>(response->success), response->state.c_str());
    }

    // ============================ 定时器回调 ============================

    void AgvNavServerNode::publish_status()
    {
        AgvStatus msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.agv_id = agv_id_;
        msg.state = control_stopped_.load() ? "STOPPED" : agv_state_;
        msg.battery = battery_level_;
        msg.pose_initialized = localization_monitor_ ? localization_monitor_->isInitialized() : false;

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            msg.active_command_id = active_command_id_;
            msg.active_node_id = active_node_id_;
        }

        status_pub_->publish(msg);
    }

    void AgvNavServerNode::publish_feedback()
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
        }

        if (!goal_handle || !goal_handle->is_active())
        {
            return;
        }

        auto pose_opt = localization_monitor_->getCurrentPose();
        if (!pose_opt.has_value())
        {
            return;
        }

        const auto &pose = pose_opt.value().pose;

        auto feedback = std::make_shared<FollowEdge::Feedback>();
        feedback->x = pose.position.x;
        feedback->y = pose.position.y;
        feedback->theta = TransformUtils::quaternion_to_yaw(pose.orientation);
        feedback->state = control_stopped_.load() ? "STOPPED" : agv_state_;

        goal_handle->publish_feedback(feedback);
    }

    void AgvNavServerNode::update_localization_monitor()
    {
        if (!localization_monitor_)
        {
            return;
        }

        localization_monitor_->update();

        static bool last_initialized_state = false;
        const bool current = localization_monitor_->isInitialized();

        if (current != last_initialized_state)
        {
            if (current)
            {
                RCLCPP_INFO(this->get_logger(), "状态变化：位置初始化完成 ✅");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "状态变化：位置未初始化 ❌");
            }
            last_initialized_state = current;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                             "%s",
                             current ? "✅ 定位已就绪，可接受导航任务" : "❌ 定位未收敛，等待 Cartographer/AMCL");
    }

    void AgvNavServerNode::broadcast_tf()
    {
        if (!enable_tf_broadcast_)
        {
            return;
        }

        auto pose_opt = localization_monitor_->getCurrentPose();
        if (!pose_opt.has_value())
        {
            return;
        }

        const auto &pose = pose_opt.value().pose;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = agv_id_ + "/base_link";
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        // 同一 10Hz 节拍同步发布专用位姿话题（与 TF 内容一致，上位机直读不受 /tf 混帧影响）
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = transform.header;
        pose_stamped.pose = pose;
        pose_pub_->publish(pose_stamped);
    }

    // ============================ NavigationCallbacks 实现 ============================

    void AgvNavServerNode::onNavigationStateChanged(agv_bridge::NavigationState state,
                                                    const std::string &command_id)
    {
        (void)command_id;
        agv_state_ = navigationStateToString(state);
        RCLCPP_INFO(this->get_logger(), "导航状态 -> %s", agv_state_.c_str());
    }

    void AgvNavServerNode::onNavigationFeedback(double x, double y, double theta)
    {
        // 反馈统一由 publish_feedback() 定时器按 feedback_interval_ms 发布，
        // 这里只保留调试日志，避免上游回调频率过高把 WS 打爆。
        RCLCPP_DEBUG(this->get_logger(), "导航反馈: (%.2f, %.2f, %.2f)", x, y, theta);
    }

    void AgvNavServerNode::onNavigationResult(bool success,
                                              const std::string &message,
                                              const std::string &command_id,
                                              const std::string &node_id)
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
            current_goal_handle_.reset();
        }

        RCLCPP_INFO(this->get_logger(), "导航结果: %s, command_id=%s, node_id=%s, message=%s",
                    success ? "SUCCESS" : "FAILED", command_id.c_str(), node_id.c_str(), message.c_str());

        if (!goal_handle || !goal_handle->is_active())
        {
            return;
        }

        auto result = std::make_shared<FollowEdge::Result>();
        result->success = success;
        result->message = message;
        result->command_id = command_id;
        result->node_id = node_id;

        if (goal_handle->is_canceling() || cancel_requested_)
        {
            goal_handle->canceled(result);
        }
        else if (success)
        {
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
        }

        // 任务结束后清空，允许接收下一条指令
        active_command_id_.clear();
        active_node_id_.clear();
        cancel_requested_ = false;
    }

    void AgvNavServerNode::onCommandAck(const std::string &command_id,
                                        const std::string &node_id,
                                        const std::string &status,
                                        const std::string &message)
    {
        // 终态（SUCCESS/FAILED/CANCELED/ABORTED）都会紧跟一次 onNavigationResult，
        // 由那里统一回填 action result，这里只做日志与状态跟踪。
        RCLCPP_INFO(this->get_logger(), "command_ack: command_id=%s, node_id=%s, status=%s, message=%s",
                    command_id.c_str(), node_id.c_str(), status.c_str(), message.c_str());
    }

    // ============================ 工具 ============================

    MoveToMessage AgvNavServerNode::to_move_to_message(const FollowEdge::Goal &goal)
    {
        MoveToMessage msg;
        msg.commandId = goal.command_id;
        msg.nodeId = goal.node_id;
        msg.x = goal.x;
        msg.y = goal.y;
        msg.theta = goal.theta;
        msg.endPoint = goal.end_point;

        msg.edgeInfo.id = goal.edge_id;
        msg.edgeInfo.sourceId = goal.source_id;
        msg.edgeInfo.targetId = goal.target_id;
        msg.edgeInfo.maxSpeed = goal.max_speed > 0.0 ? goal.max_speed : 1.0;
        msg.edgeInfo.step = goal.step > 0.0 ? goal.step : 0.1;
        msg.edgeInfo.backUp = goal.back_up;
        msg.edgeInfo.reverse = goal.reverse;

        for (const auto &cp : goal.control_points)
        {
            msg.edgeInfo.controlPoints.emplace_back(cp.x, cp.y);
        }

        // edge_type 允许上位机省略：有控制点即为曲线，否则视为直线
        if (goal.edge_type == "CURVE")
        {
            msg.edgeInfo.type = EdgeType::CURVE;
        }
        else if (goal.edge_type == "ELEVATION")
        {
            msg.edgeInfo.type = EdgeType::ELEVATION;
        }
        else if (goal.edge_type == "STRAIGHT")
        {
            msg.edgeInfo.type = EdgeType::STRAIGHT;
        }
        else
        {
            msg.edgeInfo.type = goal.control_points.empty() ? EdgeType::STRAIGHT : EdgeType::CURVE;
        }

        // 与原 handle_move_command 一致：目标节点是边的起点则反向行驶
        if (!msg.nodeId.empty())
        {
            if (!msg.edgeInfo.sourceId.empty() && msg.nodeId == msg.edgeInfo.sourceId)
            {
                msg.edgeInfo.reverse = true;
            }
            else if (!msg.edgeInfo.targetId.empty() && msg.nodeId == msg.edgeInfo.targetId)
            {
                msg.edgeInfo.reverse = false;
            }
        }

        return msg;
    }

    void AgvNavServerNode::abort_current_goal(const std::string &message)
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        std::string command_id;
        std::string node_id;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
            current_goal_handle_.reset();
            command_id = active_command_id_;
            node_id = active_node_id_;
            active_command_id_.clear();
            active_node_id_.clear();
        }

        RCLCPP_WARN(this->get_logger(), "导航任务终止: %s", message.c_str());

        if (goal_handle && goal_handle->is_active())
        {
            auto result = std::make_shared<FollowEdge::Result>();
            result->success = false;
            result->message = message;
            result->command_id = command_id;
            result->node_id = node_id;
            goal_handle->abort(result);
        }

        cancel_requested_ = false;
    }

    void AgvNavServerNode::reset_goal()
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_.reset();
        active_command_id_.clear();
        active_node_id_.clear();
    }

} // namespace agv_bridge
