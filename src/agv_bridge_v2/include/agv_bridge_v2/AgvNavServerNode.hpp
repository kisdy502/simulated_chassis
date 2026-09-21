#ifndef AGV_NAV_SERVER_NODE_HPP
#define AGV_NAV_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "agv_bridge_v2_interfaces/action/follow_edge.hpp"
#include "agv_bridge_v2_interfaces/msg/agv_status.hpp"
#include "agv_bridge_v2_interfaces/srv/set_control.hpp"

#include "agv_bridge_v2/NavigationManager.hpp"
#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <atomic>

namespace agv_bridge
{

    /**
     * @brief AGV 导航服务节点（rosbridge 架构下的「瘦节点」）
     *
     * 职责边界（这是与旧 AgvBridgeNode 最大的区别）：
     *   ✅ 保留：贝塞尔/直线/倒车路径生成、预旋转、nav2 FollowPath/Spin 调度、限速下发
     *   ❌ 删除：WebSocketServer（libwebsockets）、HttpApiClient（libcurl）、
     *            bean/* JSON 序列化类、心跳与上报定时器
     *
     * 对外只暴露标准 ROS 2 接口，上位机通过 rosbridge_server 以 JSON 协议访问：
     *   action  /agv/follow_edge   agv_bridge_v2_interfaces/action/FollowEdge
     *   srv     /agv/set_control   agv_bridge_v2_interfaces/srv/SetControl
     *   topic   /agv/status        agv_bridge_v2_interfaces/msg/AgvStatus   (1Hz, transient_local)
     *   tf      map -> <agv_id>/base_link
     *
     * 上位机不需要再「注册 / 心跳 / 握手」，WS 连接本身即注册，连接断开即注销。
     */
    class AgvNavServerNode : public rclcpp::Node, public agv_bridge::NavigationCallbacks
    {
    public:
        using FollowEdge = agv_bridge_v2_interfaces::action::FollowEdge;
        using GoalHandleFollowEdge = rclcpp_action::ServerGoalHandle<FollowEdge>;
        using SetControl = agv_bridge_v2_interfaces::srv::SetControl;
        using AgvStatus = agv_bridge_v2_interfaces::msg::AgvStatus;

        explicit AgvNavServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~AgvNavServerNode() noexcept override;

        AgvNavServerNode(const AgvNavServerNode &) = delete;
        AgvNavServerNode &operator=(const AgvNavServerNode &) = delete;

    protected:
        // ===== NavigationCallbacks 实现：把 NavigationManager 的回调映射成 action 反馈/结果 =====
        void onNavigationStateChanged(agv_bridge::NavigationState state,
                                      const std::string &command_id) override;

        void onNavigationFeedback(double x, double y, double theta) override;

        void onNavigationResult(bool success,
                                const std::string &message,
                                const std::string &command_id,
                                const std::string &node_id) override;

        void onCommandAck(const std::string &command_id,
                          const std::string &node_id,
                          const std::string &status,
                          const std::string &message) override;

    private:
        // ===== 初始化 =====
        void initialize_parameters();
        void initialize_components();
        void create_interfaces();
        void create_timers();

        // ===== action 回调 =====
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FollowEdge::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFollowEdge> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle);

        // ===== service 回调 =====
        void handle_set_control(
            std::shared_ptr<SetControl::Request> request,
            std::shared_ptr<SetControl::Response> response);

        // ===== 定时器回调 =====
        void publish_status();
        void publish_feedback();
        void update_localization_monitor();
        void broadcast_tf();

        // ===== 工具 =====
        /// @brief 把 FollowEdge::Goal 翻译成 NavigationManager 需要的 MoveToMessage
        static MoveToMessage to_move_to_message(const FollowEdge::Goal &goal);

        /// @brief 以 ABORT 结束当前 goal（并清空句柄）
        void abort_current_goal(const std::string &message);

        /// @brief 清空当前 goal 句柄
        void reset_goal();

        // ===== 参数 =====
        std::string agv_id_ = "AGV001";
        double battery_level_ = 100.0;
        int feedback_interval_ms_ = 400;
        bool enable_tf_broadcast_ = true;

        // ===== 状态 =====
        std::atomic<bool> control_stopped_{false};
        std::atomic<bool> cancel_requested_{false};
        std::string agv_state_ = "IDLE";
        std::string active_command_id_;
        std::string active_node_id_;

        // ===== ROS 组件 =====
        rclcpp_action::Server<FollowEdge>::SharedPtr follow_edge_server_;
        rclcpp::Service<SetControl>::SharedPtr set_control_srv_;
        rclcpp::Publisher<AgvStatus>::SharedPtr status_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr feedback_timer_;
        rclcpp::TimerBase::SharedPtr localization_timer_;
        rclcpp::TimerBase::SharedPtr tf_timer_;

        // ===== 业务组件（全部复用，未做修改） =====
        std::shared_ptr<agv_bridge::LocalizationMonitor> localization_monitor_;
        std::shared_ptr<agv_bridge::NavigationManager> navigation_manager_;

        // ===== 同步 =====
        mutable std::mutex goal_mutex_;
        std::shared_ptr<GoalHandleFollowEdge> current_goal_handle_;
    };

} // namespace agv_bridge

#endif // AGV_NAV_SERVER_NODE_HPP
