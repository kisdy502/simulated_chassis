#ifndef NAVIGATION_MANAGER_HPP
#define NAVIGATION_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>
#include <memory>
#include <functional>
#include <mutex>
#include <chrono>
#include <cstdint>
#include "lifecycle_msgs/srv/get_state.hpp"
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <nav2_msgs/msg/speed_limit.hpp>
#include <angles/angles.h>
#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include "agv_bridge_v2/navigation/NavigationExecutionContext.hpp"
#include "agv_bridge_v2/navigation/Nav2ActionSession.hpp"
#include "agv_bridge_v2/bean/Edge.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using Spin = nav2_msgs::action::Spin;
using GoalHandleFollowPath = rclcpp_action::Client<nav2_msgs::action::FollowPath>::GoalHandle;

namespace agv_bridge
{

    class FreeNavigationExecutor;
    class PathNavigationExecutor;

    // 导航回调接口
    class NavigationCallbacks
    {
    public:
        virtual ~NavigationCallbacks() = default;

        // 导航状态变化回调
        virtual void onNavigationStateChanged(NavigationState state, const std::string &command_id) = 0;

        // 导航反馈回调
        virtual void onNavigationFeedback(double x, double y, double theta) = 0;

        // 导航结果回调
        virtual void onNavigationResult(bool success, const std::string &message,
                                        const std::string &command_id, const std::string &node_id) = 0;

        // 命令确认回调
        virtual void onCommandAck(const std::string &command_id, const std::string &node_id,
                                  const std::string &status, const std::string &message) = 0;
    };

    class NavigationManager : public std::enable_shared_from_this<NavigationManager>,
                              private NavigationExecutionContext
    {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        using NavigationActionClient = rclcpp_action::Client<NavigateToPose>;

        /**
         * @brief 构造函数
         * @param node ROS2节点指针
         * @param callbacks 导航回调接口
         * @param pose_provider 机器人当前位置提供类
         */
        NavigationManager(rclcpp::Node *node, NavigationCallbacks *callbacks, std::shared_ptr<agv_bridge::LocalizationMonitor> pose_provider);

        /**
         * @brief 析构函数
         */
        ~NavigationManager();

        /**
         * @brief 初始化导航客户端
         * @return 是否初始化成功
         */
        bool initialize();

        /**
         * @brief 开始导航到目标点
         * @param command_id 命令ID
         * @param node_id 节点ID
         * @param x X坐标
         * @param y Y坐标
         * @param theta 朝向角（弧度）
         * @return 是否成功发送导航目标
         */
        bool startNavigation(const std::string &command_id, const std::string &node_id, double x, double y, double theta);
        bool startNavigation(const agv_bridge::MoveToMessage &task);

        /**
         * @param msg MoveToMessage
         * @return 是否成功发送导航目标
         */
        bool followPathNavigation(const agv_bridge::MoveToMessage msg);

        bool followBezierPathNavigation(const agv_bridge::MoveToMessage msg);

        /**
         * @brief 取消当前导航
         * @return 是否成功发送取消请求
         */
        bool cancelNavigation();

        /**
         * @brief 停止导航并重置状态（用于stop_move命令）
         */
        void stopNavigation();

        /**
         * @brief 获取当前导航状态
         */
        NavigationState getCurrentState() const;

        /**
         * @brief 获取当前命令ID
         */
        // std::string getCurrentCommandId() const;

        /**
         * @brief 获取当前节点ID
         */
        // std::string getCurrentNodeId() const;

        /**
         * @brief 是否正在导航
         */
        bool isNavigating() const;

        void setSpeedLimit(double speed) override;
        bool isReady() const;
        bool isStackActive() const;
        bool requiresServerRestart() const;
        std::string readinessError() const;
        void resetAfterServerShutdown();

    private:
        // Executors call only this boundary; the facade retains all ownership.
        const rclcpp::Logger &executionLogger() const override;
        rclcpp::Time executionNow() const override;
        std::optional<geometry_msgs::msg::PoseStamped> executionPose() const override;
        double backUpHeadingLimit() const override;
        bool navigateServerReady() const override;
        bool sendNavigateGoal(const NavigateToPose::Goal &goal) override;
        bool sendPathGoal(const FollowPath::Goal &goal) override;
        void reportAccepted(const std::string &message) override;
        void reportFeedback(double x, double y, double yaw) override;
        std::unique_ptr<FreeNavigationExecutor> free_executor_;
        std::unique_ptr<PathNavigationExecutor> path_executor_;

        // 回调函数
        void goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle);
        void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                              const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void resultCallback(const GoalHandleNavigateToPose::WrappedResult &result);

        void followPathGoalResponseCallback(const GoalHandleFollowPath::SharedPtr &goal_handle);
        void followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
        void followPathResultCallback(const GoalHandleFollowPath::WrappedResult &result);

        // 🔴 新增：发送旋转命令
        bool sendSpinCommand(const double spin_angle) override;
        // 🔴 新增：回调函数
        void spinGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Spin>> goal_handle);
        void spinResultCallback(const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult &result);

        // 更新状态
        void setState(NavigationState new_state) override;

        void finishTask(bool success, const std::string status, const std::string message) override;
        void failTask(const std::string status, const std::string message) override;

        friend struct NavigationManagerTestPeer;
        void checkActionProgress();
        void processActionEvents();
        bool requestCancel(double timeout_sec);
        std::shared_ptr<Nav2ActionSession> action_session_;
        void observeSpinProgress(double angle);
        void stopFailedSpin(const std::string &reason);
        bool spin_watch_active_ = false;
        double spin_timeout_sec_ = 60.0;
        double spin_progress_timeout_sec_ = 10.0;
        double spin_progress_min_angle_rad_ = 0.01;
        double spin_cancel_timeout_sec_ = 5.0;
        double spin_target_angle_ = 0.0;
        double spin_progress_angle_ = 0.0;
        double spin_reported_angle_ = 0.0;
        std::chrono::steady_clock::time_point spin_started_at_, spin_last_progress_at_;
        std::string spin_failure_reason_;

        rclcpp::TimerBase::SharedPtr action_watchdog_;
        struct LifecycleCheck {
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
            std::string name;
            bool pending = false;
            bool active = false;
            std::chrono::steady_clock::time_point checked;
            std::chrono::steady_clock::time_point responded;
            std::uint64_t request_generation = 0;
        };
        std::vector<LifecycleCheck> lifecycle_checks_;

        // 成员变量
        rclcpp::Node *node_;
        rclcpp::Logger logger_;  // 子logger，日志tag显示类名
        NavigationCallbacks *callbacks_;


        // 当前导航状态
        NavigationState current_state_;

        // 互斥锁
        mutable std::recursive_mutex mutex_;

        // 是否已初始化
        bool initialized_;



        rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

        double current_speed_limit_ = 1.5; // 当前生效的限速

        // 倒车朝向偏差阈值(度)。超过则不执行倒车指令 —— 倒车绝不自动旋转车体，
        // 避免在充电桩 / 窄过道 / 贴边作业现场因原地转向造成剐蹭。
        double back_up_max_heading_error_deg_ = 20.0;

        std::shared_ptr<agv_bridge::LocalizationMonitor> pose_provider_;

        std::optional<agv_bridge::MoveToMessage> current_task_;
    };

} // namespace agv_bridge

#endif // NAVIGATION_MANAGER_HPP