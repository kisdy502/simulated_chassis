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
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include "agv_bridge_v2/bean/PathPoint.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <nav2_msgs/msg/speed_limit.hpp>
#include <angles/angles.h>
#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include "agv_bridge_v2/bean/Edge.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using Spin = nav2_msgs::action::Spin;
using GoalHandleFollowPath = rclcpp_action::Client<nav2_msgs::action::FollowPath>::GoalHandle;

namespace agv_bridge
{

    // 导航状态枚举,其中PRE_ROTATING =移动前，先旋转到目标角度，END_ROTATING，移动结束后，旋转到目标点
    enum class NavigationState
    {
        IDLE,
        PRE_ROTATING,
        EXECUTING,
        END_ROTATING,
        BACKING_UP,  // 🔴 新增:倒车状态
        COMPLETED,
        FAILED,
        CANCELLED
    };

    inline std::string navigationStateToString(NavigationState state)
    {
        switch (state)
        {
        case NavigationState::IDLE:
            return "IDLE";
        case NavigationState::PRE_ROTATING:
            return "PRE_ROTATING";
        case NavigationState::EXECUTING:
            return "EXECUTING";
        case NavigationState::END_ROTATING:
            return "END_ROTATING";
        case NavigationState::BACKING_UP:
            return "BACKING_UP";
        case NavigationState::COMPLETED:
            return "COMPLETED";
        case NavigationState::FAILED:
            return "FAILED";
        case NavigationState::CANCELLED:
            return "CANCELLED";
        default:
            return "UNKNOWN(" + std::to_string(static_cast<int>(state)) + ")";
        }
    }

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

    class NavigationManager : public std::enable_shared_from_this<NavigationManager>
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

        void setSpeedLimit(double speed);

    private:
        // 回调函数
        void goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle);
        void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                              const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void resultCallback(const GoalHandleNavigateToPose::WrappedResult &result);

        void followPathGoalResponseCallback(const GoalHandleFollowPath::SharedPtr &goal_handle);
        void followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
        void followPathResultCallback(const GoalHandleFollowPath::WrappedResult &result);

        // /*生成直线路径点函数
        //  */
        std::vector<geometry_msgs::msg::PoseStamped> generateStraightLinePath(
            const geometry_msgs::msg::Pose robot_pose, const agv_bridge::MoveToMessage &msg);

        std::vector<geometry_msgs::msg::PoseStamped> generateBezierPath(
            const geometry_msgs::msg::Pose, const agv_bridge::MoveToMessage &msg);

        // 🔴 新增:生成倒车直线路径
        std::vector<geometry_msgs::msg::PoseStamped> generateBackUpPath(
            const geometry_msgs::msg::Pose robot_pose, const agv_bridge::MoveToMessage &msg);

        // 🔴 新增：倒车导航内部实现
        bool backUpNavigationInternal(const agv_bridge::MoveToMessage moveToMessage);

        // 🔴 新增：执行实际导航
        bool executeFollowPath(
            const geometry_msgs::msg::Pose robot_pose,
            const agv_bridge::MoveToMessage &msg);

        bool executeBezierPath(
            const geometry_msgs::msg::Pose robot_pose,
            const agv_bridge::MoveToMessage moveToMessage);

        // 🔴 新增：发送旋转命令
        bool sendSpinCommand(const double spin_angle);
        // 🔴 新增：回调函数
        void spinGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Spin>> goal_handle);
        void spinResultCallback(const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult &result);

        // 更新状态
        void setState(NavigationState new_state);

        void finishTask(bool success, const std::string status, const std::string message);
        void failTask(const std::string status, const std::string message);

        // 贝塞尔曲线计算辅助方法
        static std::pair<double, double> calculateBezierPoint(
            double t,
            double p0x, double p0y,
            const std::vector<double> &cp_x, const std::vector<double> &cp_y,
            double p1x, double p1y);

        static double calculateBezierTangent(
            double t,
            double p0x, double p0y,
            const std::vector<double> &cp_x, const std::vector<double> &cp_y,
            double p1x, double p1y);

        static double calculateBezierLength(
            double p0x, double p0y,
            const std::vector<double> &cp_x, const std::vector<double> &cp_y,
            double p1x, double p1y,
            int segments = 100);

        // 成员变量
        rclcpp::Node *node_;
        rclcpp::Logger logger_;  // 子logger，日志tag显示类名
        NavigationCallbacks *callbacks_;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

        // 当前导航状态
        NavigationState current_state_;

        // 互斥锁
        mutable std::mutex mutex_;

        // 是否已初始化
        bool initialized_;

        rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

        GoalHandleFollowPath::SharedPtr follow_path_goal_handle_;
        GoalHandleNavigateToPose::SharedPtr nav_goal_handle_;

        rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

        double current_speed_limit_ = 1.5; // 当前生效的限速

        std::shared_ptr<agv_bridge::LocalizationMonitor> pose_provider_;

        rclcpp_action::Client<Spin>::SharedPtr spin_client_; // 🔴 新增
        rclcpp_action::ClientGoalHandle<Spin>::SharedPtr spin_goal_handle_; // 🔴 新增：保存Spin动作句柄用于取消

        // 🔴 新增：待执行的导航信息
        bool pending_navigation_ = false; // 是否需要执行直线导航
        bool pending_bezier_ = false;     // 是否执行曲线导航
        bool pending_rotate_ = false;     // 最终是否要旋转到站点角度
        
        std::optional<agv_bridge::MoveToMessage> current_task_;
    };

} // namespace agv_bridge

#endif // NAVIGATION_MANAGER_HPP