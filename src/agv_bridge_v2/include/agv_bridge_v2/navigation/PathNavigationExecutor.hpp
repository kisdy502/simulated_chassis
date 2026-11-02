#pragma once
#include "agv_bridge_v2/navigation/NavigationExecutionContext.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"
#include <rclcpp_action/client_goal_handle.hpp>

namespace agv_bridge {
// One shared path-following flow for straight, curve and heading-preserving
// backup. Task/stage ownership and all action lifecycle guards stay in Manager.
class PathNavigationExecutor {
public:
    explicit PathNavigationExecutor(NavigationExecutionContext &context) : context_(context) {}
    bool startStraight(const MoveToMessage &task);
    bool startCurve(const MoveToMessage &task);
    void onAccepted();
    void onFeedback(const nav2_msgs::action::FollowPath::Feedback &feedback);
    void onResult(rclcpp_action::ResultCode code, const MoveToMessage &task);
    void onSpinSucceeded(NavigationState state, const geometry_msgs::msg::Pose &pose,
                         const MoveToMessage &task);
private:
    bool startBackUp(const MoveToMessage &task);
    bool executeStraight(const geometry_msgs::msg::Pose &pose, const MoveToMessage &task);
    bool executeCurve(const geometry_msgs::msg::Pose &pose, const MoveToMessage &task);
    NavigationExecutionContext &context_;
};
} // namespace agv_bridge
