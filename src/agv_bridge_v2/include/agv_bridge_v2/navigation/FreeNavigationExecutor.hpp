#pragma once
#include "agv_bridge_v2/navigation/NavigationExecutionContext.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"
#include <rclcpp_action/client_goal_handle.hpp>

namespace agv_bridge {
class FreeNavigationExecutor {
public:
    explicit FreeNavigationExecutor(NavigationExecutionContext &context) : context_(context) {}
    bool start(const MoveToMessage &task);
    void onAccepted();
    void onFeedback(const nav2_msgs::action::NavigateToPose::Feedback &feedback);
    void onResult(rclcpp_action::ResultCode code);
private:
    NavigationExecutionContext &context_;
};
} // namespace agv_bridge
