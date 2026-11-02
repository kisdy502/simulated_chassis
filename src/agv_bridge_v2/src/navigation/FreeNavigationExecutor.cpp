#include "agv_bridge_v2/navigation/FreeNavigationExecutor.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace agv_bridge {
bool FreeNavigationExecutor::start(const MoveToMessage &task) {
    if (!std::isfinite(task.x) || !std::isfinite(task.y) ||
        !std::isfinite(task.theta) || !std::isfinite(task.edgeInfo.maxSpeed) ||
        task.edgeInfo.maxSpeed <= 0.0 || task.edgeInfo.backUp ||
        task.edgeInfo.reverse || !task.edgeInfo.controlPoints.empty()) {
        context_.failTask("FAILED", "FREE 参数非法：目标和速度必须有效，且不能设置倒车、反向或控制点");
        return false;
    }
    if (!context_.navigateServerReady()) {
        context_.failTask("FAILED", "Nav2 navigate_to_pose 动作服务尚未就绪");
        return false;
    }
    context_.setSpeedLimit(task.edgeInfo.maxSpeed);
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = context_.executionNow();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = task.x;
    goal.pose.pose.position.y = task.y;
    goal.pose.pose.orientation = TransformUtils::yaw_to_quaternion(task.theta);
    if (!context_.sendNavigateGoal(goal)) return false;
    context_.setState(NavigationState::EXECUTING);
    RCLCPP_INFO(context_.executionLogger(),
        "Starting navigation to (%.2f, %.2f, %.2f) with command_id: %s",
        task.x, task.y, task.theta, task.commandId.c_str());
    return true;
}

void FreeNavigationExecutor::onAccepted() {
    context_.reportAccepted("准备执行自由导航到点了");
}

void FreeNavigationExecutor::onFeedback(const nav2_msgs::action::NavigateToPose::Feedback &feedback) {
    context_.reportFeedback(feedback.current_pose.pose.position.x,
        feedback.current_pose.pose.position.y,
        TransformUtils::quaternion_to_yaw(feedback.current_pose.pose.orientation));
}

void FreeNavigationExecutor::onResult(rclcpp_action::ResultCode code) {
    switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        context_.finishTask(true, "SUCCESS", "导航到点成功！"); break;
    case rclcpp_action::ResultCode::ABORTED:
        context_.failTask("ABORTED", "导航终止"); break;
    case rclcpp_action::ResultCode::CANCELED:
        context_.finishTask(false, "CANCELED", "导航取消"); break;
    default:
        context_.failTask("FAILED", "导航结果未知！"); break;
    }
}
} // namespace agv_bridge
