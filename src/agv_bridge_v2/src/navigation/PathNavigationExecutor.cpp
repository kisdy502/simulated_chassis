#include "agv_bridge_v2/navigation/PathNavigationExecutor.hpp"
#include "agv_bridge_v2/navigation/StraightPathBuilder.hpp"
#include "agv_bridge_v2/navigation/BezierPathBuilder.hpp"
#include "agv_bridge_v2/navigation/BackUpPathBuilder.hpp"
#include "agv_bridge_v2/NavigationValidation.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <rclcpp/logging.hpp>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
namespace agv_bridge {
using FollowPath = nav2_msgs::action::FollowPath;
bool PathNavigationExecutor::startBackUp(const MoveToMessage &moveToMessage)
{
    // 当前任务只由 Manager 持有，执行器使用本次传入的任务快照。

    auto current_pose_opt = context_.executionPose();
    if (!current_pose_opt.has_value())
    {
        RCLCPP_ERROR(context_.executionLogger(), "机器人当前位置信息为空");
        context_.failTask("FAILED", "无法获取机器人位姿（定位不可用），未执行沿路径导航");
        return false;
    }
    const auto &robot_pose = current_pose_opt.value().pose;


    // ===== 倒车逻辑：只后退，绝不旋转车体 =====
    // 充电桩对接、窄过道通行、贴边作业等场景下，原地旋转会剐蹭甚至碰撞，
    // 因此这里不再调用 context_.sendSpinCommand()。若朝向偏差导致直线后退在几何上不可行，
    // 直接以 FAILED 结束，把「先摆正车体」的决定权交回调度系统 / 现场操作员。
    const double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
    const double target_yaw = std::atan2(moveToMessage.y - robot_pose.position.y,
                                        moveToMessage.x - robot_pose.position.x);

    // 后退时车头应朝向「目标方向 + 180°」，即车尾对着目标点
    const double desired_backup_yaw = angles::normalize_angle(target_yaw + M_PI);
    const double heading_error_deg =
        std::abs(angles::shortest_angular_distance(robot_yaw, desired_backup_yaw)) * 180.0 / M_PI;

    RCLCPP_INFO(context_.executionLogger(),
                "倒车导航(不旋转): 目标点(%.2f, %.2f), 当前朝向:%.2f°, 车尾应对准:%.2f°, 偏差:%.2f°",
                moveToMessage.x, moveToMessage.y,
                robot_yaw * 180 / M_PI, desired_backup_yaw * 180 / M_PI, heading_error_deg);

    if (heading_error_deg > context_.backUpHeadingLimit())
    {
        RCLCPP_ERROR(context_.executionLogger(),
                     "倒车朝向偏差 %.2f° 超过阈值 %.2f°：直线后退无法贴合路径，"
                     "已拒绝执行（不会自动旋转车体）。请先摆正车体，或改用前进导航指令。",
                     heading_error_deg, context_.backUpHeadingLimit());
        context_.failTask("FAILED", "倒车朝向偏差过大，未执行（不自动旋转车体）");
        return true; // 结果已通过 failTask 回传，非「启动失败」
    }

    // 朝向可用，执行直线后退
    auto path = BackUpPathBuilder::build(robot_pose, moveToMessage, context_.executionLogger());
    if (path.empty())
    {
        RCLCPP_ERROR(context_.executionLogger(), "Failed to generate backup path");
        return false;
    }

    // 设置倒车速度限制(负值表示倒车)
    context_.setSpeedLimit(-moveToMessage.edgeInfo.maxSpeed);

    // 创建 FollowPath 目标
    auto goal_msg = FollowPath::Goal();
    goal_msg.path.poses = path;
    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = context_.executionNow();
    goal_msg.controller_id = "FollowPath";

    if (!context_.sendPathGoal(goal_msg)) return false;
    context_.setState(NavigationState::BACKING_UP); // 使用倒车状态

    RCLCPP_INFO(context_.executionLogger(),
                "开始倒车(保持当前朝向): (%.2f, %.2f) -> (%.2f, %.2f), 倒车距离: %.2fm, 速度: %.2fm/s",
                robot_pose.position.x, robot_pose.position.y, moveToMessage.x, moveToMessage.y,
                std::hypot(moveToMessage.x - robot_pose.position.x, moveToMessage.y - robot_pose.position.y),
                moveToMessage.edgeInfo.maxSpeed);

    return true;
}

bool PathNavigationExecutor::startStraight(const MoveToMessage &moveToMessage)
{
    const auto parameter_error = validateNavigationMode(
        "STRAIGHT", moveToMessage.edgeInfo.id, moveToMessage.edgeInfo.controlPoints,
        moveToMessage.edgeInfo.backUp, moveToMessage.edgeInfo.reverse);
    if (!parameter_error.empty()) {
        context_.failTask("FAILED", parameter_error);
        return false;
    }


    auto current_pose_opt = context_.executionPose();
    if (!current_pose_opt.has_value())
    {
        RCLCPP_ERROR(context_.executionLogger(), "机器人当前位置信息为空"); // 之前已经判断过，这里利落上不会出现为空的情况
        context_.failTask("FAILED", "无法获取机器人位姿（定位不可用），未执行沿路径导航");
        return false;
    }
    const auto &robot_pose = current_pose_opt.value().pose;


    // 🔴 新增: 判断是否倒车
    if (moveToMessage.edgeInfo.backUp)
    {
        RCLCPP_INFO(context_.executionLogger(), "检测到倒车标志,使用倒车模式导航");
        return startBackUp(moveToMessage);
    }

    // 计算robot当前朝向 和robot和目标点连接成的直线路线的角度差
    double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
    double target_yaw = std::atan2(moveToMessage.y - robot_pose.position.y, moveToMessage.x - robot_pose.position.x);
    double angle_diff = angles::shortest_angular_distance(robot_yaw, target_yaw);
    RCLCPP_INFO(context_.executionLogger(), "导航到点，计算夹角: 目标点(%.2f, %.2f, %.2f), 当前朝向:%.2f°, 目标朝向:%.2f°, 角度差:%.2f°",
                moveToMessage.x, moveToMessage.y, moveToMessage.theta * 180 / M_PI,
                robot_yaw * 180 / M_PI, target_yaw * 180 / M_PI, angle_diff * 180 / M_PI);

    if (std::abs(angle_diff) > 20.0 * M_PI / 180.0) // 超过20°先预旋转，小角度由路径控制器修正
    {
        // 保存导航信息，旋转完成后继续
        if (!context_.sendSpinCommand(angle_diff))
        {
            RCLCPP_ERROR(context_.executionLogger(), "直线路点导航，发送旋转命令失败");
            return false;
        }
        context_.setState(NavigationState::PRE_ROTATING);
        return true;
    }
    else
    {
        return executeStraight(robot_pose, moveToMessage);
    }
}

bool PathNavigationExecutor::executeStraight(const geometry_msgs::msg::Pose &robot_pose, const MoveToMessage &msg)
{
    double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
    auto path = StraightPathBuilder::build(robot_pose, msg, context_.executionLogger());

    // 创建 FollowPath 目标
    auto goal_msg = FollowPath::Goal();
    goal_msg.path.poses = path;
    goal_msg.path.header.frame_id = "map";
    goal_msg.controller_id = "FollowPath";

    context_.setSpeedLimit(msg.edgeInfo.maxSpeed);

    if (!context_.sendPathGoal(goal_msg)) return false;

    context_.setState(NavigationState::EXECUTING);

    RCLCPP_INFO(context_.executionLogger(),
                "开始导航: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f), command_id: %s",
                robot_pose.position.x, robot_pose.position.y, robot_yaw, msg.x, msg.y, msg.theta, msg.commandId.c_str());

    return true;
}

bool PathNavigationExecutor::startCurve(const MoveToMessage &moveToMessage)
{
    const auto parameter_error = validateNavigationMode(
        "CURVE", moveToMessage.edgeInfo.id, moveToMessage.edgeInfo.controlPoints,
        moveToMessage.edgeInfo.backUp, moveToMessage.edgeInfo.reverse);
    if (!parameter_error.empty()) {
        context_.failTask("FAILED", parameter_error);
        return false;
    }


    auto current_pose_opt = context_.executionPose();
    if (!current_pose_opt.has_value())
    {
        RCLCPP_DEBUG(context_.executionLogger(), "机器人当前位置信息为空");
        context_.failTask("FAILED", "无法获取机器人位姿（定位不可用），未执行沿路径导航");
        return false;
    }

    const geometry_msgs::msg::Pose &robot_pose = current_pose_opt.value().pose;

    double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);


    // 获取边和已反转的控制点（如果reverse_path，外部已处理反转）
    std::vector<double> control_x, control_y;
    for (const auto &cp : moveToMessage.edgeInfo.controlPoints)
    {
        control_x.push_back(cp.x);
        control_y.push_back(cp.y);
    }
    if (moveToMessage.edgeInfo.reverse)
    {
        std::reverse(control_x.begin(), control_x.end());
        std::reverse(control_y.begin(), control_y.end());
    }

    // 计算起点切线方向（t=0）
    double target_raw = BezierPathBuilder::calculateBezierTangent(0.0, robot_pose.position.x, robot_pose.position.y,
                                               control_x, control_y, moveToMessage.x, moveToMessage.y);

    double angle_diff = angles::shortest_angular_distance(robot_yaw, target_raw);

    RCLCPP_INFO(context_.executionLogger(), "贝塞尔曲线起点切线方向: %.2f°, 当前朝向: %.2f°, 角度差: %.2f°",
                target_raw * 180 / M_PI, robot_yaw * 180 / M_PI, angle_diff * 180 / M_PI);
    if (std::abs(angle_diff) > 20.0 * M_PI / 180.0)
    {
        if (!context_.sendSpinCommand(angle_diff)) // 旋转到切线方向
        {
            RCLCPP_ERROR(context_.executionLogger(), "贝塞尔曲线到点，发送旋转命令失败");
            return false;
        }
        context_.setState(NavigationState::PRE_ROTATING);
        return true;
    }
    else
    {
        return executeCurve(robot_pose, moveToMessage);
    }
}

bool PathNavigationExecutor::executeCurve(const geometry_msgs::msg::Pose &robot_pose, const MoveToMessage &moveToMessage)
{
    auto path = BezierPathBuilder::build(robot_pose, moveToMessage, context_.executionLogger()); // 生成贝塞尔曲线路径
    if (path.empty())
    {
        RCLCPP_ERROR(context_.executionLogger(), "Failed to generate Bezier path");
        return false;
    }
    context_.setSpeedLimit(moveToMessage.edgeInfo.maxSpeed); // 发布速度限制

    // 创建FollowPath目标
    auto goal_msg = FollowPath::Goal();
    goal_msg.path.poses = path;
    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = context_.executionNow();
    goal_msg.controller_id = "FollowPath";

    // 设置回调选项
    if (!context_.sendPathGoal(goal_msg)) return false;
    context_.setState(NavigationState::EXECUTING);
    return true;
}
void PathNavigationExecutor::onAccepted() {
    context_.reportAccepted("准备执行导航到点了");
}

void PathNavigationExecutor::onFeedback(const nav2_msgs::action::FollowPath::Feedback &feedback) {
    RCLCPP_DEBUG(context_.executionLogger(), "FollowPath feedback: distance remaining: %.2f", feedback.distance_to_goal);
}

void PathNavigationExecutor::onResult(rclcpp_action::ResultCode code, const MoveToMessage &task) {
    switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
        const auto pose = context_.executionPose();
        if (!pose) {
            context_.failTask("FAILED", "路径跟踪结束但无法取得新鲜定位位姿，未执行最终旋转");
            return;
        }
        const auto yaw = TransformUtils::quaternion_to_yaw(pose->pose.orientation);
        const auto angle = angles::shortest_angular_distance(yaw, task.theta);
        RCLCPP_INFO(context_.executionLogger(),
            "followPathResultCallback,目标点方向: %.2f°, 当前朝向: %.2f°, 角度差: %.2f°",
            task.theta * 180 / M_PI, yaw * 180 / M_PI, angle * 180 / M_PI);
        if (task.endPoint && std::abs(angle) > M_PI / 40) {
            if (!context_.sendSpinCommand(angle)) {
                context_.failTask("FAILED", "调用nav2 spin接口，发送旋转请求失败");
                return;
            }
            context_.setState(NavigationState::END_ROTATING);
        } else {
            context_.finishTask(true, "SUCCESS", "无需最终旋转，即导航到点完成!");
        }
        break;
    }
    case rclcpp_action::ResultCode::ABORTED:
        context_.failTask("FAILED", "路点导航请求终止！"); break;
    case rclcpp_action::ResultCode::CANCELED:
        context_.finishTask(false, "CANCELED", "路点导航请别取消！"); break;
    default:
        // Preserve existing policy; shared uncertain-terminal handling is stage 3.
        break;
    }
}

void PathNavigationExecutor::onSpinSucceeded(NavigationState state,
    const geometry_msgs::msg::Pose &pose, const MoveToMessage &task) {
    if (state == NavigationState::PRE_ROTATING) {
        if (task.edgeInfo.isBezierCurve()) executeCurve(pose, task);
        else executeStraight(pose, task);
    } else if (state == NavigationState::BACKING_UP) {
        // Legacy continuation only: normal backup entry never requests Spin.
        auto path = BackUpPathBuilder::build(pose, task, context_.executionLogger());
        if (path.empty()) {
            context_.failTask("FAILED", "倒车旋转后生成路径失败");
            return;
        }
        context_.setSpeedLimit(-task.edgeInfo.maxSpeed);
        nav2_msgs::action::FollowPath::Goal goal;
        goal.path.poses = path;
        goal.path.header.frame_id = "map";
        goal.path.header.stamp = context_.executionNow();
        goal.controller_id = "FollowPath";
        if (!context_.sendPathGoal(goal)) return;
        RCLCPP_INFO(context_.executionLogger(), "倒车旋转完成,开始倒车移动");
    } else if (state == NavigationState::END_ROTATING) {
        context_.finishTask(true, "SUCCESS", "经过最终旋转后，导航到点完成!");
    } else {
        RCLCPP_ERROR(context_.executionLogger(), "旋转完成，但是当前错误状态:%d", static_cast<int>(state));
    }
}
} // namespace agv_bridge
