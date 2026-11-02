#pragma once
#include "agv_bridge_v2/navigation/NavigationState.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <optional>
#include <string>

namespace agv_bridge {
// Narrow, synchronous boundary between mode-specific flow and shared task
// ownership. Executors own neither ROS clients nor action handles. The Manager
// invokes them under its existing task lock and generation-checked callbacks.
class NavigationExecutionContext {
public:
    virtual ~NavigationExecutionContext() = default;
    virtual const rclcpp::Logger &executionLogger() const = 0;
    virtual rclcpp::Time executionNow() const = 0;
    virtual std::optional<geometry_msgs::msg::PoseStamped> executionPose() const = 0;
    virtual double backUpHeadingLimit() const = 0;
    virtual bool navigateServerReady() const = 0;
    virtual bool sendNavigateGoal(const nav2_msgs::action::NavigateToPose::Goal &goal) = 0;
    virtual bool sendPathGoal(const nav2_msgs::action::FollowPath::Goal &goal) = 0;
    virtual bool sendSpinCommand(double angle) = 0;
    virtual void setSpeedLimit(double speed) = 0;
    virtual void setState(NavigationState state) = 0;
    virtual void finishTask(bool success, const std::string status, const std::string message) = 0;
    virtual void failTask(const std::string status, const std::string message) = 0;
    virtual void reportAccepted(const std::string &message) = 0;
    virtual void reportFeedback(double x, double y, double yaw) = 0;
};
} // namespace agv_bridge
