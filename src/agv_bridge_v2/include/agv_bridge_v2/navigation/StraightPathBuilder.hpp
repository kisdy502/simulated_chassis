#pragma once
#include "agv_bridge_v2/bean/MoveToMessage.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <vector>
#include <utility>

namespace agv_bridge {
// Stateless geometry only: no node, action client, task state or callbacks.
// Input validation/speed defaults remain at the existing navigation boundary.
class StraightPathBuilder {
public:
    static std::vector<geometry_msgs::msg::PoseStamped> build(
        const geometry_msgs::msg::Pose &robot_pose, const MoveToMessage &task,
        const rclcpp::Logger &logger);
};
} // namespace agv_bridge
