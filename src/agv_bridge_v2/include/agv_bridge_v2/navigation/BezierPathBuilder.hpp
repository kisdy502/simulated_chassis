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
class BezierPathBuilder {
public:
    static std::vector<geometry_msgs::msg::PoseStamped> build(
        const geometry_msgs::msg::Pose &robot_pose, const MoveToMessage &task,
        const rclcpp::Logger &logger);
    // Shared by pre-rotation and curve sampling; t is the curve parameter.
    static double calculateBezierTangent(double t, double p0x, double p0y,
        const std::vector<double> &cp_x, const std::vector<double> &cp_y,
        double p1x, double p1y);
private:
    static std::pair<double, double> calculateBezierPoint(double t, double p0x, double p0y,
        const std::vector<double> &cp_x, const std::vector<double> &cp_y,
        double p1x, double p1y);
    static double calculateBezierLength(double p0x, double p0y,
        const std::vector<double> &cp_x, const std::vector<double> &cp_y,
        double p1x, double p1y, int segments = 100);
};
} // namespace agv_bridge
