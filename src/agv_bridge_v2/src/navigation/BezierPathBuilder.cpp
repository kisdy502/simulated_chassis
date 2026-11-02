#include "agv_bridge_v2/navigation/BezierPathBuilder.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <cmath>
#include "agv_bridge_v2/NavigationValidation.hpp"

namespace agv_bridge {
    std::pair<double, double> BezierPathBuilder::calculateBezierPoint(double t, double p0x, double p0y,
                                                                      const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y)

    {
        if (cp_x.empty() || cp_y.empty() || cp_x.size() != cp_y.size())
        {
            // 没有控制点，返回线性插值
            return {p0x + t * (p1x - p0x), p0y + t * (p1y - p0y)};
        }

        if (cp_x.size() == 1)
        {
            // 二阶贝塞尔曲线
            double u = 1.0 - t;
            double x = u * u * p0x + 2.0 * u * t * cp_x[0] + t * t * p1x;
            double y = u * u * p0y + 2.0 * u * t * cp_y[0] + t * t * p1y;
            return {x, y};
        }
        else if (cp_x.size() == 2)
        {
            // 三阶贝塞尔曲线
            double u = 1.0 - t;
            double u2 = u * u;
            double t2 = t * t;

            double x = u2 * u * p0x + 3.0 * u2 * t * cp_x[0] + 3.0 * u * t2 * cp_x[1] + t2 * t * p1x;
            double y = u2 * u * p0y + 3.0 * u2 * t * cp_y[0] + 3.0 * u * t2 * cp_y[1] + t2 * t * p1y;
            return {x, y};
        }

        // 不支持更高阶贝塞尔曲线
        return {p0x + t * (p1x - p0x), p0y + t * (p1y - p0y)};
    }

    double BezierPathBuilder::calculateBezierTangent(double t, double p0x, double p0y,
                                                     const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y)
    {
        if (cp_x.empty() || cp_y.empty() || cp_x.size() != cp_y.size())
        {
            // 直线
            return std::atan2(p1y - p0y, p1x - p0x);
        }

        if (cp_x.size() == 1)
        {
            // 二阶贝塞尔曲线导数
            double dx = 2.0 * (1.0 - t) * (cp_x[0] - p0x) + 2.0 * t * (p1x - cp_x[0]);
            double dy = 2.0 * (1.0 - t) * (cp_y[0] - p0y) + 2.0 * t * (p1y - cp_y[0]);
            return std::atan2(dy, dx);
        }
        else if (cp_x.size() == 2)
        {
            // 三阶贝塞尔曲线导数
            double u = 1.0 - t;
            double dx = 3.0 * u * u * (cp_x[0] - p0x) + 6.0 * u * t * (cp_x[1] - cp_x[0]) + 3.0 * t * t * (p1x - cp_x[1]);
            double dy = 3.0 * u * u * (cp_y[0] - p0y) + 6.0 * u * t * (cp_y[1] - cp_y[0]) + 3.0 * t * t * (p1y - cp_y[1]);
            return std::atan2(dy, dx);
        }

        return std::atan2(p1y - p0y, p1x - p0x);
    }

    double BezierPathBuilder::calculateBezierLength(double p0x, double p0y,
                                                    const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y, int segments)
    {
        double length = 0.0;
        double prev_x = p0x;
        double prev_y = p0y;

        for (int i = 1; i <= segments; ++i)
        {
            double t = static_cast<double>(i) / segments;
            auto [x, y] = calculateBezierPoint(t, p0x, p0y, cp_x, cp_y, p1x, p1y);

            double dx = x - prev_x;
            double dy = y - prev_y;
            length += std::hypot(dx, dy);

            prev_x = x;
            prev_y = y;
        }

        return length;
    }

    std::vector<geometry_msgs::msg::PoseStamped> BezierPathBuilder::build(
        const geometry_msgs::msg::Pose &robot_pose, const agv_bridge::MoveToMessage &move_msg, const rclcpp::Logger &logger)
    {
        const auto parameter_error = validateNavigationMode(
            "CURVE", move_msg.edgeInfo.id, move_msg.edgeInfo.controlPoints,
            move_msg.edgeInfo.backUp, move_msg.edgeInfo.reverse);
        if (!parameter_error.empty()) {
            RCLCPP_ERROR(logger, "%s", parameter_error.c_str());
            return {};
        }
        bool reverse_path = move_msg.nodeId == move_msg.edgeInfo.sourceId; // 目标点和通道起点相同，说明是reverse的，主要用途就是反转切点
        std::vector<geometry_msgs::msg::PoseStamped> path;
        // 处理贝塞尔曲线
        std::vector<double> control_x, control_y;
        const auto &cps = move_msg.edgeInfo.controlPoints; // 已经确认有值
        for (const auto &cp : cps)
        {
            control_x.push_back(cp.x);
            control_y.push_back(cp.y);
        }

        // 如果需要反向，反转控制点
        if (reverse_path)
        {
            std::reverse(control_x.begin(), control_x.end());
            std::reverse(control_y.begin(), control_y.end());
            RCLCPP_INFO(logger, "已反转贝塞尔曲线控制点顺序，控制点数量: %zu", control_x.size());
        }
        // 计算贝塞尔曲线长度
        double length = calculateBezierLength(robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);

        // 计算需要的点数
        int num_points = std::max(2, static_cast<int>(length / move_msg.edgeInfo.step) + 1);

        RCLCPP_INFO(logger, "生成贝塞尔曲线路点: 长度=%.2f, 路点数量=%d, 控制点个数=%zu, 通道速度=%.2f",
                    length, num_points, control_x.size(), move_msg.edgeInfo.maxSpeed);


        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);
            // 计算位置
            auto [x, y] = calculateBezierPoint(t, robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);
            // 计算朝向
            double yaw = calculateBezierTangent(t, robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);
            // 创建位姿
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Time(0);

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            // 第一个点和最后一个点设置角度，中间点间隔一个设置角度，防止频繁设置角度导致nav2 出错
            if (i == 1 || i == num_points - 1 || i % 2 == 0)
            {
                pose.pose.orientation = TransformUtils::yaw_to_quaternion(yaw);
            }

            path.push_back(pose);
            RCLCPP_INFO(logger, "路点[%d](%.2f,%.2f), stamp: %d.%09d", i, pose.pose.position.x, pose.pose.position.y,
                        pose.header.stamp.sec, pose.header.stamp.nanosec);
        }

        if (!path.empty())
        {
            path.back().pose.position.x = move_msg.x;
            path.back().pose.position.y = move_msg.y;

        }

        return path;
    }
} // namespace agv_bridge
