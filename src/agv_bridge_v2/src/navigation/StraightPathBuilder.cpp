#include "agv_bridge_v2/navigation/StraightPathBuilder.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <cmath>

namespace agv_bridge {
    std::vector<geometry_msgs::msg::PoseStamped> StraightPathBuilder::build(const geometry_msgs::msg::Pose &robot_pose,
                                                                                             const agv_bridge::MoveToMessage &msg, const rclcpp::Logger &logger)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        double dx = msg.x - robot_pose.position.x;
        double dy = msg.y - robot_pose.position.y;
        double distance = std::hypot(dx, dy);

        // 计算点数（最少10个点，确保平滑）
        int num_points = std::max(10, static_cast<int>(distance / msg.edgeInfo.step) + 1);


        // 路径点保留零时间戳；Path整体header由导航调度方设置。

        // 直线方向
        double edge_yaw = std::atan2(dy, dx);

        RCLCPP_INFO(logger,
                    "生成直线路径: 起点(%.2f,%.2f) → 终点(%.2f,%.2f), 距离=%.2fm, 点数=%d",
                    robot_pose.position.x, robot_pose.position.y, msg.x, msg.y, distance, num_points);

        // 🔴 简化：只生成直线点，所有点朝向为直线方向
        // 旋转由 Nav2 行为树处理
        for (int i = 0; i < num_points; ++i)
        {
            double ratio = static_cast<double>(i) / (num_points - 1);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            // 时间戳设为0，Nav2会使用收到消息时的当前时间
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.position.x = robot_pose.position.x + dx * ratio;
            pose.pose.position.y = robot_pose.position.y + dy * ratio;
            pose.pose.position.z = 0.0;
            // 所有点朝向直线方向（Nav2 会在到达后处理最终旋转）
            pose.pose.orientation = TransformUtils::yaw_to_quaternion(edge_yaw);
            path.push_back(pose);
            RCLCPP_INFO(logger, "路点[%d](%.2f,%.2f), stamp: %d.%09d", i, pose.pose.position.x, pose.pose.position.y,
                        pose.header.stamp.sec, pose.header.stamp.nanosec);
        }

        // 确保最后一个点位置精确
        if (!path.empty())
        {
            path.back().pose.position.x = msg.x;
            path.back().pose.position.y = msg.y;
        }

        RCLCPP_INFO(logger, "路径生成完成: %zu 个点", path.size());
        return path;
    }
} // namespace agv_bridge
