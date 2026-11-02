#include "agv_bridge_v2/navigation/BackUpPathBuilder.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <cmath>

namespace agv_bridge {
    std::vector<geometry_msgs::msg::PoseStamped> BackUpPathBuilder::build(
        const geometry_msgs::msg::Pose &robot_pose, const agv_bridge::MoveToMessage &msg, const rclcpp::Logger &logger)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        const double dx = msg.x - robot_pose.position.x;
        const double dy = msg.y - robot_pose.position.y;
        const double distance = std::hypot(dx, dy);

        // 计算点数(最少10个点)
        const int num_points = std::max(10, static_cast<int>(distance / msg.edgeInfo.step) + 1);

        // 后退全程保持机器人「当前朝向」不变：
        // Nav2 控制器以路径点的朝向作为期望朝向，期望朝向 == 当前朝向 ⇒ 不产生转向量，
        // 配合负的 /speed_limit，车体沿当前朝向缓慢平移后退。
        // ⚠️ 切勿改成 atan2(dy, dx)（指向目标）。那会让控制器把车头转向目标，
        //    在充电桩 / 窄过道现场会造成剐蹭 —— 这正是必须避免的行为。
        const double hold_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);

        RCLCPP_INFO(logger,
                    "生成倒车路径(保持当前朝向): 起点(%.2f,%.2f) → 终点(%.2f,%.2f), 距离=%.2fm, 点数=%d, 保持朝向=%.2f°",
                    robot_pose.position.x, robot_pose.position.y, msg.x, msg.y,
                    distance, num_points, hold_yaw * 180 / M_PI);

        // 生成倒车路径点
        for (int i = 0; i < num_points; ++i)
        {
            double ratio = static_cast<double>(i) / (num_points - 1);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Time(0);

            // 插值位置
            pose.pose.position.x = robot_pose.position.x + dx * ratio;
            pose.pose.position.y = robot_pose.position.y + dy * ratio;
            pose.pose.position.z = 0.0;

            // ✅ 关键:所有点的朝向 = 机器人当前朝向，全程不转向
            //    位置在后退方向上插值，朝向保持不变，实现「保持姿态平移后退」
            pose.pose.orientation = TransformUtils::yaw_to_quaternion(hold_yaw);

            path.push_back(pose);
        }

        // 确保最后一个点位置精确
        if (!path.empty())
        {
            path.back().pose.position.x = msg.x;
            path.back().pose.position.y = msg.y;
        }

        RCLCPP_INFO(logger, "倒车路径生成完成: %zu 个点", path.size());
        return path;
    }
} // namespace agv_bridge
