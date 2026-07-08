// include/bridge/LocalizationMonitor.hpp
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // 提供 tf2::fromMsg
#include <tf2/utils.h>                                // 提供 tf2::getYaw

namespace agv_bridge
{
    class LocalizationMonitor
    {
    public:
        struct Config
        {
            double particle_spread_threshold = 1.0;
            int required_stable_count = 3;
            double quality_threshold = 0.8;
        };

        LocalizationMonitor(rclcpp::Node *parent_node, const Config &config);

        /// @brief 检查 TF 树完整性，更新初始化状态（由外部定时器驱动）
        void update();

        /// @brief 定位是否已初始化（map→base 变换可用且持续更新）
        bool isInitialized() const;

        /// @brief 查询指定 frame 到 map 的 TF 变换
        bool getTransform(
            const std::string &frame,
            const rclcpp::Time &time,
            geometry_msgs::msg::TransformStamped &transform);

        /// @brief 获取当前机器人位姿（map 坐标系下）
        std::optional<geometry_msgs::msg::PoseStamped> getCurrentPose() const;

    private:
        void checkTFStability();
        void timerCallback();

        rclcpp::Node *parent_node_;
        rclcpp::Logger logger_;  // 子logger，日志tag显示类名
        Config config_;

        bool is_initialized_ = false;
        bool tf_initialized_ = false;

        int tf_stable_count_ = 0;               // TF 连续稳定次数
        static constexpr int TF_STABLE_THRESHOLD = 5;  // 连续 N 次稳定才算初始化

        // TF2组件
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::PoseStamped pose_stamped;
        double continuous_yaw_; // 累积的连续偏航角（弧度）
        double last_raw_yaw_;   // 上一次的原始偏航角（用于差值计算）
        bool have_previous_;    // 是否有上一帧
        mutable std::mutex mutex_;
    };

}