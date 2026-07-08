#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
namespace agv_bridge
{

    LocalizationMonitor::LocalizationMonitor(rclcpp::Node *parent_node, const Config &config)
        : parent_node_(parent_node),
          logger_(parent_node->get_logger().get_child("LocalizationMonitor")),
          config_(config),
          is_initialized_(false),
          tf_initialized_(false),
          tf_stable_count_(0),
          tf_buffer_(parent_node_->get_clock()),
          tf_listener_(tf_buffer_)
    {

        if (!parent_node_)
        {
            throw std::invalid_argument("Parent node cannot be null");
        }

        // 10Hz 定时器，持续监听 TF 更新位姿缓存
        timer_ = parent_node->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&LocalizationMonitor::timerCallback, this));

        RCLCPP_INFO(logger_,
                    "LocalizationMonitor initialized (Cartographer mode): TF stable threshold=%d",
                    TF_STABLE_THRESHOLD);
    }

    void LocalizationMonitor::timerCallback()
    {
        try
        {
            // 查找最新变换（时间戳设为 0 表示获取最新可用变换）
            auto transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));

            geometry_msgs::msg::Pose pose;
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;

            // 角度连续性处理
            double raw_yaw = tf2::getYaw(pose.orientation); // 范围 [-π, π]

            std::lock_guard<std::mutex> lock(mutex_);
            if (!have_previous_)
            {
                // 第一帧：初始化连续偏航角为原始值
                continuous_yaw_ = raw_yaw;
                have_previous_ = true;
            }
            else
            {
                // 计算差值，处理环绕
                double diff = raw_yaw - last_raw_yaw_;
                if (diff > M_PI)
                    diff -= 2 * M_PI;
                else if (diff < -M_PI)
                    diff += 2 * M_PI;

                continuous_yaw_ += diff; // 累积连续角度
            }
            last_raw_yaw_ = raw_yaw;
            //latest_pose_ = pose; // 保存原始位姿（含四元数）
            pose_stamped.header.stamp = transform.header.stamp;
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose = pose;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(logger_, "TF lookup failed: %s", ex.what());
        }
    }

    void LocalizationMonitor::update()
    {
        checkTFStability();

        // Cartographer 纯定位模式：仅依赖 TF 树完整性判断初始化
        // map→odom→base_footprint 链路完整且时间戳持续更新 = 定位已收敛
        is_initialized_ = tf_initialized_;
    }

    bool LocalizationMonitor::isInitialized() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return is_initialized_;
    }

    // ============================================================
    //  checkTFStability() — Cartographer 纯定位模式
    //  判定依据：map→base_footprint TF 链完整 + 时间戳新鲜
    //  连续 TF_STABLE_THRESHOLD 次稳定才认为定位已初始化
    // ============================================================
    void LocalizationMonitor::checkTFStability()
    {
        try
        {
            // 每 15 秒打印一次完整 TF 树
            std::string tf_string = tf_buffer_.allFramesAsString();
            RCLCPP_INFO_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                 "Available TF frames:\n%s", tf_string.c_str());

            // 检查 map→base_footprint 链路是否完整
            if (!tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.1)))
            {
                RCLCPP_WARN_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                     "TF 链路未就绪: map → base_footprint 不可达");
                throw tf2::TransformException("Transform not available");
            }

            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_footprint", tf2::TimePointZero,
                tf2::durationFromSec(0.1));

            rclcpp::Time transform_time(transform.header.stamp);
            rclcpp::Time now = parent_node_->now();
            double time_diff = std::abs(transform_time.seconds() - now.seconds());

            RCLCPP_INFO_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                 "TF map→base_footprint: pos=(%.2f,%.2f), stamp=%.3f, now=%.3f, diff=%.3fs",
                                 transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform_time.seconds(), now.seconds(), time_diff);

            std::lock_guard<std::mutex> lock(mutex_);

            if (time_diff < 1.0)
            {
                tf_stable_count_++;
                RCLCPP_INFO_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                     "TF 稳定计数: %d/%d", tf_stable_count_, TF_STABLE_THRESHOLD);

                if (tf_stable_count_ >= TF_STABLE_THRESHOLD)
                {
                    tf_initialized_ = true;
                }
            }
            else
            {
                if (tf_stable_count_ > 0)
                {
                    RCLCPP_WARN_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                         "TF 时间戳过期 (diff=%.3fs)，重置稳定计数", time_diff);
                }
                tf_stable_count_ = 0;
                tf_initialized_ = false;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tf_stable_count_ = 0;
            tf_initialized_ = false;
            RCLCPP_WARN_THROTTLE(logger_, *parent_node_->get_clock(), 15000,
                                 "TF 查询失败: %s", ex.what());
        }
    }

    bool LocalizationMonitor::getTransform(
        const std::string &frame,
        const rclcpp::Time &time, // 关键：使用消息时间戳
        geometry_msgs::msg::TransformStamped &transform)
    {
        // // 1. 查询TF树
        try
        {
            transform = tf_buffer_.lookupTransform(
                "map",                      // 目标坐标系（如map或odom）
                frame,                      // 源坐标系（laser_link）
                time,                       // 使用指定时间戳
                tf2::durationFromSec(0.1)); // 容差时间
            return true;
        }
        catch (const tf2::TransformException &ex)
        {

            return false;
        }
    }

    // bool LocalizationMonitor::getRobotPose(geometry_msgs::msg::Pose &pose)
    // {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     if (!is_initialized_)
    //     {
    //         return false;
    //     }
    //     geometry_msgs::msg::TransformStamped transform;
    //     // 获取从 map 到 base_footprint（或 base_link）的最新变换
    //     if (getTransform("base_footprint", rclcpp::Time(0), transform))
    //     {
    //         pose.position.x = transform.transform.translation.x;
    //         pose.position.y = transform.transform.translation.y;
    //         pose.orientation = transform.transform.rotation;
    //         return true;
    //     }
    //     return false;
    // }

    std::optional<geometry_msgs::msg::PoseStamped> LocalizationMonitor::getCurrentPose() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_previous_)
            return std::nullopt;
        return pose_stamped;
    }

}