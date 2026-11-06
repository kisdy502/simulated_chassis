#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace lightning {

// Both transforms must describe the same measurement time. Allow controller TF
// to arrive after the pose without blocking the SLAM or localization callback.
class MapOdomTf {
   public:
    MapOdomTf(rclcpp::Node& node, const std::string& odom_frame)
        : odom_frame_(odom_frame),
          buffer_(node.get_clock()),
          listener_(buffer_),
          broadcaster_(node),
          logger_(node.get_logger()) {
        RCLCPP_INFO(logger_, "Publishing map->%s from timestamp-aligned base_link TF", odom_frame_.c_str());
        timer_ = node.create_wall_timer(std::chrono::milliseconds(10), [this] {
            std::lock_guard<std::mutex> lock(mutex_);
            PublishPending();
        });
    }

    void Update(const geometry_msgs::msg::PoseStamped& pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (stamp < last_received_) {
            pending_.clear();
            buffer_.clear();
            last_published_ = -1;
        }
        last_received_ = stamp;
        pending_.push_back({pose, std::chrono::steady_clock::now()});
        if (pending_.size() > 100) {
            pending_.pop_front();
        }
        PublishPending();
    }

   private:
    void PublishPending() {
        while (!pending_.empty()) {
            const auto& entry = pending_.front();
            const auto& pose = entry.pose;
            const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
            if (stamp <= last_published_) {
                pending_.pop_front();
                continue;
            }

            geometry_msgs::msg::TransformStamped base_from_odom;
            try {
                base_from_odom = buffer_.lookupTransform("base_link", odom_frame_,
                                                        tf2_ros::fromMsg(pose.header.stamp));
            } catch (const tf2::TransformException& e) {
                if (std::chrono::steady_clock::now() - entry.received <= std::chrono::milliseconds(500)) {
                    return;
                }
                RCLCPP_WARN_THROTTLE(logger_, warning_clock_, 5000,
                                     "Dropping map->%s update: %s", odom_frame_.c_str(), e.what());
                pending_.pop_front();
                continue;
            }

            const auto& p = pose.pose.position;
            const auto& q = pose.pose.orientation;
            const auto& t = base_from_odom.transform.translation;
            const auto& r = base_from_odom.transform.rotation;
            const tf2::Transform map_from_base(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(p.x, p.y, p.z));
            const tf2::Transform base_from_odom_tf(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(t.x, t.y, t.z));
            const auto map_from_odom_tf = map_from_base * base_from_odom_tf;

            geometry_msgs::msg::TransformStamped output;
            output.header = pose.header;
            output.child_frame_id = odom_frame_;
            const auto translation = map_from_odom_tf.getOrigin();
            const auto rotation = map_from_odom_tf.getRotation();
            output.transform.translation.x = translation.x();
            output.transform.translation.y = translation.y();
            output.transform.translation.z = translation.z();
            output.transform.rotation.x = rotation.x();
            output.transform.rotation.y = rotation.y();
            output.transform.rotation.z = rotation.z();
            output.transform.rotation.w = rotation.w();
            broadcaster_.sendTransform(output);
            last_published_ = stamp;
            pending_.pop_front();
        }
    }

    struct PendingPose {
        geometry_msgs::msg::PoseStamped pose;
        std::chrono::steady_clock::time_point received;
    };
    std::string odom_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
    rclcpp::Logger logger_;
    rclcpp::Clock warning_clock_{RCL_STEADY_TIME};
    std::mutex mutex_;
    std::deque<PendingPose> pending_;
    int64_t last_received_ = -1;
    int64_t last_published_ = -1;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lightning
