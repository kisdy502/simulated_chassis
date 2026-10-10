/**
 * tracked_pose_tf_node：订阅 cartographer 的 /tracked_pose（map 系下 tracking_frame
 * 的高频外推位姿；最终 map 位姿仍可能包含三维地图对齐量），在同一时间戳查
 * odom→tracking 的 TF 后重锚定发布 map→odom。
 *
 * 背景：cartographer 直接发布的 map→odom 是「外推到 now 的 SLAM 位姿 ∘ t_slam
 * 旧时刻 odom 位姿⁻¹」，两个时刻不一致，与控制器的 odom→base 组合会把
 * t_slam→now 的运动计两次，造成前进-回退锯齿；而关闭外推后 map→odom 只随
 * local SLAM 结果（约10Hz+计算延迟）步进，RViz 里机器人卡顿。
 *
 * 本节点把锚定统一到 tracked_pose 的时间戳：
 *   map→odom(t) = T_map_tracking(t) ∘ T_tracking_odom(t)
 * 消费端组合 odom→base 后恒等于 tracked_pose 本身：平滑且无双重计数。
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <chrono>

class TrackedPoseTfNode : public rclcpp::Node {
public:
  TrackedPoseTfNode() : Node("tracked_pose_tf_node") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "base_link");
    const std::string topic =
        declare_parameter<std::string>("tracked_pose_topic", "/tracked_pose");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (stamp < last_received_stamp_) {
            pending_.clear();
            tf_buffer_->clear();
            last_published_stamp_ = -1;
            RCLCPP_WARN(get_logger(), "tracked_pose time moved backwards; cleared pending TF state");
          }
          last_received_stamp_ = stamp;
          last_received_ = std::chrono::steady_clock::now();
          pending_.push_back({msg, last_received_});
          if (pending_.size() > 100) {
            pending_.pop_front();
            ++dropped_;
          }
          processPending();
        });
    retry_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      processPending();
      const auto now = std::chrono::steady_clock::now();
      if (now - last_published_ > std::chrono::seconds(2) &&
          now - last_warning_ > std::chrono::seconds(5)) {
        last_warning_ = now;
        RCLCPP_WARN(get_logger(),
                    "map->odom output stalled: tracked_pose silence=%.2fs, output silence=%.2fs, pending=%zu, dropped=%zu, last TF error=%s",
                    std::chrono::duration<double>(now - last_received_).count(),
                    std::chrono::duration<double>(now - last_published_).count(),
                    pending_.size(), dropped_, last_error_.c_str());
      }
    });

    RCLCPP_INFO(get_logger(), "tracked_pose TF reanchoring: same timestamp, bounded 0.5s retry queue");
  }

private:
  void processPending() {
    while (!pending_.empty()) {
          const auto msg = pending_.front().pose;
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (stamp <= last_published_stamp_) {
            pending_.pop_front();
            continue;
          }
          const auto &p = msg->pose.position;
          const auto &q = msg->pose.orientation;
          const tf2::Quaternion q_map_tracking(q.x, q.y, q.z, q.w);
          const tf2::Vector3 t_map_tracking(p.x, p.y, p.z);

          geometry_msgs::msg::TransformStamped tracking_from_odom;
          try {
            tracking_from_odom = tf_buffer_->lookupTransform(
                tracking_frame_, odom_frame_, tf2_ros::fromMsg(msg->header.stamp));
          } catch (const tf2::TransformException &e) {
            last_error_ = e.what();
            if (std::chrono::steady_clock::now() - pending_.front().received >
                std::chrono::milliseconds(500)) {
              pending_.pop_front();
              ++dropped_;
              continue;
            }
            return;
          }

          const auto &to = tracking_from_odom.transform;
          const tf2::Quaternion q_tracking_odom(
              to.rotation.x, to.rotation.y, to.rotation.z, to.rotation.w);
          const tf2::Vector3 t_tracking_odom(
              to.translation.x, to.translation.y, to.translation.z);

          const tf2::Quaternion q_map_odom = q_map_tracking * q_tracking_odom;
          const tf2::Vector3 t_map_odom =
              t_map_tracking + tf2::quatRotate(q_map_tracking, t_tracking_odom);

          geometry_msgs::msg::TransformStamped map_from_odom;
          map_from_odom.header.stamp = msg->header.stamp;
          map_from_odom.header.frame_id = map_frame_;
          map_from_odom.child_frame_id = odom_frame_;
          map_from_odom.transform.rotation = tf2::toMsg(q_map_odom);
          map_from_odom.transform.translation.x = t_map_odom.x();
          map_from_odom.transform.translation.y = t_map_odom.y();
          map_from_odom.transform.translation.z = t_map_odom.z();
          tf_broadcaster_->sendTransform(map_from_odom);
          last_error_ = "none";
          pending_.pop_front();
          last_published_stamp_ = stamp;
          last_published_ = std::chrono::steady_clock::now();
    }
  }
  struct Pending {
    geometry_msgs::msg::PoseStamped::SharedPtr pose;
    std::chrono::steady_clock::time_point received;
  };
  std::deque<Pending> pending_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  int64_t last_received_stamp_ = -1, last_published_stamp_ = -1;
  size_t dropped_ = 0;
  std::string last_error_ = "none";
  std::chrono::steady_clock::time_point last_received_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_published_ = last_received_;
  std::chrono::steady_clock::time_point last_warning_ = last_received_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string tracking_frame_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackedPoseTfNode>());
  rclcpp::shutdown();
  return 0;
}
