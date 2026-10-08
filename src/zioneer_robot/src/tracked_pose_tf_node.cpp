/**
 * tracked_pose_tf_node：订阅 cartographer 的 /tracked_pose（map 系下 tracking_frame
 * 的高频外推位姿，已按 publish_frame_projected_to_2d 投影），在同一时间戳查
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
          const auto &p = msg->pose.position;
          const auto &q = msg->pose.orientation;
          const tf2::Quaternion q_map_tracking(q.x, q.y, q.z, q.w);
          const tf2::Vector3 t_map_tracking(p.x, p.y, p.z);

          geometry_msgs::msg::TransformStamped tracking_from_odom;
          try {
            tracking_from_odom = tf_buffer_->lookupTransform(
                tracking_frame_, odom_frame_, tf2_ros::fromMsg(msg->header.stamp));
          } catch (const tf2::TransformException &e) {
            // tracked_pose 时间戳略超前于最新 odom→base TF 时插值失败，
            // 丢弃本帧即可（后续高频位姿继续尝试同时间戳锚定）
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "lookup %s<- %s failed: %s",
                                  tracking_frame_.c_str(), odom_frame_.c_str(), e.what());
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
        });

    RCLCPP_INFO(get_logger(),
                "tracked_pose_tf_node: %s → %s←%s 重锚定（topic: %s）",
                map_frame_.c_str(), tracking_frame_.c_str(), odom_frame_.c_str(),
                topic.c_str());
  }

private:
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
