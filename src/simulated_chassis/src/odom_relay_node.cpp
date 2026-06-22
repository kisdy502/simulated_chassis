/** 将阿克曼的里程计(/ackermann_steering_controller/odometry)转成标准odom
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomRelayNode : public rclcpp::Node {
public:
  OdomRelayNode() : Node("odom_relay_node") {
    this->declare_parameter<std::string>(
        "input_topic", "/ackermann_steering_controller/odometry");
    this->declare_parameter<std::string>("output_topic", "/odom");
    this->declare_parameter<bool>("publish_tf", false);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    bool publish_tf = this->get_parameter("publish_tf").as_bool();


    if (publish_tf) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        input_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          // 转发消息（完整保留 header）
          odom_pub_->publish(*msg);

          // 发布 TF
          if (tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped t;
            t.header = msg->header;
            t.child_frame_id = msg->child_frame_id;
            t.transform.translation.x = msg->pose.pose.position.x;
            t.transform.translation.y = msg->pose.pose.position.y;
            t.transform.translation.z = msg->pose.pose.position.z;
            t.transform.rotation = msg->pose.pose.orientation;
            tf_broadcaster_->sendTransform(t);
          }
        });

    RCLCPP_INFO(this->get_logger(), "Relaying %s -> %s, publish_tf=%s",
                input_topic.c_str(), output_topic.c_str(),
                publish_tf ? "true" : "false");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRelayNode>());
  rclcpp::shutdown();
  return 0;
}