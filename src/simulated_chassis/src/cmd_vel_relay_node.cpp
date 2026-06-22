/**
 * 将标准速度话题 /cmd_vel 转发到阿克曼速度控制话题
 * /ackermann_steering_controller/reference_unstamped
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelRelayNode : public rclcpp::Node {
public:
  CmdVelRelayNode() : Node("cmd_vel_relay_node") {

    this->declare_parameter<std::string>("input_topic", "/cmd_vel");
    this->declare_parameter<std::string>(
        "output_topic", "/three_wheel_base_controller/cmd_vel");

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        output_topic, 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_vel_pub_->publish(*msg);
        });

    RCLCPP_INFO(this->get_logger(), "Relaying %s -> %s", input_topic.c_str(),
                output_topic.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelRelayNode>());
  rclcpp::shutdown();
  return 0;
}