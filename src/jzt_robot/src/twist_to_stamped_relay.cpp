#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("twist_to_stamped_relay");

    node->declare_parameter("input_topic", "/cmd_vel");
    node->declare_parameter("output_topic", "/diff_drive_controller/cmd_vel");

    auto input = node->get_parameter("input_topic").as_string();
    auto output = node->get_parameter("output_topic").as_string();

    rclcpp::QoS qos(10);

    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(output, qos);

    // 统计计数器
    auto msg_count = std::make_shared<int>(0);
    auto last_vx   = std::make_shared<double>(0.0);
    auto last_wz   = std::make_shared<double>(0.0);

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        input, qos,
        [pub, node, msg_count, last_vx, last_wz](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            (*msg_count)++;
            *last_vx = msg->linear.x;
            *last_wz = msg->angular.z;
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = node->now();
            stamped.header.frame_id = "base_footprint";
            stamped.twist = *msg;
            pub->publish(stamped);
        });

    // 每 15 秒打印摘要
    auto timer = node->create_wall_timer(
        std::chrono::seconds(15),
        [node, input, output, msg_count, last_vx, last_wz]()
        {
            RCLCPP_INFO(node->get_logger(),
                        "[%s→%s] rx=%d | vx=%.3f m/s  wz=%.3f rad/s",
                        input.c_str(), output.c_str(),
                        *msg_count, *last_vx, *last_wz);
        });

    RCLCPP_INFO(node->get_logger(), "Relay: %s (Twist) → %s (TwistStamped)",
                input.c_str(), output.c_str());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}