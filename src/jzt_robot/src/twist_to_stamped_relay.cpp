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

    // 默认 QoS (reliable)，兼容 Nav2 和 teleop_twist_keyboard
    rclcpp::QoS qos(10);

    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(output, qos);

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        input, qos,
        [pub, node](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = node->now();
            stamped.header.frame_id = "base_footprint";
            stamped.twist = *msg;
            pub->publish(stamped);
        });

    RCLCPP_INFO(node->get_logger(), "Relay: %s (Twist) → %s (TwistStamped)",
                input.c_str(), output.c_str());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}