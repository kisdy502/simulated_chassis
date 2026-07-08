#include "agv_bridge_v2/AgvBridgeNode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    try {
        // 初始化ROS2
        rclcpp::init(argc, argv);
        
        // 创建节点
        auto node = std::make_shared<agv_bridge::AgvBridgeNode>();
        
        // 记录启动信息
        RCLCPP_INFO(node->get_logger(), "AGV Bridge Node started successfully");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
        
        // 运行节点
        rclcpp::spin(node);
        
        // 清理
        rclcpp::shutdown();
        RCLCPP_INFO(node->get_logger(), "AGV Bridge Node shutdown complete");
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "AGV Bridge Node fatal error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "AGV Bridge Node unknown fatal error" << std::endl;
        return 2;
    }
}