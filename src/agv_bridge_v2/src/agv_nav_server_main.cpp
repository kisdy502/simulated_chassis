#include "agv_bridge_v2/AgvNavServerNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <exception>
#include <iostream>

/**
 * agv_nav_server —— rosbridge 架构下的 AGV 瘦节点
 *
 * 与旧 agv_bridge_node 的区别：
 *   - 不再启动 WebSocket 服务器（rosbridge_server 负责）与 HTTP 上报（上位机直接订阅 topic）
 *   - 只暴露标准 ROS 2 接口：/agv/follow_edge (action)、/agv/set_control (srv)、/agv/status (topic)
 *
 * 必须使用多线程执行器：NavigationManager::cancelNavigation() 会在回调线程内阻塞等待
 * nav2 的取消响应 future，单线程执行器会造成死锁。
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<agv_bridge::AgvNavServerNode>();

        RCLCPP_INFO(node->get_logger(), "AGV Nav Server started successfully");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
        RCLCPP_INFO(node->get_logger(), "AGV Nav Server shutdown complete");
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "AGV Nav Server fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    catch (...)
    {
        std::cerr << "AGV Nav Server unknown fatal error" << std::endl;
        rclcpp::shutdown();
        return 2;
    }
}
