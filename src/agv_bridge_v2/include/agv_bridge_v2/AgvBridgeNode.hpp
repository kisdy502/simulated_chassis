#ifndef AGV_BRIDGE_NODE_HPP
#define AGV_BRIDGE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <nlohmann/json.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>

#include "agv_bridge_v2/websocket_server.hpp"
#include "agv_bridge_v2/HttpApiClient.hpp"
#include "agv_bridge_v2/NavigationManager.hpp"
#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include "agv_bridge_v2/bean/Edge.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"
#include "agv_bridge_v2/ImuManager.hpp"

namespace agv_bridge
{

    class AgvBridgeNode : public rclcpp::Node, public agv_bridge::NavigationCallbacks
    {
    public:
        /**
         * @brief 构造函数
         */
        AgvBridgeNode();

        /**
         * @brief 析构函数
         */
        ~AgvBridgeNode() noexcept override;

        // 禁止拷贝和移动
        AgvBridgeNode(const AgvBridgeNode &) = delete;
        AgvBridgeNode &operator=(const AgvBridgeNode &) = delete;
        AgvBridgeNode(AgvBridgeNode &&) = delete;
        AgvBridgeNode &operator=(AgvBridgeNode &&) = delete;

    protected:
        // 导航回调接口实现
        void onNavigationStateChanged(agv_bridge::NavigationState state,
                                      const std::string &command_id) override;

        void onNavigationFeedback(double x, double y, double theta) override;

        void onNavigationResult(bool success,
                                const std::string &message,
                                const std::string &command_id,
                                const std::string &node_id) override;

        void onCommandAck(const std::string &command_id,
                          const std::string &node_id,
                          const std::string &status,
                          const std::string &message) override;

    private:
        // 初始化方法
        void initialize_parameters();
        void initialize_components();
        void create_subscriptions();
        void create_publishers();
        void create_timers();
        void start_websocket_server();

        // 回调方法
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        // void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        // 定时器回调
        void publish_status();
        void send_heartbeat();
        void broadcast_tf();
        void update_localization_monitor();
        void report_position();

        // WebSocket消息处理
        void handle_websocket_message(const std::string &message);
        void handle_move_command(const nlohmann::json &data);
        void handle_initial_pose(const nlohmann::json &data);
        void handle_velocity_command(const nlohmann::json &data);
        void handle_agv_control(const nlohmann::json &data);

        // 工具方法
        // double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &quat);
        // geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
        std::string get_current_time_iso() const;
        int64_t get_current_timestamp_ms() const;
        
        // 成员变量
        // 参数
        std::string agv_id_;
        std::string springboot_host_;
        int springboot_port_;
        int websocket_port_;
        std::string robot_namespace_ = "/";

        // 状态
        // std::optional<geometry_msgs::msg::Pose> current_pose_;
        std::optional<geometry_msgs::msg::Twist> current_velocity_;
        double battery_level_;
        std::string agv_state_;
        rclcpp::Time last_scan_time_;

        // ROS2组件
        // 订阅器
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
   

        // 发布器
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr agv_control_pub_;

        // 定时器
        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
        rclcpp::TimerBase::SharedPtr localization_timer_;

        // TF广播器
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // 管理组件
        std::shared_ptr<WebSocketServer> websocket_server_;
        std::shared_ptr<HttpApiClient> http_client_;
        std::shared_ptr<agv_bridge::NavigationManager> navigation_manager_;
        std::shared_ptr<agv_bridge::LocalizationMonitor> localization_monitor_;

        // 同步
        mutable std::mutex mutex_;
        geometry_msgs::msg::Twist current_vel_; // 速度信息（可从 odom 获取）

        // 定时上报位置信息
        rclcpp::TimerBase::SharedPtr position_report_timer_;
        // 可配置上报周期（毫秒）
        int report_interval_ = 400; // 默认300毫秒

        std::string imu_serial_port_;
        int imu_baud_rate_;
        int imu_window_size_;
        double imu_gyro_limit_;
        double imu_acc_limit_;
        std::shared_ptr<ImuManager> imu_manager_;
    };

} // namespace agv_bridge

#endif // AGV_BRIDGE_NODE_HPP