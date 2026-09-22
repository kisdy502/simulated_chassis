#ifndef AGV_NAV_SERVER_NODE_HPP
#define AGV_NAV_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "agv_bridge_v2_interfaces/action/follow_edge.hpp"
#include "agv_bridge_v2_interfaces/msg/agv_status.hpp"
#include "agv_bridge_v2_interfaces/srv/set_control.hpp"
#include "agv_bridge_v2_interfaces/srv/get_map.hpp"
#include "agv_bridge_v2_interfaces/srv/load_map.hpp"
#include "agv_bridge_v2_interfaces/srv/list_maps.hpp"
#include "agv_bridge_v2_interfaces/srv/start_mapping.hpp"
#include "agv_bridge_v2_interfaces/srv/save_map.hpp"

#include "cartographer_ros_msgs/srv/write_state.hpp"

#include "agv_bridge_v2/NavigationManager.hpp"
#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include "agv_bridge_v2/MapFileManager.hpp"
#include "agv_bridge_v2/bean/MoveToMessage.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <thread>
#include <vector>
#include <sys/types.h>

namespace agv_bridge
{

    /**
     * @brief AGV 导航服务节点（rosbridge 架构下的「瘦节点」）
     *
     * 职责边界（这是与旧 AgvBridgeNode 最大的区别）：
     *   ✅ 保留：贝塞尔/直线/倒车路径生成、预旋转、nav2 FollowPath/Spin 调度、限速下发
     *   ❌ 删除：WebSocketServer（libwebsockets）、HttpApiClient（libcurl）、
     *            bean/* JSON 序列化类、心跳与上报定时器
     *
     * 对外只暴露标准 ROS 2 接口，上位机通过 rosbridge_server 以 JSON 协议访问：
     *   action  /agv/follow_edge     agv_bridge_v2_interfaces/action/FollowEdge
     *   srv     /agv/set_control     agv_bridge_v2_interfaces/srv/SetControl
     *   srv     /agv/get_map         agv_bridge_v2_interfaces/srv/GetMap（导出栅格给上位机）
     *   srv     /agv/load_map        agv_bridge_v2_interfaces/srv/LoadMap（重启定位加载新图）
     *   srv     /agv/list_maps       agv_bridge_v2_interfaces/srv/ListMaps（列出可导入地图）
     *   srv     /agv/start_mapping   agv_bridge_v2_interfaces/srv/StartMapping（进入建图 MAPPING）
     *   srv     /agv/save_map        agv_bridge_v2_interfaces/srv/SaveMap（保存建图并回定位）
     *   topic   /agv/status          agv_bridge_v2_interfaces/msg/AgvStatus   (1Hz, transient_local)
     *   tf      map -> <agv_id>/base_link
     *
     * 上位机不需要再「注册 / 心跳 / 握手」，WS 连接本身即注册，连接断开即注销。
     */
    class AgvNavServerNode : public rclcpp::Node, public agv_bridge::NavigationCallbacks
    {
    public:
        using FollowEdge = agv_bridge_v2_interfaces::action::FollowEdge;
        using GoalHandleFollowEdge = rclcpp_action::ServerGoalHandle<FollowEdge>;
        using SetControl = agv_bridge_v2_interfaces::srv::SetControl;
        using GetMap = agv_bridge_v2_interfaces::srv::GetMap;
        using LoadMap = agv_bridge_v2_interfaces::srv::LoadMap;
        using ListMaps = agv_bridge_v2_interfaces::srv::ListMaps;
        using StartMapping = agv_bridge_v2_interfaces::srv::StartMapping;
        using SaveMap = agv_bridge_v2_interfaces::srv::SaveMap;
        using AgvStatus = agv_bridge_v2_interfaces::msg::AgvStatus;

        /// 业务模式（/agv/status.mode）：与导航任务状态 state 正交
        static constexpr const char *MODE_NAVIGATION = "NAVIGATION";
        static constexpr const char *MODE_RELOCALIZING = "RELOCALIZING";
        static constexpr const char *MODE_MAPPING = "MAPPING";

        explicit AgvNavServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~AgvNavServerNode() noexcept override;

        AgvNavServerNode(const AgvNavServerNode &) = delete;
        AgvNavServerNode &operator=(const AgvNavServerNode &) = delete;

    protected:
        // ===== NavigationCallbacks 实现：把 NavigationManager 的回调映射成 action 反馈/结果 =====
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
        // ===== 初始化 =====
        void initialize_parameters();
        void initialize_components();
        void create_interfaces();
        void create_timers();

        // ===== action 回调 =====
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FollowEdge::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFollowEdge> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle);

        // ===== service 回调 =====
        void handle_set_control(
            std::shared_ptr<SetControl::Request> request,
            std::shared_ptr<SetControl::Response> response);

        void handle_get_map(
            std::shared_ptr<GetMap::Request> request,
            std::shared_ptr<GetMap::Response> response);

        void handle_load_map(
            std::shared_ptr<LoadMap::Request> request,
            std::shared_ptr<LoadMap::Response> response);

        void handle_list_maps(
            std::shared_ptr<ListMaps::Request> request,
            std::shared_ptr<ListMaps::Response> response);

        void handle_start_mapping(
            std::shared_ptr<StartMapping::Request> request,
            std::shared_ptr<StartMapping::Response> response);

        void handle_save_map(
            std::shared_ptr<SaveMap::Request> request,
            std::shared_ptr<SaveMap::Response> response);

        // ===== 定时器回调 =====
        void publish_status();
        void publish_feedback();
        void update_localization_monitor();
        void broadcast_tf();

        // ===== 工具 =====
        /// @brief 把 FollowEdge::Goal 翻译成 NavigationManager 需要的 MoveToMessage
        static MoveToMessage to_move_to_message(const FollowEdge::Goal &goal);

        /// @brief 以 ABORT 结束当前 goal（并清空句柄）
        void abort_current_goal(const std::string &message);

        /// @brief 清空当前 goal 句柄
        void reset_goal();

        // ===== 地图 / 定位 / 建图进程管理 =====
        /// @brief 相对路径按启动 cwd 转绝对路径（pbstream_file 参数用）
        static std::string absolute_path(const std::string &path);

        /// @brief 取路径的文件名主干（去掉目录与最后一个扩展名），用于 map_name
        static std::string file_stem(const std::string &path);

        /// @brief fork/exec 拉起 `ros2 launch <package> <launch_file> <extra_args...>`，
        /// 独立进程组（便于整组终止），输出重定向到 log_path，pid 写入 pid_slot
        bool spawn_ros_launch(const std::string &package, const std::string &launch_file,
                              const std::vector<std::string> &extra_args,
                              pid_t &pid_slot, const char *log_path);

        /// @brief 停止 spawn_ros_launch 拉起的子进程（SIGINT 优雅退出，超时 SIGKILL 整组）
        void stop_child_process(pid_t &pid_slot, const char *what);

        /// @brief 同步执行外部命令（fork/exec + wait），成功返回 true
        bool run_command_sync(const std::vector<std::string> &args, const char *log_path);

        /// @brief 调 cartographer /write_state 保存 pbstream（超时 15s）
        bool call_write_state(const std::string &pbstream_abs_path, std::string &error);

        // ===== 参数 =====
        std::string agv_id_ = "AGV001";
        double battery_level_ = 100.0;
        int feedback_interval_ms_ = 400;
        bool enable_tf_broadcast_ = true;
        bool use_sim_time_ = false;

        // ===== 地图管理参数 =====
        std::string maps_dir_ = "maps";
        std::string pbstream_file_;                          // 初始定位地图（空 = 不托管定位）
        std::string robot_package_ = "jzt_robot";            // 机器人包（定位/建图 launch 所在包的默认值）
        std::string localization_launch_package_;            // 空 = 用 robot_package_
        std::string localization_launch_file_ = "localization.launch.py";
        std::string slam_launch_package_;                    // 空 = 用 robot_package_
        std::string slam_launch_file_ = "slam.launch.py";

        // ===== 状态 =====
        std::atomic<bool> control_stopped_{false};
        std::atomic<bool> cancel_requested_{false};
        std::string agv_state_ = "IDLE";
        std::string active_command_id_;
        std::string active_node_id_;

        // 业务模式与当前地图（mode_mutex_ 保护；定位收敛后由定时器把
        // RELOCALIZING 切回 NAVIGATION）
        mutable std::mutex mode_mutex_;
        std::string mode_ = MODE_NAVIGATION;
        std::string map_name_;

        // ===== 地图 / 定位 / 建图子进程管理 =====
        std::unique_ptr<MapFileManager> map_file_manager_;
        pid_t localization_pid_ = -1;                 // 定位子进程（proc_mutex_ 保护）
        pid_t slam_pid_ = -1;                         // 建图子进程（proc_mutex_ 保护）
        mutable std::mutex proc_mutex_;
        std::atomic<bool> transition_in_progress_{false};   // 模式切换互斥（load/start_mapping/save_map）
        std::thread transition_thread_;               // 模式切换后台线程（析构时 join）
        rclcpp::Client<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_client_;

        // ===== ROS 组件 =====
        rclcpp_action::Server<FollowEdge>::SharedPtr follow_edge_server_;
        rclcpp::Service<SetControl>::SharedPtr set_control_srv_;
        rclcpp::Service<GetMap>::SharedPtr get_map_srv_;
        rclcpp::Service<LoadMap>::SharedPtr load_map_srv_;
        rclcpp::Service<ListMaps>::SharedPtr list_maps_srv_;
        rclcpp::Service<StartMapping>::SharedPtr start_mapping_srv_;
        rclcpp::Service<SaveMap>::SharedPtr save_map_srv_;
        rclcpp::Publisher<AgvStatus>::SharedPtr status_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr feedback_timer_;
        rclcpp::TimerBase::SharedPtr localization_timer_;
        rclcpp::TimerBase::SharedPtr tf_timer_;

        // ===== 业务组件（全部复用，未做修改） =====
        std::shared_ptr<agv_bridge::LocalizationMonitor> localization_monitor_;
        std::shared_ptr<agv_bridge::NavigationManager> navigation_manager_;

        // ===== 同步 =====
        mutable std::mutex goal_mutex_;
        std::shared_ptr<GoalHandleFollowEdge> current_goal_handle_;
    };

} // namespace agv_bridge

#endif // AGV_NAV_SERVER_NODE_HPP
