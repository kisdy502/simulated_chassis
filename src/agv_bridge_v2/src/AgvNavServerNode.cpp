#include "agv_bridge_v2/AgvNavServerNode.hpp"

#include "agv_bridge_v2/utils/TransformUtils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <optional>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cartographer_ros_msgs/msg/trajectory_states.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace agv_bridge;

namespace agv_bridge
{
    namespace
    {
        /// use_sim_time 参数串（"use_sim_time:=" + bool），const char* 不能直接拼接
        std::string useSimTimeArg(bool use_sim_time)
        {
            return "use_sim_time:=" + std::string(use_sim_time ? "true" : "false");
        }

    }


    // ============================ 构造 / 析构 ============================

    AgvNavServerNode::AgvNavServerNode(const rclcpp::NodeOptions &options)
        : Node("agv_nav_server", options)
    {
        initialize_parameters();
        initialize_components();
        create_interfaces();
        create_timers();

        // ROS 接口创建完毕后再异步拉起初始定位。主线程进入 executor 后，
        // 工作线程才能可靠收到 /start_trajectory 的响应。
        if (!pbstream_file_.empty())
        {
            const std::string pbstream_abs = absolute_path(pbstream_file_);
            const std::string name = file_stem(pbstream_abs);
            {
                std::lock_guard<std::mutex> lock(mode_mutex_);
                map_name_ = name;
                mode_ = MODE_RELOCALIZING;
            }
            transition_in_progress_.store(true);
            transition_thread_ = std::thread([this, pbstream_abs, name]()
            {
                geometry_msgs::msg::Pose remembered_pose;
                const bool has_pose = load_remembered_pose(name, remembered_pose);
                std::string error;
                if (!start_managed_localization(
                        pbstream_abs, name, has_pose ? &remembered_pose : nullptr, error))
                {
                    RCLCPP_ERROR(this->get_logger(), "初始定位启动失败: %s (has_pose=%d)",
                                 error.c_str(), has_pose);
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    mode_ = MODE_NAVIGATION;
                }
                transition_in_progress_.store(false);
            });
        }

        RCLCPP_INFO(this->get_logger(),
                    "AGV Nav Server 已启动 (agv_id=%s, map=%s)。上位机请通过 rosbridge 访问 "
                    "/agv/follow_edge、/agv/set_control、/agv/get_map、/agv/load_map、"
                    "/agv/list_maps、/agv/relocalize、/agv/status",
                    agv_id_.c_str(), map_name_.c_str());
    }

    AgvNavServerNode::~AgvNavServerNode() noexcept
    {
        try
        {
            // 退出时取消正在执行的导航，避免 nav2 控制器停留在最后一条路径
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "析构中取消导航失败: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "析构中发生未知异常");
        }

        // 回收模式切换后台线程，再停掉托管的定位/建图子进程
        if (transition_thread_.joinable())
        {
            transition_thread_.join();
        }
        stop_child_process(slam_pid_, "建图");
        stop_child_process(localization_pid_, "定位");
    }

    // ============================ 初始化 ============================

    void AgvNavServerNode::initialize_parameters()
    {
        this->declare_parameter<std::string>("agv_id", "AGV001");
        this->declare_parameter<double>("battery_level", 100.0);
        this->declare_parameter<int>("feedback_interval_ms", 400);
        this->declare_parameter<bool>("enable_tf_broadcast", true);

        // ===== 地图管理 =====
        this->declare_parameter<std::string>("maps_dir", "maps");
        this->declare_parameter<std::string>("pbstream_file", "");
        this->declare_parameter<std::string>("robot_package", "jzt_robot");
        this->declare_parameter<std::string>("localization_launch_package", "");
        this->declare_parameter<std::string>("localization_launch_file", "localization.launch.py");
        this->declare_parameter<std::string>("localization_configuration_basename", "");
        this->declare_parameter<std::string>("slam_launch_package", "");
        this->declare_parameter<std::string>("slam_launch_file", "slam.launch.py");
        // 注意：use_sim_time 由 rclcpp 内置自动声明，这里只能读取，不能重复 declare

        this->get_parameter("agv_id", agv_id_);
        this->get_parameter("battery_level", battery_level_);
        this->get_parameter("feedback_interval_ms", feedback_interval_ms_);
        this->get_parameter("enable_tf_broadcast", enable_tf_broadcast_);
        this->get_parameter("use_sim_time", use_sim_time_);

        this->get_parameter("maps_dir", maps_dir_);
        this->get_parameter("pbstream_file", pbstream_file_);
        this->get_parameter("robot_package", robot_package_);
        this->get_parameter("localization_launch_package", localization_launch_package_);
        this->get_parameter("localization_launch_file", localization_launch_file_);
        this->get_parameter("localization_configuration_basename", localization_configuration_basename_);
        this->get_parameter("slam_launch_package", slam_launch_package_);
        this->get_parameter("slam_launch_file", slam_launch_file_);

        // 定位/建图 launch 包名缺省跟随机器人包
        if (localization_launch_package_.empty())
        {
            localization_launch_package_ = robot_package_;
        }
        if (slam_launch_package_.empty())
        {
            slam_launch_package_ = robot_package_;
        }
        if (localization_configuration_basename_.empty())
        {
            // 三舵轮仿真已切换 Cartographer 2D（水平扫描，无地面回波/无z漂移），
            // 与 localization.launch.py 的默认配置保持一致；需要 3D 时显式传参覆盖。
            localization_configuration_basename_ = "localization_2d.lua";
        }

        if (feedback_interval_ms_ <= 0)
        {
            feedback_interval_ms_ = 400;
        }
    }

    void AgvNavServerNode::initialize_components()
    {
        LocalizationMonitor::Config loc_config;
        loc_config.particle_spread_threshold = 1.0;
        loc_config.required_stable_count = 5;
        loc_config.quality_threshold = 0.5;

        localization_monitor_ =
            std::make_shared<LocalizationMonitor>(static_cast<rclcpp::Node *>(this), loc_config);

        navigation_manager_ = std::make_shared<agv_bridge::NavigationManager>(
            static_cast<rclcpp::Node *>(this), this, localization_monitor_);

        if (!navigation_manager_->initialize())
        {
            RCLCPP_WARN(this->get_logger(),
                        "NavigationManager 初始化未完全成功（nav2 动作服务可能尚未起来）");
        }

        // ===== 地图管理：目录扫描 + 可选托管定位子进程 =====
        map_file_manager_ = std::make_unique<MapFileManager>(maps_dir_);
        RCLCPP_INFO(this->get_logger(), "地图目录: %s（机器人包: %s）",
                    map_file_manager_->mapsDir().c_str(), robot_package_.c_str());

        // Cartographer 管理请求可能在后台线程同步等待响应，必须使用独立的
        // Reentrant callback group，不能与 10Hz TF 查询定时器共用默认互斥组。
        cartographer_client_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // cartographer /write_state 客户端（save_map 保存 pbstream 用）
        write_state_client_ = this->create_client<cartographer_ros_msgs::srv::WriteState>(
            "/write_state", rmw_qos_profile_services_default, cartographer_client_group_);
        start_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>(
            "/start_trajectory", rmw_qos_profile_services_default, cartographer_client_group_);
        trajectory_states_client_ = this->create_client<cartographer_ros_msgs::srv::GetTrajectoryStates>(
            "/get_trajectory_states", rmw_qos_profile_services_default, cartographer_client_group_);

        if (pbstream_file_.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "未配置 pbstream_file：启动后无定位。可先 /agv/start_mapping 建图，"
                        "再 /agv/save_map 保存后自动进入定位（navigation launch 请传 "
                        "include_localization:=false，避免 cartographer 双开）");
        }
    }

    void AgvNavServerNode::create_interfaces()
    {
        // 注意：NavigationManager 内部使用相对名创建 nav2 客户端（follow_path / spin /
        // navigate_to_pose），因此本节点必须运行在根命名空间下。为保证上位机侧名字稳定，
        // 这里的接口一律使用显式绝对名。
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // ---- action: /agv/follow_edge ----
        rclcpp::Node *self = static_cast<rclcpp::Node *>(this);
        follow_edge_server_ = rclcpp_action::create_server<FollowEdge>(
            self,
            "/agv/follow_edge",
            std::bind(&AgvNavServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AgvNavServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&AgvNavServerNode::handle_accepted, this, std::placeholders::_1));

        // ---- service: /agv/set_control ----
        set_control_srv_ = this->create_service<SetControl>(
            "/agv/set_control",
            std::bind(&AgvNavServerNode::handle_set_control, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/get_map（导出栅格给上位机）----
        get_map_srv_ = this->create_service<GetMap>(
            "/agv/get_map",
            std::bind(&AgvNavServerNode::handle_get_map, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/load_map（重启定位加载新图）----
        load_map_srv_ = this->create_service<LoadMap>(
            "/agv/load_map",
            std::bind(&AgvNavServerNode::handle_load_map, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/list_maps（列出可导入地图）----
        list_maps_srv_ = this->create_service<ListMaps>(
            "/agv/list_maps",
            std::bind(&AgvNavServerNode::handle_list_maps, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/start_mapping（进入在线建图）----
        start_mapping_srv_ = this->create_service<StartMapping>(
            "/agv/start_mapping",
            std::bind(&AgvNavServerNode::handle_start_mapping, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/save_map（保存建图并回到定位）----
        save_map_srv_ = this->create_service<SaveMap>(
            "/agv/save_map",
            std::bind(&AgvNavServerNode::handle_save_map, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- service: /agv/relocalize（指定地图坐标重启 Cartographer 定位）----
        relocalize_srv_ = this->create_service<Relocalize>(
            "/agv/relocalize",
            std::bind(&AgvNavServerNode::handle_relocalize, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- topic: /agv/status (transient_local，新客户端一接入即可拿到最后一帧) ----
        rclcpp::QoS status_qos(1);
        status_qos.transient_local();
        status_pub_ = this->create_publisher<AgvStatus>("/agv/status", status_qos);

        // ---- topic: /agv/pose (map 系 PoseStamped，10Hz，上位机直读位姿专用) ----
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/agv/pose", 10);

        // ---- 可选：真实电池数据，无发布者时保持 battery_level 参数值 ----
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
            {
                if (std::isfinite(msg->percentage) && msg->percentage >= 0.0)
                {
                    // 标准约定：0.0 ~ 1.0；部分驱动直接给 0~100
                    battery_level_ = msg->percentage <= 1.0 ? msg->percentage * 100.0
                                                             : msg->percentage;
                    if (battery_level_ > 100.0) battery_level_ = 100.0;
                }
            });
    }

    void AgvNavServerNode::create_timers()
    {
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&AgvNavServerNode::publish_status, this));

        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(feedback_interval_ms_),
            std::bind(&AgvNavServerNode::publish_feedback, this));

        localization_timer_ = this->create_wall_timer(
            1s, std::bind(&AgvNavServerNode::update_localization_monitor, this));

        tf_timer_ = this->create_wall_timer(
            100ms, std::bind(&AgvNavServerNode::broadcast_tf, this));
    }

    // ============================ action 回调 ============================

    rclcpp_action::GoalResponse AgvNavServerNode::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowEdge::Goal> goal)
    {
        (void)uuid;

        // 不在这里拒绝「定位未初始化 / 启动失败」，而是先 ACCEPT 再 ABORT。
        // 这样上位机拿到的是一条带 message 的 action_result，等价于旧的 command_ack FAILED。
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (current_goal_handle_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "已有导航任务在执行 (command_id=%s)，拒绝新的 move_to (command_id=%s)",
                            active_command_id_.c_str(), goal->command_id.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (control_stopped_.load())
        {
            RCLCPP_WARN(this->get_logger(),
                        "AGV 处于 stop 状态，拒绝导航任务 (command_id=%s)", goal->command_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ != MODE_NAVIGATION)
            {
                RCLCPP_WARN(this->get_logger(),
                            "当前 mode=%s (map=%s)，拒绝导航任务 (command_id=%s)",
                            mode_.c_str(), map_name_.c_str(), goal->command_id.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "接受导航任务: command_id=%s, node_id=%s, edge=%s(%s), 目标(%.2f, %.2f, %.3f), "
                    "max_speed=%.2f, back_up=%d, reverse=%d, control_points=%zu",
                    goal->command_id.c_str(), goal->node_id.c_str(),
                    goal->edge_id.c_str(), goal->edge_type.c_str(),
                    goal->x, goal->y, goal->theta,
                    goal->max_speed, static_cast<int>(goal->back_up),
                    static_cast<int>(goal->reverse), goal->control_points.size());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AgvNavServerNode::handle_cancel(
        const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
    {
        cancel_requested_ = true;

        if (navigation_manager_ && navigation_manager_->isNavigating())
        {
            // 让 NavigationManager 去取消 nav2 的 follow_path / spin / navigate_to_pose，
            // 结果会通过 onNavigationResult 回到当前 goal，此处不做终结。
            RCLCPP_INFO(this->get_logger(), "收到取消请求，正在取消底层 nav2 动作...");
            navigation_manager_->stopNavigation();
        }
        else
        {
            // 底层没有在执行的动作，直接终结，避免上位机一直等 cancel 响应
            RCLCPP_INFO(this->get_logger(), "收到取消请求，但当前没有正在执行的导航，直接结束");

            auto result = std::make_shared<FollowEdge::Result>();
            result->success = false;
            result->message = "导航已取消";
            result->command_id = active_command_id_;
            result->node_id = active_node_id_;

            if (goal_handle && goal_handle->is_active())
            {
                goal_handle->canceled(result);
            }
            reset_goal();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AgvNavServerNode::handle_accepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            current_goal_handle_ = goal_handle;
        }
        cancel_requested_ = false;

        auto goal = goal_handle->get_goal();
        active_command_id_ = goal->command_id;
        active_node_id_ = goal->node_id;

        // 定位未收敛时不允许下发导航（与原 handle_move_command 的检查一致）
        if (!localization_monitor_->isInitialized())
        {
            abort_current_goal("无法获取机器人位姿（定位未初始化），拒绝执行导航命令");
            return;
        }

        MoveToMessage move_msg = to_move_to_message(*goal);

        bool started = false;
        if (move_msg.edgeInfo.isBezierCurve())
        {
            RCLCPP_INFO(this->get_logger(), "贝塞尔曲线阶数: %d, 控制点数量: %zu",
                        move_msg.edgeInfo.getBezierOrder(),
                        move_msg.edgeInfo.controlPoints.size());
            started = navigation_manager_->followBezierPathNavigation(move_msg);
        }
        else
        {
            started = navigation_manager_->followPathNavigation(move_msg);
        }

        if (!started)
        {
            abort_current_goal("启动导航失败（路径生成失败或位姿不可用）");
        }
    }

    // ============================ service 回调 ============================

    void AgvNavServerNode::handle_set_control(
        std::shared_ptr<SetControl::Request> request,
        std::shared_ptr<SetControl::Response> response)
    {
        const std::string &action = request->action;

        if (action == "start")
        {
            control_stopped_ = false;
            response->success = true;
            response->message = "已恢复任务接收";
        }
        else if (action == "stop")
        {
            control_stopped_ = true;
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
            response->success = true;
            response->message = "已停止当前导航并暂停接收任务";
        }
        else if (action == "reset")
        {
            control_stopped_ = false;
            if (navigation_manager_ && navigation_manager_->isNavigating())
            {
                navigation_manager_->stopNavigation();
            }
            response->success = true;
            response->message = "已复位";
        }
        else
        {
            response->success = false;
            response->message = "未知 action: " + action + "（仅支持 start | stop | reset）";
        }

        response->state = control_stopped_.load() ? "STOPPED" : agv_state_;

        RCLCPP_INFO(this->get_logger(), "set_control(%s) -> success=%d, state=%s",
                    action.c_str(), static_cast<int>(response->success), response->state.c_str());
    }

    void AgvNavServerNode::handle_get_map(
        std::shared_ptr<GetMap::Request> request,
        std::shared_ptr<GetMap::Response> response)
    {
        std::string name = request->map_name;
        if (name.empty())
        {
            // 未指定时取当前定位地图
            std::lock_guard<std::mutex> lock(mode_mutex_);
            name = map_name_;
        }
        if (name.empty())
        {
            response->success = false;
            response->message = "未指定 map_name，且当前无已加载地图";
            return;
        }
        if (!map_file_manager_)
        {
            response->success = false;
            response->message = "地图管理未初始化（maps_dir 参数无效）";
            return;
        }

        std::string error;
        if (!map_file_manager_->loadGrid(name, response->map, error))
        {
            response->success = false;
            response->message = error;
            return;
        }

        response->map.header.stamp = this->now();
        response->map_name = name;
        response->success = true;
        response->message = "ok";

        RCLCPP_INFO(this->get_logger(),
                    "get_map(%s) -> %ux%u @ %.3fm/px, data=%zu",
                    name.c_str(), response->map.info.width, response->map.info.height,
                    response->map.info.resolution, response->map.data.size());
    }

    void AgvNavServerNode::handle_list_maps(
        std::shared_ptr<ListMaps::Request> /*request*/,
        std::shared_ptr<ListMaps::Response> response)
    {
        if (!map_file_manager_)
        {
            response->success = false;
            response->message = "地图管理未初始化（maps_dir 参数无效）";
            return;
        }

        std::string error;
        const auto entries = map_file_manager_->listMaps(&error);
        if (!error.empty())
        {
            response->success = false;
            response->message = error;
            return;
        }

        for (const auto &entry : entries)
        {
            response->map_names.push_back(entry.name);
        }
        response->success = true;
        response->message = "ok";

        RCLCPP_INFO(this->get_logger(), "list_maps -> %zu 张 (pgm+yaml 齐全)",
                    response->map_names.size());
    }

    void AgvNavServerNode::handle_load_map(
        std::shared_ptr<LoadMap::Request> request,
        std::shared_ptr<LoadMap::Response> response)
    {
        const std::string &name = request->map_name;

        if (!MapFileManager::isValidMapName(name))
        {
            response->success = false;
            response->message = "地图名非法（非空且不含路径分隔符）: '" + name + "'";
            return;
        }
        if (!map_file_manager_ || !map_file_manager_->hasPbstream(name))
        {
            response->success = false;
            response->message = "地图目录下不存在 " + name +
                                ".pbstream（load_map 需要 pbstream，可用 /agv/list_maps 查看）";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ == MODE_RELOCALIZING)
            {
                response->success = false;
                response->message = "正在切换地图 (map=" + map_name_ +
                                    ")，请等待 /agv/status.mode 回到 NAVIGATION";
                return;
            }
            if (mode_ == MODE_MAPPING)
            {
                response->success = false;
                response->message = "正在建图 (MAPPING)，请先 /agv/save_map 保存后再切换地图";
                return;
            }
        }

        {
            // 定位子进程必须由本节点托管，否则无法停掉外部启动的 cartographer
            std::lock_guard<std::mutex> lock(proc_mutex_);
            if (localization_pid_ <= 0)
            {
                response->success = false;
                response->message =
                    "定位节点不由 agv_nav_server 托管，无法切换地图。"
                    "请以 pbstream_file:=<abs> 参数启动本节点（同时 navigation launch 传 "
                    "include_localization:=false），由本节点拉起定位";
                return;
            }
        }

        if (navigation_manager_ && navigation_manager_->isNavigating())
        {
            response->success = false;
            response->message = "正在执行导航任务，请先取消或等待完成后再切换地图";
            return;
        }

        if (transition_in_progress_.exchange(true))
        {
            response->success = false;
            response->message = "上一次模式切换仍在进行中";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode_ = MODE_RELOCALIZING;
            map_name_ = name;
        }

        const std::string pbstream_abs = map_file_manager_->pbstreamPath(name);
        if (transition_thread_.joinable())
        {
            transition_thread_.join();
        }
        transition_thread_ = std::thread([this, pbstream_abs, name]()
        {
            RCLCPP_INFO(this->get_logger(), "load_map(%s)：停止旧定位进程...", name.c_str());
            stop_child_process(localization_pid_, "定位");

            geometry_msgs::msg::Pose remembered_pose;
            const bool has_pose = load_remembered_pose(name, remembered_pose);
            std::string error;
            RCLCPP_INFO(this->get_logger(), "load_map(%s)：以%s初始位姿重启定位...",
                        name.c_str(), has_pose ? "记忆的" : "全局搜索");
            if (!start_managed_localization(
                    pbstream_abs, name, has_pose ? &remembered_pose : nullptr, error))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "load_map(%s)：定位进程重启失败: %s", name.c_str(), error.c_str());
                std::lock_guard<std::mutex> lock(mode_mutex_);
                mode_ = MODE_NAVIGATION;
            }
            transition_in_progress_.store(false);
            // 收敛后由 update_localization_monitor 把 RELOCALIZING 切回 NAVIGATION
        });

        response->success = true;
        response->map_name = name;
        response->message = "定位重启中，等待 /agv/status.mode: RELOCALIZING -> NAVIGATION";

        RCLCPP_INFO(this->get_logger(), "load_map(%s) -> 已发起定位重启", name.c_str());
    }

    void AgvNavServerNode::handle_relocalize(
        std::shared_ptr<Relocalize::Request> request,
        std::shared_ptr<Relocalize::Response> response)
    {
        const std::string name = request->map_name;
        if (!MapFileManager::isValidMapName(name) ||
            !std::isfinite(request->x) || !std::isfinite(request->y) ||
            !std::isfinite(request->yaw))
        {
            response->success = false;
            response->message = "地图名或 x/y/yaw 非法";
            return;
        }
        if (!map_file_manager_ || !map_file_manager_->hasPbstream(name))
        {
            response->success = false;
            response->message = "地图不存在或缺少 pbstream: " + name;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ == MODE_MAPPING)
            {
                response->success = false;
                response->message = "正在建图，请先保存地图";
                return;
            }
        }
        if (navigation_manager_ && navigation_manager_->isNavigating())
        {
            response->success = false;
            response->message = "正在执行导航任务，请先取消后再重定位";
            return;
        }
        if (transition_in_progress_.exchange(true))
        {
            response->success = false;
            response->message = "上一次模式切换仍在进行中";
            return;
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = request->x;
        pose.position.y = request->y;
        pose.position.z = 0.0;
        pose.orientation = TransformUtils::yaw_to_quaternion(request->yaw);
        const std::string pbstream_abs = map_file_manager_->pbstreamPath(name);

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            map_name_ = name;
            mode_ = MODE_RELOCALIZING;
        }
        if (transition_thread_.joinable()) transition_thread_.join();
        transition_thread_ = std::thread([this, name, pbstream_abs, pose]()
        {
            stop_child_process(localization_pid_, "定位");
            std::string error;
            if (!start_managed_localization(pbstream_abs, name, &pose, error))
            {
                RCLCPP_ERROR(this->get_logger(), "relocalize(%s) 失败: %s",
                             name.c_str(), error.c_str());
                std::lock_guard<std::mutex> lock(mode_mutex_);
                mode_ = MODE_NAVIGATION;
            }
            else if (!save_remembered_pose(name, pose, error))
            {
                RCLCPP_WARN(this->get_logger(), "定位已启动，但保存位置记忆失败: %s", error.c_str());
            }
            transition_in_progress_.store(false);
        });

        response->success = true;
        response->map_name = name;
        response->message = "已接受重定位，等待 /agv/status.mode 从 RELOCALIZING 变为 NAVIGATION";
    }

    void AgvNavServerNode::handle_start_mapping(
        std::shared_ptr<StartMapping::Request> /*request*/,
        std::shared_ptr<StartMapping::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ == MODE_RELOCALIZING)
            {
                response->success = false;
                response->message = "正在切换地图，请等待 mode 回到 NAVIGATION 后再建图";
                return;
            }
            if (mode_ == MODE_MAPPING)
            {
                response->success = false;
                response->message = "已在建图中 (MAPPING)，遥控探索完成后请调用 /agv/save_map";
                return;
            }
        }

        if (navigation_manager_ && navigation_manager_->isNavigating())
        {
            response->success = false;
            response->message = "正在执行导航任务，请先取消或等待完成后再开始建图";
            return;
        }

        if (transition_in_progress_.exchange(true))
        {
            response->success = false;
            response->message = "上一次模式切换仍在进行中";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            if (localization_pid_ <= 0)
            {
                // 没有托管定位：允许直接建图，但外部若还跑着 cartographer 会 TF 双发，
                // 这里只能提示（navigation launch 必须 include_localization:=false）
                RCLCPP_WARN(this->get_logger(),
                            "本节点未托管定位进程：请确认外部没有 cartographer 在跑"
                            "（navigation launch 需 include_localization:=false）");
            }
        }

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode_ = MODE_MAPPING;
            map_name_.clear();
        }

        if (transition_thread_.joinable())
        {
            transition_thread_.join();
        }
        transition_thread_ = std::thread([this]()
        {
            RCLCPP_INFO(this->get_logger(), "start_mapping：停止托管定位进程...");
            stop_child_process(localization_pid_, "定位");

            RCLCPP_INFO(this->get_logger(), "start_mapping：拉起建图进程...");
            if (!spawn_ros_launch(
                    slam_launch_package_, slam_launch_file_,
                    {useSimTimeArg(use_sim_time_)},
                    slam_pid_, "/tmp/agv_slam.log"))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "start_mapping：建图进程启动失败，请检查 %s / %s 与日志 /tmp/agv_slam.log",
                             slam_launch_package_.c_str(), slam_launch_file_.c_str());
                std::lock_guard<std::mutex> lock(mode_mutex_);
                mode_ = MODE_NAVIGATION;
            }
            transition_in_progress_.store(false);
        });

        response->success = true;
        response->message = "建图已启动（mode=MAPPING）：导航任务被拒绝，/cmd_vel 遥控可用；"
                            "完成后调用 /agv/save_map 保存并回到定位";

        RCLCPP_INFO(this->get_logger(), "start_mapping -> 已发起建图");
    }

    void AgvNavServerNode::handle_save_map(
        std::shared_ptr<SaveMap::Request> request,
        std::shared_ptr<SaveMap::Response> response)
    {
        const std::string &name = request->map_name;

        if (!MapFileManager::isValidMapName(name))
        {
            response->success = false;
            response->message = "地图名非法（非空且不含路径分隔符）: '" + name + "'";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ != MODE_MAPPING)
            {
                response->success = false;
                response->message = "当前 mode=" + mode_ + "，仅 MAPPING 模式下可保存地图（先 /agv/start_mapping）";
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            if (slam_pid_ <= 0)
            {
                response->success = false;
                response->message = "建图进程不由本节点托管，无法保存";
                return;
            }
        }

        if (transition_in_progress_.exchange(true))
        {
            response->success = false;
            response->message = "上一次模式切换仍在进行中";
            return;
        }

        std::string directory_error;
        if (!map_file_manager_->ensureMapDirectory(name, directory_error))
        {
            transition_in_progress_.store(false);
            response->success = false;
            response->message = directory_error;
            return;
        }

        // 新地图统一保存到 maps/<name>/<name>.*，不再平铺到 maps 根目录。
        const std::string stem = map_file_manager_->mapStem(name);
        const std::string pbstream_abs = stem + ".pbstream";
        // 建图结束时 map→base_link 已知；保存这份位姿并用它启动新图定位，
        // 避免新轨迹错误地从地图原点开始。
        const auto mapping_pose = localization_monitor_ ?
            localization_monitor_->getCurrentPose() : std::nullopt;

        if (transition_thread_.joinable())
        {
            transition_thread_.join();
        }
        transition_thread_ = std::thread([this, name, pbstream_abs, stem, mapping_pose]()
        {
            // 1. 保存 pbstream（cartographer /write_state）
            std::string error;
            RCLCPP_INFO(this->get_logger(), "save_map(%s)：写入 %s ...", name.c_str(), pbstream_abs.c_str());
            if (!call_write_state(pbstream_abs, error))
            {
                RCLCPP_ERROR(this->get_logger(), "save_map(%s)：保存 pbstream 失败: %s（mode 保持 MAPPING，可重试）",
                             name.c_str(), error.c_str());
                transition_in_progress_.store(false);
                return;
            }

            // 2. pgm + yaml（三件套齐，get_map/list_maps 才能识别）
            //    改用 nav2_map_server 的 map_saver 从 /map 话题直接保存：
            //    cartographer_pbstream_to_ros_map 在本机因 cairo 兼容问题必崩
            //    （image.cc:55 Check failed: cairo_image_surface_get_format (-1 vs 0)，
            //     实测 2MB 正常 2D pbstream 也崩），而 occupancy_grid_node 实时
            //    投影的 /map 数据完好——截图即所得。必须在停 slam 之前执行。
            RCLCPP_INFO(this->get_logger(), "save_map(%s)：从 /map 保存 pgm/yaml ...", name.c_str());
            if (!run_command_sync(
                    {"ros2", "run", "nav2_map_server", "map_saver_cli",
                     "-f", stem, "--occ", "0.65", "--free", "0.25", "--fmt", "pgm"},
                    "/tmp/agv_map_export.log"))
            {
                // 常见诱因：刚开建图就保存/放弃（无完整子图），pbstream_to_ros_map
                // 对零尺寸画布在 cairo 里直接 abort（exit 250），重试永远失败。
                // 此时不能停在 MAPPING 卡死：停建图，回退进入建图前的地图恢复定位。
                RCLCPP_ERROR(this->get_logger(),
                             "save_map(%s)：pbstream 转 pgm/yaml 失败（地图内容为空或转换器崩溃，"
                             "详见 /tmp/agv_map_export.log），停建图并回退之前的地图", name.c_str());
                stop_child_process(slam_pid_, "建图");

                std::string prev_map;
                {
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    prev_map = last_nav_map_name_;
                    map_name_ = prev_map;
                }
                bool restored = false;
                // start_managed_localization 内部含最长 300s 的等待，期间不能持有
                // mode_mutex_（会卡住 1Hz 的 /agv/status 发布），先判断后调用。
                if (!prev_map.empty() && prev_map != name)
                {
                    const std::string prev_pb = map_file_manager_->mapStem(prev_map) + ".pbstream";
                    if (::access(prev_pb.c_str(), F_OK) == 0)
                    {
                        std::string err;
                        restored = start_managed_localization(prev_pb, prev_map, nullptr, err);
                        if (!restored)
                        {
                            RCLCPP_ERROR(this->get_logger(),
                                         "save_map：回退旧图 %s 定位失败: %s", prev_map.c_str(), err.c_str());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(),
                                    "save_map：旧图 %s 的 pbstream 不存在，仅停止建图", prev_map.c_str());
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    mode_ = restored ? MODE_RELOCALIZING : MODE_NAVIGATION;
                }
                if (restored)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "save_map：已放弃本次建图，回退到地图 %s", prev_map.c_str());
                }
                transition_in_progress_.store(false);
                return;
            }

            // 3. 停建图，用新图拉起定位
            RCLCPP_INFO(this->get_logger(), "save_map(%s)：停止建图进程，拉起定位...", name.c_str());
            stop_child_process(slam_pid_, "建图");

            {
                std::lock_guard<std::mutex> lock(mode_mutex_);
                map_name_ = name;
                mode_ = MODE_RELOCALIZING;
            }
            const geometry_msgs::msg::Pose *initial_pose =
                mapping_pose.has_value() ? &mapping_pose->pose : nullptr;
            if (!start_managed_localization(pbstream_abs, name, initial_pose, error))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "save_map(%s)：定位进程启动失败: %s", name.c_str(), error.c_str());
                std::lock_guard<std::mutex> lock(mode_mutex_);
                mode_ = MODE_NAVIGATION;
            }
            else if (initial_pose && !save_remembered_pose(name, *initial_pose, error))
            {
                RCLCPP_WARN(this->get_logger(), "新图定位已启动，但保存初始位姿失败: %s", error.c_str());
            }
            transition_in_progress_.store(false);
            // 收敛后由 update_localization_monitor 把 RELOCALIZING 切回 NAVIGATION
        });

        response->success = true;
        response->map_name = name;
        response->message = "保存与定位重启已发起：保存/转换失败时 mode 保持 MAPPING，"
                            "成功则 RELOCALIZING -> NAVIGATION";

        RCLCPP_INFO(this->get_logger(), "save_map(%s) -> 已发起", name.c_str());
    }

    // ============================ 定时器回调 ============================

    void AgvNavServerNode::publish_status()
    {
        AgvStatus msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.agv_id = agv_id_;
        msg.state = control_stopped_.load() ? "STOPPED" : agv_state_;
        msg.battery = battery_level_;
        msg.pose_initialized = localization_monitor_ ? localization_monitor_->isInitialized() : false;

        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            msg.mode = mode_;
            msg.map_name = map_name_;
            // 记住最近一次稳定导航时的地图名，供 save_map 导出失败回退
            if (mode_ == MODE_NAVIGATION && !map_name_.empty())
            {
                last_nav_map_name_ = map_name_;
            }
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            msg.active_command_id = active_command_id_;
            msg.active_node_id = active_node_id_;
        }

        status_pub_->publish(msg);
    }

    void AgvNavServerNode::publish_feedback()
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
        }

        if (!goal_handle || !goal_handle->is_active())
        {
            return;
        }

        auto pose_opt = localization_monitor_->getCurrentPose();
        if (!pose_opt.has_value())
        {
            return;
        }

        const auto &pose = pose_opt.value().pose;

        auto feedback = std::make_shared<FollowEdge::Feedback>();
        feedback->x = pose.position.x;
        feedback->y = pose.position.y;
        feedback->theta = TransformUtils::quaternion_to_yaw(pose.orientation);
        feedback->state = control_stopped_.load() ? "STOPPED" : agv_state_;

        goal_handle->publish_feedback(feedback);
    }

    void AgvNavServerNode::update_localization_monitor()
    {
        if (!localization_monitor_)
        {
            return;
        }

        localization_monitor_->update();

        static bool last_initialized_state = false;
        const bool current = localization_monitor_->isInitialized();

        if (current != last_initialized_state)
        {
            if (current)
            {
                RCLCPP_INFO(this->get_logger(), "状态变化：位置初始化完成 ✅");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "状态变化：位置未初始化 ❌");
            }
            last_initialized_state = current;
        }

        // 切图重启定位收敛后（TF 恢复新鲜），RELOCALIZING -> NAVIGATION
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            if (mode_ == MODE_RELOCALIZING && current && !transition_in_progress_.load())
            {
                mode_ = MODE_NAVIGATION;
                RCLCPP_INFO(this->get_logger(), "地图 %s 定位已收敛，mode -> NAVIGATION",
                            map_name_.c_str());
            }
        }

        // 定位稳定后每 5 秒刷新当前地图的位置记忆。异常断电最多丢失约 5 秒，
        // 下次启动或 load_map 时可直接从最近位姿开始匹配。
        if (current)
        {
            std::string current_map;
            bool navigation_mode = false;
            {
                std::lock_guard<std::mutex> lock(mode_mutex_);
                current_map = map_name_;
                navigation_mode = mode_ == MODE_NAVIGATION;
            }
            const auto now = std::chrono::steady_clock::now();
            if (navigation_mode && !current_map.empty() &&
                (last_pose_save_time_.time_since_epoch().count() == 0 ||
                 now - last_pose_save_time_ >= std::chrono::seconds(5)))
            {
                const auto pose = localization_monitor_->getCurrentPose();
                if (pose.has_value())
                {
                    std::string error;
                    if (!save_remembered_pose(current_map, pose->pose, error))
                    {
                        RCLCPP_WARN(this->get_logger(), "更新位置记忆失败: %s", error.c_str());
                    }
                    else
                    {
                        last_pose_save_time_ = now;
                    }
                }
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                             "%s",
                             current ? "✅ 定位已就绪，可接受导航任务" : "❌ 定位未收敛，等待 Cartographer/AMCL");
    }

    void AgvNavServerNode::broadcast_tf()
    {
        if (!enable_tf_broadcast_)
        {
            return;
        }

        auto pose_opt = localization_monitor_->getCurrentPose();
        if (!pose_opt.has_value())
        {
            return;
        }

        const auto &pose = pose_opt.value().pose;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = agv_id_ + "/base_link";
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        // 同一 10Hz 节拍同步发布专用位姿话题（与 TF 内容一致，上位机直读不受 /tf 混帧影响）
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = transform.header;
        pose_stamped.pose = pose;
        pose_pub_->publish(pose_stamped);
    }

    // ============================ NavigationCallbacks 实现 ============================

    void AgvNavServerNode::onNavigationStateChanged(agv_bridge::NavigationState state,
                                                    const std::string &command_id)
    {
        (void)command_id;
        agv_state_ = navigationStateToString(state);
        RCLCPP_INFO(this->get_logger(), "导航状态 -> %s", agv_state_.c_str());
    }

    void AgvNavServerNode::onNavigationFeedback(double x, double y, double theta)
    {
        // 反馈统一由 publish_feedback() 定时器按 feedback_interval_ms 发布，
        // 这里只保留调试日志，避免上游回调频率过高把 WS 打爆。
        RCLCPP_DEBUG(this->get_logger(), "导航反馈: (%.2f, %.2f, %.2f)", x, y, theta);
    }

    void AgvNavServerNode::onNavigationResult(bool success,
                                              const std::string &message,
                                              const std::string &command_id,
                                              const std::string &node_id)
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
            current_goal_handle_.reset();
        }

        RCLCPP_INFO(this->get_logger(), "导航结果: %s, command_id=%s, node_id=%s, message=%s",
                    success ? "SUCCESS" : "FAILED", command_id.c_str(), node_id.c_str(), message.c_str());

        if (!goal_handle || !goal_handle->is_active())
        {
            return;
        }

        auto result = std::make_shared<FollowEdge::Result>();
        result->success = success;
        result->message = message;
        result->command_id = command_id;
        result->node_id = node_id;

        if (goal_handle->is_canceling() || cancel_requested_)
        {
            goal_handle->canceled(result);
        }
        else if (success)
        {
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
        }

        // 任务结束后清空，允许接收下一条指令
        active_command_id_.clear();
        active_node_id_.clear();
        cancel_requested_ = false;
    }

    void AgvNavServerNode::onCommandAck(const std::string &command_id,
                                        const std::string &node_id,
                                        const std::string &status,
                                        const std::string &message)
    {
        // 终态（SUCCESS/FAILED/CANCELED/ABORTED）都会紧跟一次 onNavigationResult，
        // 由那里统一回填 action result，这里只做日志与状态跟踪。
        RCLCPP_INFO(this->get_logger(), "command_ack: command_id=%s, node_id=%s, status=%s, message=%s",
                    command_id.c_str(), node_id.c_str(), status.c_str(), message.c_str());
    }

    // ============================ 工具 ============================

    MoveToMessage AgvNavServerNode::to_move_to_message(const FollowEdge::Goal &goal)
    {
        MoveToMessage msg;
        msg.commandId = goal.command_id;
        msg.nodeId = goal.node_id;
        msg.x = goal.x;
        msg.y = goal.y;
        msg.theta = goal.theta;
        msg.endPoint = goal.end_point;

        msg.edgeInfo.id = goal.edge_id;
        msg.edgeInfo.sourceId = goal.source_id;
        msg.edgeInfo.targetId = goal.target_id;
        msg.edgeInfo.maxSpeed = goal.max_speed > 0.0 ? goal.max_speed : 1.0;
        msg.edgeInfo.step = goal.step > 0.0 ? goal.step : 0.1;
        msg.edgeInfo.backUp = goal.back_up;
        msg.edgeInfo.reverse = goal.reverse;

        for (const auto &cp : goal.control_points)
        {
            msg.edgeInfo.controlPoints.emplace_back(cp.x, cp.y);
        }

        // edge_type 允许上位机省略：有控制点即为曲线，否则视为直线
        if (goal.edge_type == "CURVE")
        {
            msg.edgeInfo.type = EdgeType::CURVE;
        }
        else if (goal.edge_type == "ELEVATION")
        {
            msg.edgeInfo.type = EdgeType::ELEVATION;
        }
        else if (goal.edge_type == "STRAIGHT")
        {
            msg.edgeInfo.type = EdgeType::STRAIGHT;
        }
        else
        {
            msg.edgeInfo.type = goal.control_points.empty() ? EdgeType::STRAIGHT : EdgeType::CURVE;
        }

        // 与原 handle_move_command 一致：目标节点是边的起点则反向行驶
        if (!msg.nodeId.empty())
        {
            if (!msg.edgeInfo.sourceId.empty() && msg.nodeId == msg.edgeInfo.sourceId)
            {
                msg.edgeInfo.reverse = true;
            }
            else if (!msg.edgeInfo.targetId.empty() && msg.nodeId == msg.edgeInfo.targetId)
            {
                msg.edgeInfo.reverse = false;
            }
        }

        return msg;
    }

    void AgvNavServerNode::abort_current_goal(const std::string &message)
    {
        std::shared_ptr<GoalHandleFollowEdge> goal_handle;
        std::string command_id;
        std::string node_id;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_handle = current_goal_handle_;
            current_goal_handle_.reset();
            command_id = active_command_id_;
            node_id = active_node_id_;
            active_command_id_.clear();
            active_node_id_.clear();
        }

        RCLCPP_WARN(this->get_logger(), "导航任务终止: %s", message.c_str());

        if (goal_handle && goal_handle->is_active())
        {
            auto result = std::make_shared<FollowEdge::Result>();
            result->success = false;
            result->message = message;
            result->command_id = command_id;
            result->node_id = node_id;
            goal_handle->abort(result);
        }

        cancel_requested_ = false;
    }

    void AgvNavServerNode::reset_goal()
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_.reset();
        active_command_id_.clear();
        active_node_id_.clear();
    }

    // ============================ 地图 / 定位 / 建图进程管理 ============================

    std::string AgvNavServerNode::absolute_path(const std::string &path)
    {
        if (path.empty() || path[0] == '/')
        {
            return path;
        }
        char *cwd = ::getcwd(nullptr, 0);
        if (cwd == nullptr)
        {
            return path;
        }
        std::string abs = std::string(cwd) + (cwd[std::strlen(cwd) - 1] == '/' ? "" : "/") + path;
        ::free(cwd);
        return abs;
    }

    std::string AgvNavServerNode::file_stem(const std::string &path)
    {
        const auto slash = path.rfind('/');
        const std::string filename = slash == std::string::npos ? path : path.substr(slash + 1);
        const auto dot = filename.rfind('.');
        return dot == std::string::npos ? filename : filename.substr(0, dot);
    }

    bool AgvNavServerNode::spawn_ros_launch(const std::string &package, const std::string &launch_file,
                                            const std::vector<std::string> &extra_args,
                                            pid_t &pid_slot, const char *log_path)
    {
        std::vector<std::string> argv_str = {"ros2", "launch", package, launch_file};
        argv_str.insert(argv_str.end(), extra_args.begin(), extra_args.end());

        const pid_t pid = ::fork();
        if (pid < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "fork 子进程失败: %s", std::strerror(errno));
            return false;
        }
        if (pid == 0)
        {
            // 子进程：独立进程组（便于整组终止），输出重定向到日志文件
            ::setpgid(0, 0);
            const int fd = ::open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd >= 0)
            {
                ::dup2(fd, STDOUT_FILENO);
                ::dup2(fd, STDERR_FILENO);
                if (fd > STDERR_FILENO)
                {
                    ::close(fd);
                }
            }
            ::signal(SIGINT, SIG_DFL);
            ::signal(SIGTERM, SIG_DFL);

            std::vector<char *> argv;
            argv.reserve(argv_str.size() + 1);
            for (const auto &arg : argv_str)
            {
                argv.push_back(const_cast<char *>(arg.c_str()));
            }
            argv.push_back(nullptr);
            ::execvp(argv[0], argv.data());
            ::_exit(127); // execvp 仅在失败时返回
        }
        // 父子两侧都 setpgid，避免竞态
        ::setpgid(pid, pid);
        {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            pid_slot = pid;
        }

        std::string cmdline;
        for (const auto &arg : argv_str)
        {
            cmdline += arg + " ";
        }
        RCLCPP_INFO(this->get_logger(), "子进程已启动 pid=%d: %s（日志: %s）",
                    pid, cmdline.c_str(), log_path);
        return true;
    }

    void AgvNavServerNode::stop_child_process(pid_t &pid_slot, const char *what)
    {
        pid_t pid = -1;
        {
            std::lock_guard<std::mutex> lock(proc_mutex_);
            pid = pid_slot;
            pid_slot = -1;
        }
        if (pid <= 0)
        {
            return;
        }

        // 先 SIGINT（ros2 launch 会优雅停掉 cartographer 等子节点），超时整组 SIGKILL
        if (::kill(-pid, SIGINT) != 0)
        {
            ::kill(pid, SIGINT);
        }
        for (int i = 0; i < 20; ++i)
        {
            if (::waitpid(pid, nullptr, WNOHANG) == pid)
            {
                RCLCPP_INFO(this->get_logger(), "%s子进程 pid=%d 已退出", what, pid);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_WARN(this->get_logger(), "%s子进程 pid=%d 未在 2s 内退出，SIGKILL 整个进程组", what, pid);
        ::kill(-pid, SIGKILL);
        ::waitpid(pid, nullptr, 0);
    }

    bool AgvNavServerNode::run_command_sync(const std::vector<std::string> &args, const char *log_path)
    {
        const pid_t pid = ::fork();
        if (pid < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "fork 失败: %s", std::strerror(errno));
            return false;
        }
        if (pid == 0)
        {
            ::setpgid(0, 0);
            const int fd = ::open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd >= 0)
            {
                ::dup2(fd, STDOUT_FILENO);
                ::dup2(fd, STDERR_FILENO);
                if (fd > STDERR_FILENO)
                {
                    ::close(fd);
                }
            }
            std::vector<char *> argv;
            argv.reserve(args.size() + 1);
            for (const auto &arg : args)
            {
                argv.push_back(const_cast<char *>(arg.c_str()));
            }
            argv.push_back(nullptr);
            ::execvp(argv[0], argv.data());
            ::_exit(127);
        }

        int status = 0;
        ::waitpid(pid, &status, 0);
        const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        if (!ok)
        {
            const int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            std::string cmdline;
            for (const auto &arg : args)
            {
                cmdline += arg + " ";
            }
            RCLCPP_ERROR(this->get_logger(), "命令执行失败(exit=%d): %s（详见 %s）",
                         exit_code, cmdline.c_str(), log_path);
        }
        return ok;
    }

    bool AgvNavServerNode::start_managed_localization(
        const std::string &pbstream_abs_path,
        const std::string &map_name,
        const geometry_msgs::msg::Pose *initial_pose,
        std::string &error)
    {
        if (localization_monitor_) localization_monitor_->reset();
        if (!spawn_ros_launch(
                localization_launch_package_, localization_launch_file_,
                {"pbstream_file:=" + pbstream_abs_path,
                 "start_trajectory_with_default_topics:=false",
                 useSimTimeArg(use_sim_time_)},
                localization_pid_, "/tmp/agv_localization.log"))
        {
            error = "无法拉起定位 launch（详见 /tmp/agv_localization.log）";
            return false;
        }

        if (!start_trajectory_client_->wait_for_service(std::chrono::seconds(30)))
        {
            error = "Cartographer /start_trajectory 服务 30 秒内未就绪";
            stop_child_process(localization_pid_, "定位");
            return false;
        }

        // launch 必须使用单个 gflags 参数
        // "-start_trajectory_with_default_topics=false"。此时加载 pbstream 后只能有
        // FROZEN/FINISHED 轨迹，不能已有 ACTIVE 轨迹；否则新轨迹会争用传感器 topics。
        int frozen_trajectory_id = -1;
        std::vector<int> active_trajectory_ids;
        if (!trajectory_states_client_->wait_for_service(std::chrono::seconds(30)))
        {
            error = "Cartographer /get_trajectory_states 服务不可用";
            stop_child_process(localization_pid_, "定位");
            return false;
        }
        auto states_future = trajectory_states_client_->async_send_request(
            std::make_shared<cartographer_ros_msgs::srv::GetTrajectoryStates::Request>());
        // 服务名称会在 pbstream 完全加载前就出现。大地图加载期间 Cartographer
        // 暂时不能处理请求，因此这里不能使用普通 service 的 5 秒短超时。
        // 实测 arm64 上大地图 load_state 全局优化可超过 150s（2026-09-26 复测
        // 01:35 会话正好 150s 被杀），进一步放宽到 300s；上位机任务截止需 >= 360s。
        if (states_future.wait_for(std::chrono::seconds(300)) != std::future_status::ready)
        {
            error = "读取轨迹状态超时（300 秒，pbstream 可能仍在加载或 Cartographer 已异常）";
            stop_child_process(localization_pid_, "定位");
            return false;
        }
        const auto states = states_future.get()->trajectory_states;
        for (size_t i = 0; i < states.trajectory_id.size() && i < states.trajectory_state.size(); ++i)
        {
            if (states.trajectory_state[i] ==
                cartographer_ros_msgs::msg::TrajectoryStates::FROZEN)
            {
                // 多轨迹 pbstream 优先使用最后一条冻结轨迹作为相对位姿参照。
                frozen_trajectory_id = std::max(frozen_trajectory_id, states.trajectory_id[i]);
            }
            else if (states.trajectory_state[i] ==
                     cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE)
            {
                active_trajectory_ids.push_back(states.trajectory_id[i]);
            }
        }

        if (!active_trajectory_ids.empty())
        {
            error = "Cartographer 加载地图后意外存在 ACTIVE 轨迹；请确认 localization.launch.py "
                    "传入的是单个参数 -start_trajectory_with_default_topics=false，且同一 ROS domain "
                    "没有第二个 cartographer_node。为避免破坏轨迹，不自动 finish";
            stop_child_process(localization_pid_, "定位");
            return false;
        }

        if (initial_pose && frozen_trajectory_id < 0)
        {
            error = "pbstream 中没有可作为初始位姿参照的冻结轨迹";
            stop_child_process(localization_pid_, "定位");
            return false;
        }

        auto request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
        try
        {
            request->configuration_directory =
                ament_index_cpp::get_package_share_directory(localization_launch_package_) + "/config";
        }
        catch (const std::exception &e)
        {
            error = std::string("找不到定位配置包: ") + e.what();
            stop_child_process(localization_pid_, "定位");
            return false;
        }
        request->configuration_basename = localization_configuration_basename_;
        request->use_initial_pose = initial_pose != nullptr;
        if (initial_pose)
        {
            request->initial_pose = *initial_pose;
            request->relative_to_trajectory_id = frozen_trajectory_id;
        }

        auto future = start_trajectory_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready)
        {
            error = "/start_trajectory 调用超时（60 秒）";
            stop_child_process(localization_pid_, "定位");
            return false;
        }
        const auto result = future.get();
        if (result->status.code != 0)
        {
            error = "/start_trajectory 被拒绝: " + result->status.message;
            stop_child_process(localization_pid_, "定位");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "地图 %s 定位轨迹已启动%s",
                    map_name.c_str(), initial_pose ? "（使用指定初始位姿）" : "（无初始位姿，全局搜索）");
        return true;
    }

    bool AgvNavServerNode::load_remembered_pose(
        const std::string &map_name, geometry_msgs::msg::Pose &pose) const
    {
        const std::string path = map_file_manager_->mapDirectory(map_name) + "/last_pose.yaml";
        std::ifstream in(path);
        if (!in) return false;

        

        double x = 0.0, y = 0.0, yaw = 0.0;
        bool have_x = false, have_y = false, have_yaw = false;
        std::string key;
        while (in >> key)
        {
            double value = 0.0;
            if (!(in >> value)) return false;
            if (key == "x:") { x = value; have_x = true; }
            else if (key == "y:") { y = value; have_y = true; }
            else if (key == "yaw:") { yaw = value; have_yaw = true; }
        }
        if (!have_x || !have_y || !have_yaw ||
            !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return false;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation = TransformUtils::yaw_to_quaternion(yaw);
        RCLCPP_INFO(this->get_logger(), "map_name= %s,last_pose配置完整路径：%s",map_name.c_str(), path.c_str());
        return true;
    }

    bool AgvNavServerNode::save_remembered_pose(
        const std::string &map_name,
        const geometry_msgs::msg::Pose &pose,
        std::string &error) const
    {
        std::string directory_error;
        if (!map_file_manager_->ensureMapDirectory(map_name, directory_error))
        {
            error = directory_error;
            return false;
        }
        const std::string path = map_file_manager_->mapDirectory(map_name) + "/last_pose.yaml";
        const std::string temporary = path + ".tmp";
        std::ofstream out(temporary, std::ios::trunc);
        if (!out)
        {
            error = "无法写入位置记忆: " + temporary;
            return false;
        }
        out << std::setprecision(17)
            << "x: " << pose.position.x << '\n'
            << "y: " << pose.position.y << '\n'
            << "yaw: " << TransformUtils::quaternion_to_yaw(pose.orientation) << '\n';
        out.close();
        if (!out || std::rename(temporary.c_str(), path.c_str()) != 0)
        {
            error = "提交位置记忆失败: " + path;
            std::remove(temporary.c_str());
            return false;
        }
        return true;
    }

    bool AgvNavServerNode::call_write_state(const std::string &pbstream_abs_path, std::string &error)
    {
        if (!write_state_client_->wait_for_service(std::chrono::seconds(5)))
        {
            error = "cartographer /write_state 服务不可用（建图节点未运行？）";
            return false;
        }

        auto request = std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>();
        request->filename = pbstream_abs_path;

        auto future = write_state_client_->async_send_request(request);
        const auto rc = future.wait_for(std::chrono::seconds(15));
        if (rc != std::future_status::ready)
        {
            error = "/write_state 调用超时（15s）";
            return false;
        }
        // Humble 版 WriteState 响应不带结果字段：服务正常返回后以文件落盘为准
        future.get();

        struct stat st;
        if (::stat(pbstream_abs_path.c_str(), &st) != 0 || st.st_size <= 0)
        {
            error = "/write_state 已返回但 pbstream 未落盘: " + pbstream_abs_path;
            return false;
        }
        return true;
    }

} // namespace agv_bridge
