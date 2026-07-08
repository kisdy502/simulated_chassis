#include "agv_bridge_v2/AgvBridgeNode.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <curl/curl.h>

using namespace std::chrono_literals;
using json = nlohmann::json;
using namespace agv_bridge;

namespace agv_bridge
{

    AgvBridgeNode::AgvBridgeNode() : Node("agv_bridge_node"), battery_level_(100.0), agv_state_("IDLE")
    {
        // 初始化参数
        initialize_parameters();
        // 初始化组件
        initialize_components();
        // 创建订阅器
        create_subscriptions();

        // 创建发布器
        create_publishers();

        // 创建定时器
        create_timers();

        // 启动WebSocket服务器
        start_websocket_server();

        RCLCPP_INFO(this->get_logger(), "AGV Bridge节点已启动，AGV ID: %s", agv_id_.c_str());
    }

    AgvBridgeNode::~AgvBridgeNode() noexcept
    {
        try
        {
            // 停止WebSocket服务器
            if (websocket_server_ && websocket_server_->isRunning())
            {
                websocket_server_->stop();
            }
            // 清理CURL全局资源
            curl_global_cleanup();

            RCLCPP_INFO(this->get_logger(), "AGV Bridge节点已关闭");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "析构函数中发生异常: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "析构函数中发生未知异常");
        }
    }

    void AgvBridgeNode::initialize_parameters()
    {
        // 声明参数
        this->declare_parameter<std::string>("agv_id", "AGV001");
        this->declare_parameter<std::string>("springboot_host", "192.168.2.130");
        this->declare_parameter<int>("springboot_port", 22777);
        this->declare_parameter<int>("websocket_port", 9090);
        this->declare_parameter<std::string>("robot_namespace", "/");

        // 获取参数
        this->get_parameter("agv_id", agv_id_);
        this->get_parameter("springboot_host", springboot_host_);
        this->get_parameter("springboot_port", springboot_port_);
        this->get_parameter("websocket_port", websocket_port_);

          // IMU 参数
        this->declare_parameter<std::string>("imu_serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("imu_baud_rate", 115200);
        this->declare_parameter<int>("imu_window_size", 5);
        this->declare_parameter<double>("imu_gyro_limit", 300.0);
        this->declare_parameter<double>("imu_acc_limit", 4.0);
    
        // 获取参数（注意需要对应的成员变量）
        this->get_parameter("imu_serial_port", imu_serial_port_);
        this->get_parameter("imu_baud_rate", imu_baud_rate_);

        // 初始化时间
        last_scan_time_ = this->now();
    }

    void AgvBridgeNode::initialize_components()
    {
        imu_manager_ = std::make_shared<ImuManager>(this);
        if (!imu_manager_->initialize(imu_serial_port_, imu_baud_rate_,
                                    imu_window_size_, imu_gyro_limit_, imu_acc_limit_)) {
            RCLCPP_WARN(this->get_logger(), "IMU 初始化失败，将不提供 IMU 数据");
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU 管理器已启动");
        }


        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 初始化定位监控器
        LocalizationMonitor::Config loc_config;
        loc_config.particle_spread_threshold = 1.0;
        loc_config.required_stable_count = 5;
        loc_config.quality_threshold = 0.5;

        localization_monitor_ = std::make_shared<LocalizationMonitor>(this, loc_config);

        // 初始化机器人位姿提供者
        // robot_pose_provider_ = std::make_shared<agv_bridge::RobotPoseProvider>(this);

        // 初始化导航管理器
        navigation_manager_ = std::make_shared<agv_bridge::NavigationManager>(this, this, localization_monitor_);

        if (!navigation_manager_->initialize())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize navigation manager");
        }

        // 初始化HTTP API客户端
        http_client_ = std::make_shared<HttpApiClient>(
            this,  // 添加 node 指针作为第一个参数
            springboot_host_,
            springboot_port_,
            agv_id_,
            [this](int level, const std::string &message)
            {
                switch (level)
                {
                case HttpApiClient::DEBUG:
                    RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
                    break;
                case HttpApiClient::INFO:
                    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
                    break;
                case HttpApiClient::WARN:
                    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                    break;
                case HttpApiClient::ERROR:
                    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
                    break;
                }
            });
    }

    void AgvBridgeNode::create_subscriptions()
    {
        // 创建QoS配置
        auto qos = rclcpp::QoS(10);

        // 订阅里程计
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos,
            std::bind(&AgvBridgeNode::odom_callback, this, std::placeholders::_1));

        // 订阅雷达数据
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos,
            std::bind(&AgvBridgeNode::scan_callback, this, std::placeholders::_1));

        // 订阅速度命令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos,
            std::bind(&AgvBridgeNode::cmd_vel_callback, this, std::placeholders::_1));
    }

    void AgvBridgeNode::create_publishers()
    {
        auto qos = rclcpp::QoS(10);

        // 发布目标点
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", qos);

        // 发布初始位置
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", qos);

        // 发布速度命令
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

        // 发布AGV控制
        agv_control_pub_ = this->create_publisher<std_msgs::msg::String>("agv_control", qos);
    }

    void AgvBridgeNode::create_timers()
    {
        // 状态发布定时器（1秒）
        status_timer_ = this->create_wall_timer(
            1000ms, std::bind(&AgvBridgeNode::publish_status, this));

        // 心跳定时器（5秒）
        heartbeat_timer_ = this->create_wall_timer(5000ms, std::bind(&AgvBridgeNode::send_heartbeat, this));

        // TF广播定时器（100毫秒）
        tf_timer_ = this->create_wall_timer(100ms, std::bind(&AgvBridgeNode::broadcast_tf, this));
        // 定位监控定时器（1秒）
        localization_timer_ = this->create_wall_timer(1000ms, std::bind(&AgvBridgeNode::update_localization_monitor, this));

        // 新增：位置上报定时器（3Hz）
        position_report_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(report_interval_),
            std::bind(&AgvBridgeNode::report_position, this));
    }

    void AgvBridgeNode::start_websocket_server()
    {
        try
        {
            websocket_server_ = std::make_shared<WebSocketServer>(
                websocket_port_,
                [this](const std::string &message)
                {
                    this->handle_websocket_message(message);
                });
            websocket_server_->start();
            RCLCPP_INFO(this->get_logger(), "WebSocket服务器已启动在端口 %d", websocket_port_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "启动WebSocket服务器失败: %s", e.what());
        }
    }

    void AgvBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_vel_ = msg->twist.twist;
    }

    void AgvBridgeNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 每分钟打印一次，确认订阅能收到数据
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                             "收到雷达数据: frame_id=%s, 点数=%zu, stamp=%.3f",
                             msg->header.frame_id.c_str(),
                             msg->ranges.size(),
                             rclcpp::Time(msg->header.stamp).seconds());

        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
        LaserScan scan;
        scan.agv_id = agv_id_;
        scan.timestamp = get_current_timestamp_ms();
        scan.range_min = msg->range_min;
        scan.range_max = msg->range_max;
        scan.angle_min = msg->angle_min;
        scan.angle_max = msg->angle_max;
        scan.ranges = ranges; // 假设ranges已经定义

        // 2. 通过TF获取变换关系
        geometry_msgs::msg::TransformStamped transform;               // 包含平移和旋转关系
        if (localization_monitor_->getTransform(msg->header.frame_id, // 源坐标系（通常是laser_link）
                                                msg->header.stamp,    // 使用消息的时间戳！
                                                transform))
        {
            scan.pose_x = transform.transform.translation.x;
            scan.pose_y = transform.transform.translation.y;
            scan.pose_theta = TransformUtils::quaternion_to_yaw(transform.transform.rotation);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 15000,
                                 "获取雷达坐标变换失败!");
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 15000,
                             "雷达时间:%s,点数:%zu,有效距离:%.2f~%.2f m, 扫描角度: %.1f ~ %.1f度, 角增量: %.3f度,耗时: %.1fs",
                             get_current_time_iso().c_str(),
                             msg->ranges.size(),
                             msg->range_min, msg->range_max,
                             msg->angle_min, msg->angle_max,
                             msg->angle_increment,
                             msg->scan_time);

        // 每 0.3 秒发送一次雷达数据（业务节流，非日志）
        auto now = this->now();
        if ((now - last_scan_time_).seconds() >= 0.3)
        {
            last_scan_time_ = now;
            if (http_client_)
            {
                http_client_->send_laser_scan(scan);
            }
        }
    }

    void AgvBridgeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "收到速度命令: v=%.2f, ω=%.2f",
                     msg->linear.x, msg->angular.z);
    }

    void AgvBridgeNode::publish_status()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        json status_msg;
        status_msg["agv_id"] = agv_id_;
        status_msg["timestamp"] = get_current_timestamp_ms();
        status_msg["state"] = agv_state_;
        status_msg["battery"] = battery_level_;
        status_msg["pose_initialized"] = localization_monitor_->isInitialized();

        // 本地发布状态
        auto ros_msg = std_msgs::msg::String();
        ros_msg.data = status_msg.dump();
        agv_control_pub_->publish(ros_msg);

        StatusUpdate status;
        status.agv_id = agv_id_;
        status.timestamp = get_current_timestamp_ms();
        status.battery = battery_level_;
        status.pose_initialized = localization_monitor_->isInitialized();

        // 发送状态到服务器
        if (http_client_)
        {
            http_client_->send_status(status);
        }
    }

    void AgvBridgeNode::send_heartbeat()
    {
        // 客户端发给服务器就行了，不要服务器发给客户端了
        //  if (http_client_)
        //  {
        //      http_client_->send_heartbeat();
        //  }
    }

    void AgvBridgeNode::report_position()
    {
        // 加锁保护 robot_last_pose_
        std::lock_guard<std::mutex> lock(mutex_);

        // 检查是否有有效位置（可根据需要判断，例如是否初始化过）
        if (!localization_monitor_->isInitialized())
        {
            RCLCPP_DEBUG(this->get_logger(), "位置未初始化，跳过上报");
            return;
        }
        auto current_pose_opt = localization_monitor_->getCurrentPose();
        if (!current_pose_opt.has_value())
        {
            RCLCPP_DEBUG(this->get_logger(), "位置信息为空");
            return;
        }

        const auto &current_pose = current_pose_opt.value();

        // 从 robot_last_pose_ 构建更新数据（注意 robot_last_pose_ 是 geometry_msgs::msg::Pose 类型）
        double yaw = TransformUtils::quaternion_to_yaw(current_pose.pose.orientation);

        PositionUpdate update;
        update.agv_id = agv_id_;
        update.x = current_pose.pose.position.x;
        update.y = current_pose.pose.position.y;
        // 填充四元数
        update.qx = current_pose.pose.orientation.x;
        update.qy = current_pose.pose.orientation.y;
        update.qz = current_pose.pose.orientation.z;
        update.qw = current_pose.pose.orientation.w;

        update.theta = yaw;
        // 如果需要速度，可以从 current_vel_ 获取（也在 mutex_ 保护下）
        update.vx = current_vel_.linear.x;
        update.vy = current_vel_.linear.y;
        update.omega = current_vel_.angular.z;
        // 方案2：使用ROS2的Clock获取纳秒
        uint64_t timestamp_ms = 0;
        if (current_pose.header.stamp.sec != 0 || current_pose.header.stamp.nanosec != 0)
        {
            update.timestamp_ns = current_pose.header.stamp.sec * 1000000000ULL +
                                  current_pose.header.stamp.nanosec;
            update.timestamp = update.timestamp_ns / 1000000ULL;
        }

        if (http_client_)
        {
            http_client_->send_position_update(update);
            RCLCPP_DEBUG(this->get_logger(), "周期上报位置: (%.2f, %.2f, %.2f)",
                         update.x, update.y, update.theta);
        }
    }

    void AgvBridgeNode::broadcast_tf()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto current_pose_opt = localization_monitor_->getCurrentPose();
        if (!current_pose_opt.has_value())
        {
            return;
        }

        const auto &current_pose = current_pose_opt.value().pose;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = agv_id_ + "/base_link";

        transform.transform.translation.x = current_pose.position.x;
        transform.transform.translation.y = current_pose.position.y;
        transform.transform.translation.z = current_pose.position.z;
        transform.transform.rotation = current_pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    void AgvBridgeNode::update_localization_monitor()
    {
        if (!localization_monitor_)
        {
            return;
        }

        localization_monitor_->update();

        // 记录定位状态变化
        static bool last_initialized_state = false;
        bool current_initialized_state = localization_monitor_->isInitialized();

        if (current_initialized_state != last_initialized_state)
        {
            if (current_initialized_state)
            {
                RCLCPP_INFO(this->get_logger(), "状态变化，位置初始化完成 ✅");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "状态变化，位置未初始化 ❌ ");
            }
            last_initialized_state = current_initialized_state;
        }

        // 每 30 秒打印一次当前定位状态
        if (current_initialized_state)
        {
            auto initial_pose = localization_monitor_->getCurrentPose().value();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                 "✅ 位置初始化完成，导航系统已就绪,位置: (%.2f, %.2f)",
                                 initial_pose.pose.position.x, initial_pose.pose.position.y);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                 "❌ 位置未初始化，等待 Cartographer 纯定位收敛...");
        }
    }

    void AgvBridgeNode::handle_websocket_message(const std::string &message)
    {

        try
        {
            json data = json::parse(message);
            std::string cmd_type = data.value("type", "");

            RCLCPP_DEBUG(this->get_logger(), "收到WebSocket消息, type=%s, content=%s", cmd_type.c_str(), message.c_str());

            if (cmd_type == "register")
            {
                if (http_client_)
                {
                    http_client_->setRegistered(true);
                    http_client_->setWebSocketClientCount(1);
                }
                RCLCPP_INFO(this->get_logger(), "AGV 连接到ros2 bridge节点: %s", message.c_str());
            }
            else if (cmd_type == "heartbeat")
            {

                RCLCPP_DEBUG(this->get_logger(), "收到AGV心跳: %s", message.c_str());
            }
            else if (cmd_type == "move_to")
            {
                RCLCPP_WARN(this->get_logger(), "handle websocket move message: %s", message.c_str());
                handle_move_command(data);
            }
            else if (cmd_type == "stop_move")
            {
            
                if (navigation_manager_)
                {
                    navigation_manager_->stopNavigation();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "NavigationManager 未初始化，无法停止移动");
                }
            }
            else if (cmd_type == "set_initial_pose")
            {
                handle_initial_pose(data);
            }
            else if (cmd_type == "velocity_command")
            {
                handle_velocity_command(data);
            }
            else if (cmd_type == "agv_control")
            {
                handle_agv_control(data);
            }
            else if (cmd_type == "query_status")
            {
                publish_status();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "未知命令类型: %s", cmd_type.c_str());
            }
        }
        catch (const json::parse_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "无效的JSON消息 (文件: %s, 行: %d, 函数: %s): %s, 消息内容: %s",
                         __FILE__, __LINE__, __FUNCTION__, e.what(), message.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理消息时出错 (文件: %s, 行: %d, 函数: %s): %s, 消息内容: %s",
                         __FILE__, __LINE__, __FUNCTION__, e.what(), message.c_str());
        }
    }

    /*
    {
        "requestId": "move_42_ec1bda99",
        "type": "move_to",
        "timestamp": 1772445676314,
        "agvId": "AGV001",
        "commandId": "move_0ca1e7b0-9a9f-470e-9a16-10035ac4237c",
        "nodeId": "I006",
        "x": -8.299999564886093,
        "y": -1.349999736994505,
        "theta": 0.1787930548029214,
        "edgeInfo": {
            "id": "E066",
            "sourceId": "I006",
            "targetId": "S008",
            "weight": 6.500000096857548,
            "direction": "BIDIRECTIONAL",
            "type": "STRAIGHT",
            "maxSpeed": 0.6,
            "priority": 1,
            "enabled": true,
            "controlPoints": null
        },
        "endPoint": false
    }
    */
    void AgvBridgeNode::handle_move_command(const json &data)
    {
        try
        {
            // 1. 将JSON直接解析为MoveToMessage对象
            auto move_msg = data.get<agv_bridge::MoveToMessage>();

            move_msg.edgeInfo.step = 0.1;

            auto current_pose_opt = localization_monitor_->getCurrentPose();
            if (!current_pose_opt.has_value())
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取当前机器人位姿，拒绝执行导航命令");
                // 发送失败 ACK
                CommandAck ack;
                ack.agv_id = agv_id_;
                ack.timestamp = get_current_timestamp_ms();
                ack.command_id = move_msg.commandId;
                ack.node_id = move_msg.nodeId;
                ack.type = "command_ack";
                ack.status = "FAILED";
                ack.message = "无法获取当前机器人位姿，拒绝执行导航命令";
                if (http_client_)
                {
                    http_client_->send_command_ack(ack);
                }
                return;
            }

            // （0=STRAIGHT, 1=CURVE, 2=ELEVATION）
            RCLCPP_INFO(this->get_logger(), "处理边 %s,类型:%d(0=STRAIGHT, 1=CURVE, 2=ELEVATION),目标点: %s", move_msg.edgeInfo.id.c_str(), static_cast<int>(move_msg.edgeInfo.type), move_msg.nodeId.c_str());
            if (move_msg.edgeInfo.isBezierCurve())
            {
                RCLCPP_INFO(this->get_logger(), "贝塞尔曲线阶数: %d, 控制点数量: %zu",
                            move_msg.edgeInfo.getBezierOrder(), move_msg.edgeInfo.controlPoints.size());
            }

            // 判断方向：如果目标节点是边的起点，则需要反向
            if (move_msg.nodeId == move_msg.edgeInfo.sourceId)
            {
                move_msg.edgeInfo.reverse = true;
                RCLCPP_INFO(this->get_logger(), "检测到反向导航: 目标节点 %s 是边 %s 的起点",
                            move_msg.nodeId.c_str(), move_msg.edgeInfo.id.c_str());
            }
            else if (move_msg.nodeId == move_msg.edgeInfo.targetId)
            {
                move_msg.edgeInfo.reverse = false;
                RCLCPP_INFO(this->get_logger(), "检测到正向导航: 目标节点 %s 是边 %s 的终点",
                            move_msg.nodeId.c_str(), move_msg.edgeInfo.id.c_str());
            }

            // 根据边类型执行导航
            if (move_msg.edgeInfo.isBezierCurve())
            {
                if (navigation_manager_->followBezierPathNavigation(move_msg))
                {
                    RCLCPP_INFO(this->get_logger(), "贝塞尔曲线导航已启动, 边ID: %s, 最大速度: %.2f m/s", move_msg.edgeInfo.id.c_str(), move_msg.edgeInfo.maxSpeed);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "启动贝塞尔曲线导航失败");
                    // 发送失败 ACK
                    CommandAck ack;
                    ack.agv_id = agv_id_;
                    ack.timestamp = get_current_timestamp_ms();
                    ack.command_id = move_msg.commandId;
                    ack.node_id = move_msg.nodeId;
                    ack.type = "command_ack";
                    ack.status = "FAILED";
                    ack.message = "Failed to start Bezier curve navigation";
                    if (http_client_)
                    {
                        http_client_->send_command_ack(ack);
                    }
                }
            }
            else
            {
                // 普通直线导航
                if (navigation_manager_)
                {
                    navigation_manager_->followPathNavigation(move_msg);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "NavigationManager 没有创建成功");
                    // 发送失败 ACK
                    CommandAck ack;
                    ack.agv_id = agv_id_;
                    ack.timestamp = get_current_timestamp_ms();
                    ack.command_id = move_msg.commandId;
                    ack.node_id = move_msg.nodeId;
                    ack.type = "command_ack";
                    ack.status = "FAILED";
                    ack.message = "NavigationManager 没有创建成功";
                    if (http_client_)
                    {
                        http_client_->send_command_ack(ack);
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "handle_move_command 异常: %s\nJSON: %s", e.what(), data.dump().c_str());
        }
    }

    void AgvBridgeNode::handle_initial_pose(const json &data)
    {
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.stamp = this->now();
        initial_pose.header.frame_id = "map";

        initial_pose.pose.pose.position.x = data["x"];
        initial_pose.pose.pose.position.y = data["y"];
        initial_pose.pose.pose.position.z = 0.0;

        if (data.contains("theta"))
        {
            initial_pose.pose.pose.orientation = agv_bridge::TransformUtils::yaw_to_quaternion(data["theta"]);
        }
        else
        {
            initial_pose.pose.pose.orientation.w = 1.0;
        }

        // 设置协方差
        initial_pose.pose.covariance[0] = 0.25;
        initial_pose.pose.covariance[7] = 0.25;
        initial_pose.pose.covariance[35] = 0.06853891945200942;

        initial_pose_pub_->publish(initial_pose);

        RCLCPP_INFO(this->get_logger(), "设置初始位置: (%.2f, %.2f)",
                    data["x"].get<double>(),
                    data["y"].get<double>());
    }

    void AgvBridgeNode::handle_velocity_command(const json &data)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = data.value("vx", 0.0);
        twist.linear.y = data.value("vy", 0.0);
        twist.angular.z = data.value("omega", 0.0);

        cmd_vel_pub_->publish(twist);
    }

    void AgvBridgeNode::handle_agv_control(const json &data)
    {
        std::string action = data.value("action", "");

        if (action == "start")
        {
            agv_state_ = "EXECUTING";
        }

        else if (action == "stop")
        {
            agv_state_ = "IDLE";
        }
        else if (action == "reset")
        {
            agv_state_ = "IDLE";
        }

        RCLCPP_INFO(this->get_logger(), "AGV状态改变: %s", agv_state_.c_str());
    }

    std::string AgvBridgeNode::get_current_time_iso() const
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;

        std::ostringstream oss;
        oss << std::put_time(std::gmtime(&now_c), "%Y-%m-%dT%H:%M:%S");
        oss << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << "Z";

        return oss.str();
    }

    int64_t AgvBridgeNode::get_current_timestamp_ms() const
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }

    void AgvBridgeNode::onNavigationStateChanged(agv_bridge::NavigationState state,
                                                 const std::string &command_id)
    {
        switch (state)
        {
        case agv_bridge::NavigationState::IDLE:
            agv_state_ = "IDLE";
            break;
        case agv_bridge::NavigationState::PRE_ROTATING:
            agv_state_ = "PRE_ROTATING";
            break;
        case agv_bridge::NavigationState::EXECUTING:
            agv_state_ = "EXECUTING";
            break;
        case agv_bridge::NavigationState::COMPLETED:
            agv_state_ = "COMPLETED";
            break;
        case agv_bridge::NavigationState::END_ROTATING:
            agv_state_ = "END_ROTATING";
            break;
        case agv_bridge::NavigationState::FAILED:
            agv_state_ = "FAILED";
            break;
        case agv_bridge::NavigationState::CANCELLED:
            agv_state_ = "CANCELLED";
            break;
        }

        RCLCPP_INFO(this->get_logger(), "Navigation state changed, AGV state: %s", agv_state_.c_str());
    }

    void AgvBridgeNode::onNavigationFeedback(double x, double y, double theta)
    {
        RCLCPP_DEBUG(this->get_logger(), "Navigation feedback: (%.2f, %.2f, %.2f)", x, y, theta);
    }

    void AgvBridgeNode::onNavigationResult(bool success, const std::string &message,
                                           const std::string &command_id,
                                           const std::string &node_id)

    {
        RCLCPP_INFO(this->get_logger(), "Navigation result: %s, message: %s",
                    success ? "SUCCESS" : "FAILED", message.c_str());
    }

    void AgvBridgeNode::onCommandAck(const std::string &command_id,
                                     const std::string &node_id,
                                     const std::string &status,
                                     const std::string &message)
    {
        // 发送命令确认到HTTP服务器
        if (http_client_)
        {
            CommandAck ack;
            ack.agv_id = agv_id_;
            ack.timestamp = get_current_timestamp_ms();
            ack.command_id = command_id;
            ack.node_id = node_id;
            ack.type = "move_result"; // 对应 command_type
            ack.status = status;
            ack.message = message; // 可选，根据业务需要可留空或设置具体消息
            http_client_->send_command_ack(ack);
        }
    }

} // namespace agv_bridge