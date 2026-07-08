#ifndef HTTP_API_CLIENT_HPP
#define HTTP_API_CLIENT_HPP

#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include "agv_bridge_v2/bean/CommandAck.hpp"
#include "agv_bridge_v2/bean/LaserScan.hpp"
#include "agv_bridge_v2/bean/PositionUpdate.hpp"
#include "agv_bridge_v2/bean/StatusUpdate.hpp"


// 方便使用的宏，用于追踪HTTP请求的源代码位置
#define SEND_TO_psringboot(client, endpoint, data) \
    client->send_to_springboot_with_trace(endpoint, data, __FILE__, __LINE__)

class HttpApiClient {
public:
    using LoggerCallback = std::function<void(int level, const std::string& message)>;

    // 日志级别枚举
    enum LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3
    };

    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param host SpringBoot服务器主机地址
     * @param port SpringBoot服务器端口
     * @param agv_id AGV ID
     * @param logger_callback 日志回调函数，用于将日志输出到ROS2
     */
    HttpApiClient(rclcpp::Node* node, const std::string& host, int port, const std::string& agv_id, 
                  LoggerCallback logger_callback = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~HttpApiClient();

    /**
     * @brief 设置AGV是否已注册
     * @param registered 注册状态
     */
    void setRegistered(bool registered);

    /**
     * @brief 设置WebSocket客户端数量
     * @param count 客户端数量
     */
    void setWebSocketClientCount(int count);

    /**
     * @brief 发送数据到SpringBoot（带源代码追踪）
     * @param endpoint API端点
     * @param data JSON数据
     * @param file 源文件名（由宏自动传入）
     * @param line 行号（由宏自动传入）
     */
    bool send_to_springboot_with_trace(const std::string& endpoint, const nlohmann::json &data,
                                       const char* file, int line);

    /**
     * @brief 发送数据到SpringBoot
     * @param endpoint API端点
     * @param data JSON数据
     * @return 是否发送成功
     */
    bool send_to_springboot(const std::string& endpoint, const nlohmann::json &data);

    /**
     * @brief 发送心跳数据
     */
    void send_heartbeat();

    /**
     * @brief 发送位置更新
     * @param x X坐标
     * @param y Y坐标
     * @param theta 朝向角
     * @param vx 线速度X
     * @param vy 线速度Y
     * @param omega 角速度
     */
    // void send_position_update(double x, double y, double theta, 
    //                           double vx = 0.0, double vy = 0.0, double omega = 0.0);

    /**
     * @brief 发送雷达扫描数据
     * @param range_min 最小距离
     * @param range_max 最大距离
     * @param angle_min 最小角度
     * @param angle_max 最大角度
     * @param ranges 距离数组
     */
    // void send_laser_scan(float range_min, float range_max,
    //                      float angle_min, float angle_max,
    //                      const std::vector<float>& ranges);

    /**
     * @brief 发送状态更新
     * @param state AGV状态
     * @param battery 电池电量
     */
    // void send_status(const std::string& state, double battery,const bool pose_initialized);

    /**
     * @brief 发送命令确认
     * @param command_id 命令ID
     * @param node_id 节点ID
     * @param command_type 
     * @param status 状态（SUCCESS/FAILED/CANCELED等）
     * @param message 附加消息
     */
    // void send_command_ack(const std::string& command_id, const std::string& node_id,const std::string& command_type,
    //                       const std::string& status, const std::string& message = "");

    // 新增方法：发送定位状态
    // bool send_localization_status(bool is_initialized, const std::string& message = "");

    /**
     * @brief 获取当前时间（ISO格式）
     * @return ISO格式的时间字符串
     */
    static std::string get_current_time_iso();

    void send_position_update(const PositionUpdate& update);
    void send_laser_scan(const LaserScan& scan);
    void send_status(const StatusUpdate& status);
    void send_command_ack(const CommandAck& ack);

private:
    // CURL回调函数
    static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp);

    // 实际发送HTTP请求
    bool http_post(const std::string& url, const std::string& json_str, std::string& response);

    // 日志输出
    void log(LogLevel level, const std::string& message) const;

    // 成员变量
    rclcpp::Node* node_;          // ROS2节点指针
    rclcpp::Logger logger_;       // 子logger，日志tag显示类名
    std::string host_;
    int port_;
    std::string agv_id_;
    bool is_registered_;
    int websocket_client_count_;
    
    // 日志回调
    LoggerCallback logger_callback_;
    
    // 互斥锁，用于线程安全
    std::mutex mutex_;
};

#endif // HTTP_API_CLIENT_HPP