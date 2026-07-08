// src/HttpApiClient.cpp
#include "agv_bridge_v2/HttpApiClient.hpp"
#include <curl/curl.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

HttpApiClient::HttpApiClient(rclcpp::Node* node, const std::string &host, int port, const std::string &agv_id,
                             LoggerCallback logger_callback)
    : node_(node), 
      logger_(node->get_logger().get_child("HttpApiClient")),
      host_(host), port_(port), agv_id_(agv_id), is_registered_(false), websocket_client_count_(0), logger_callback_(logger_callback)
{
    // 全局初始化CURL
    curl_global_init(CURL_GLOBAL_DEFAULT);
    log(INFO, "HttpApiClient initialized for AGV: " + agv_id_);
}

HttpApiClient::~HttpApiClient()
{
    curl_global_cleanup();
    log(INFO, "HttpApiClient destroyed");
}

void HttpApiClient::setRegistered(bool registered)
{
    std::lock_guard<std::mutex> lock(mutex_);
    is_registered_ = registered;
}

void HttpApiClient::setWebSocketClientCount(int count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    websocket_client_count_ = count;
}

bool HttpApiClient::send_to_springboot_with_trace(const std::string &endpoint, const nlohmann::json &data,
                                                  const char *file, int line)
{
    // 检查是否可以发送
    if (!is_registered_ || websocket_client_count_ == 0)
    {
        log(WARN, "Skipping HTTP request to " + endpoint +
                      " (registered: " + std::to_string(is_registered_) +
                      ", clients: " + std::to_string(websocket_client_count_) + ")");
        return false;
    }

    bool success = send_to_springboot(endpoint, data);

    if (!success)
    {
        log(WARN, "HTTP request failed [" + std::string(file) + ":" + std::to_string(line) +
                      "] to endpoint: " + endpoint);
    }
    return success;
}

bool HttpApiClient::send_to_springboot(const std::string &endpoint, const nlohmann::json &data)
{
    std::string url = "http://" + host_ + ":" + std::to_string(port_) + "/api/agv/" + endpoint;
    std::string json_str = data.dump();
    std::string response;

    bool success = http_post(url, json_str, response);

    if (success)
    {
        log(DEBUG, "Successfully sent data to " + endpoint);
    }

    return success;
}

void HttpApiClient::send_heartbeat()
{
    nlohmann::json heartbeat;
    heartbeat["agv_id"] = agv_id_;
    heartbeat["timestamp"] = get_current_time_iso();
    heartbeat["type"] = "heartbeat";

    send_to_springboot_with_trace("heartbeat", heartbeat, __FILE__, __LINE__);
}

// void HttpApiClient::send_position_update(double x, double y, double theta,
//                                          double vx, double vy, double omega)
// {
//     nlohmann::json position_msg;
//     position_msg["agv_id"] = agv_id_;
//     position_msg["timestamp"] = get_current_time_iso();

//     // 位置信息
//     position_msg["position"]["x"] = x;
//     position_msg["position"]["y"] = y;
//     position_msg["position"]["theta"] = theta;

//     // 速度信息
//     position_msg["velocity"]["vx"] = vx;
//     position_msg["velocity"]["vy"] = vy;
//     position_msg["velocity"]["omega"] = omega;

//     // send_to_springboot("position_update", position_msg);
//     send_to_springboot_with_trace("handle_position_update", position_msg, __FILE__, __LINE__);
// }

// void HttpApiClient::send_laser_scan(float range_min, float range_max,
//                                     float angle_min, float angle_max,
//                                     const std::vector<float> &ranges)
// {
//     nlohmann::json scan_msg;
//     scan_msg["agv_id"] = agv_id_;
//     scan_msg["timestamp"] = get_current_time_iso();
//     scan_msg["range_min"] = range_min;
//     scan_msg["range_max"] = range_max;
//     scan_msg["angle_min"] = angle_min;
//     scan_msg["angle_max"] = angle_max;
//     scan_msg["ranges"] = ranges;

//     // send_to_springboot("laser_scan", scan_msg);
//     send_to_springboot_with_trace("handle_laser_scan", scan_msg, __FILE__, __LINE__);
// }

// void HttpApiClient::send_status(const std::string &state, double battery,const bool pose_initialized)
// {
//     nlohmann::json status_msg;
//     status_msg["agv_id"] = agv_id_;
//     status_msg["timestamp"] = get_current_time_iso();
//     status_msg["state"] = state;
//     status_msg["battery"] = battery;
//     status_msg["pose_initialized"] = pose_initialized;

//     // send_to_springboot("status", status_msg);
//     send_to_springboot_with_trace("handle_status", status_msg, __FILE__, __LINE__);
// }

// void HttpApiClient::send_command_ack(const std::string &command_id, const std::string &node_id, const std::string &command_type,
//                                      const std::string &status, const std::string &message)
// {
//     nlohmann::json ack_msg;
//     ack_msg["type"] = command_type;
//     ack_msg["agv_id"] = agv_id_;
//     ack_msg["command_id"] = command_id;
//     ack_msg["node_id"] = node_id;
//     ack_msg["status"] = status;
//     ack_msg["message"] = message;
//     ack_msg["timestamp"] = get_current_time_iso();

//     // send_to_springboot("command_ack", ack_msg);
//     send_to_springboot_with_trace("handle_command_ack", ack_msg, __FILE__, __LINE__);
// }


void HttpApiClient::send_position_update(const PositionUpdate& update)
{
    nlohmann::json j = update;
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "→ POST handle_position_update: (%.2f, %.2f, %.2f)",
                         update.x, update.y, update.theta);
    bool ok = send_to_springboot_with_trace("handle_position_update", j, __FILE__, __LINE__);
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "← handle_position_update %s", ok ? "OK" : "FAIL");
}

void HttpApiClient::send_laser_scan(const LaserScan& scan)
{
    nlohmann::json j = scan;
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "→ POST handle_laser_scan: %zu ranges", scan.ranges.size());
    bool ok = send_to_springboot_with_trace("handle_laser_scan", j, __FILE__, __LINE__);
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "← handle_laser_scan %s", ok ? "OK" : "FAIL");
}

void HttpApiClient::send_status(const StatusUpdate& status)
{
    nlohmann::json j = status;
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "→ POST handle_status: battery=%.1f%%, initialized=%s",
                         status.battery, status.pose_initialized ? "true" : "false");
    bool ok = send_to_springboot_with_trace("handle_status", j, __FILE__, __LINE__);
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "← handle_status %s", ok ? "OK" : "FAIL");
}

void HttpApiClient::send_command_ack(const CommandAck& ack)
{
    nlohmann::json j = ack;
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "→ POST handle_command_ack: cmd=%s, status=%s",
                         ack.command_id.c_str(), ack.status.c_str());
    bool ok = send_to_springboot_with_trace("handle_command_ack", j, __FILE__, __LINE__);
    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 10000,
                         "← handle_command_ack %s", ok ? "OK" : "FAIL");
}

size_t HttpApiClient::write_callback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
}

bool HttpApiClient::http_post(const std::string &url, const std::string &json_str, std::string &response)
{
    CURL *curl = curl_easy_init();
    if (!curl)
    {
        log(ERROR, "Failed to create CURL easy handle");
        return false;
    }

    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L); // 避免多线程信号问题

    CURLcode res = curl_easy_perform(curl);

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return res == CURLE_OK;
}

std::string HttpApiClient::get_current_time_iso()
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

void HttpApiClient::log(LogLevel level, const std::string &message) const
{
    // 通过回调输出日志
    if (logger_callback_)
    {
        logger_callback_(static_cast<int>(level), "[HttpApiClient] " + message);
    }
    
    // 同时通过 rclcpp logger 输出
    switch (level)
    {
        case DEBUG:
            RCLCPP_DEBUG(logger_, "%s", message.c_str());
            break;
        case INFO:
            RCLCPP_INFO(logger_, "%s", message.c_str());
            break;
        case WARN:
            RCLCPP_WARN(logger_, "%s", message.c_str());
            break;
        case ERROR:
            RCLCPP_ERROR(logger_, "%s", message.c_str());
            break;
    }
}