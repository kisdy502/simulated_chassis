// PositionUpdate.hpp
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "agv_bridge_v2/bean/BaseMessage.hpp"

struct PositionUpdate :public BaseMessage{
    std::string agv_id;
    // std::string timestamp;
    double x = 0.0;
    double y = 0.0;
    double qx = 0.0;  // 四元数 x
    double qy = 0.0;  // 四元数 y
    double qz = 0.0;  // 四元数 z
    double qw = 1.0;  // 四元数 w（默认单位四元数）
    double theta = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    int64_t timestamp_ns; // 纳秒级时间戳，表示位置数据的采集时间（相对于某个参考时间点，如系统启动或1970年）
};

void to_json(nlohmann::json& j, const PositionUpdate& p);
void from_json(const nlohmann::json& j, PositionUpdate& p);


