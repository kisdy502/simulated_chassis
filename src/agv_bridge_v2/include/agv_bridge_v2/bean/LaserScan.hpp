// LaserScan.hpp
#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "agv_bridge_v2/bean/BaseMessage.hpp"

struct LaserScan : public BaseMessage
{
    std::string agv_id;
    // std::string timestamp;
    float range_min = 0.0f;
    float range_max = 0.0f;
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    std::vector<float> ranges;
    double pose_x;
    double pose_y;
    double pose_theta; // 弧度
};

void to_json(nlohmann::json &j, const LaserScan &p);
void from_json(const nlohmann::json &j, LaserScan &p);