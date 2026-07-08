// LaserScan.cpp
#include "agv_bridge_v2/bean/LaserScan.hpp"

void to_json(nlohmann::json &j, const LaserScan &p)
{
    nlohmann::json base = static_cast<const BaseMessage &>(p);
    // 然后添加子类字段
    j = base;

    j.update({
        {"agv_id", p.agv_id},
        // {"timestamp", p.timestamp},
        {"range_min", p.range_min},
        {"range_max", p.range_max},
        {"angle_min", p.angle_min},
        {"angle_max", p.angle_max},
        {"ranges", p.ranges},
        {"pose_x", p.pose_x},
        {"pose_y", p.pose_y},
        {"pose_theta", p.pose_theta},
    });
}

void from_json(const nlohmann::json &j, LaserScan &p)
{
    // 先调用基类的 from_json 解析基类字段
    static_cast<BaseMessage &>(p) = j.get<BaseMessage>();
    j.at("agv_id").get_to(p.agv_id);
    // j.at("timestamp").get_to(p.timestamp);
    j.at("range_min").get_to(p.range_min);
    j.at("range_max").get_to(p.range_max);
    j.at("angle_min").get_to(p.angle_min);
    j.at("angle_max").get_to(p.angle_max);
    j.at("ranges").get_to(p.ranges);
    j.at("pose_x").get_to(p.pose_x);
    j.at("pose_y").get_to(p.pose_y);
    j.at("pose_theta").get_to(p.pose_theta);
}