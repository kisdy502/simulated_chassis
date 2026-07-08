// LaserScan.cpp
#include "agv_bridge_v2/bean/BaseMessage.hpp"

void to_json(nlohmann::json &j, const BaseMessage &p)
{
    j = nlohmann::json{
        {"type", p.type},
        {"timestamp", p.timestamp},
        {"requestId", p.requestId},

    };
}

void from_json(const nlohmann::json &j, BaseMessage &p)
{
    j.at("type").get_to(p.type);
    j.at("timestamp").get_to(p.timestamp);
    j.at("requestId").get_to(p.requestId);
}