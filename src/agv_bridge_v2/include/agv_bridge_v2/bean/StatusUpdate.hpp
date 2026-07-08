// StatusUpdate.hpp
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "agv_bridge_v2/bean/BaseMessage.hpp"

struct StatusUpdate : public BaseMessage
{
    std::string agv_id;
    // std::string timestamp;
    std::string state;
    double battery = 0.0;
    bool pose_initialized = false;
};

void to_json(nlohmann::json &j, const StatusUpdate &p);
void from_json(const nlohmann::json &j, StatusUpdate &p);