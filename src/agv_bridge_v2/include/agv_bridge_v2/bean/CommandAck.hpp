// CommandAck.hpp
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include "agv_bridge_v2/bean/BaseMessage.hpp"
struct CommandAck :public BaseMessage{
    // std::string type;          // command_type
    std::string agv_id;
    std::string command_id;
    std::string node_id;
    std::string status;
    std::string message;
    // std::string timestamp;
};

void to_json(nlohmann::json& j, const CommandAck& p);
void from_json(const nlohmann::json& j, CommandAck& p);