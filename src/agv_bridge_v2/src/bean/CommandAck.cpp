#include "agv_bridge_v2/bean/CommandAck.hpp"

void to_json(nlohmann::json &j, const CommandAck &ack)
{
    nlohmann::json base = static_cast<const BaseMessage &>(ack);
    // 然后添加子类字段
    j = base;
    j.update({
        {"agv_id", ack.agv_id},
        // {"timestamp", ack.timestamp},
        {"command_id", ack.command_id},
        // {"type", ack.type},
        {"node_id", ack.node_id},
        {"status", ack.status},
        {"message", ack.message},

    });
}

void from_json(const nlohmann::json &j, CommandAck &ack)
{
    static_cast<BaseMessage &>(ack) = j.get<BaseMessage>();
    j.at("agv_id").get_to(ack.agv_id);
    // j.at("timestamp").get_to(ack.timestamp);
    j.at("command_id").get_to(ack.command_id);
    // j.at("type").get_to(ack.type);
    j.at("node_id").get_to(ack.node_id);
    j.at("status").get_to(ack.status);
    j.at("message").get_to(ack.message);
}