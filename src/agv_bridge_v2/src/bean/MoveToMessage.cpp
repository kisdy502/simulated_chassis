#include "agv_bridge_v2/bean/MoveToMessage.hpp"

namespace agv_bridge
{

    void to_json(nlohmann::json &j, const ControlPoint &cp)
    {
        j = nlohmann::json{{"x", cp.x}, {"y", cp.y}};
    }

    void from_json(const nlohmann::json &j, ControlPoint &cp)
    {
        j.at("x").get_to(cp.x);
        j.at("y").get_to(cp.y);
    }

    void to_json(nlohmann::json &j, const Edge &edge)
    {
        j = nlohmann::json{
            {"id", edge.id},
            {"sourceId", edge.sourceId},
            {"targetId", edge.targetId},
            {"weight", edge.weight},
            {"direction", edge.direction},
            {"type", edge.type},
            {"maxSpeed", edge.maxSpeed},
            {"priority", edge.priority},
            {"enabled", edge.enabled},
            {"backUp", edge.backUp}};
        j["controlPoints"] = edge.controlPoints;
    }

    void from_json(const nlohmann::json &j, Edge &edge)
    {
        j.at("id").get_to(edge.id);
        j.at("sourceId").get_to(edge.sourceId);
        j.at("targetId").get_to(edge.targetId);
        j.at("weight").get_to(edge.weight);
        j.at("direction").get_to(edge.direction);
        j.at("type").get_to(edge.type);
        j.at("maxSpeed").get_to(edge.maxSpeed);
        j.at("priority").get_to(edge.priority);
        j.at("enabled").get_to(edge.enabled);
        // backUp字段可选,如果不存在则默认为false
        if (j.contains("backUp"))
        {
            j.at("backUp").get_to(edge.backUp);
        }
        else
        {
            edge.backUp = false;
        }
        edge.controlPoints = j.at("controlPoints").get<std::vector<ControlPoint>>();
    }

    void to_json(nlohmann::json &j, const MoveToMessage &msg)
    {
        nlohmann::json base = static_cast<const BaseMessage &>(msg);
        // 然后添加子类字段
        j = base;
        j.update({{"agvId", msg.agvId},
                  {"commandId", msg.commandId},
                  {"nodeId", msg.nodeId},
                  {"x", msg.x},
                  {"y", msg.y},
                  {"theta", msg.theta},
                  {"endPoint", msg.endPoint},
                  {"edgeInfo", msg.edgeInfo}});
    }

    void from_json(const nlohmann::json &j, MoveToMessage &msg)
    {
        static_cast<BaseMessage &>(msg) = j.get<BaseMessage>();
        // 解析派生类字段
        j.at("agvId").get_to(msg.agvId);
        j.at("commandId").get_to(msg.commandId);
        j.at("nodeId").get_to(msg.nodeId);
        j.at("x").get_to(msg.x);
        j.at("y").get_to(msg.y);
        j.at("theta").get_to(msg.theta);
        j.at("endPoint").get_to(msg.endPoint);
        msg.edgeInfo = j.at("edgeInfo").get<Edge>();
    }

} // namespace agv_bridge