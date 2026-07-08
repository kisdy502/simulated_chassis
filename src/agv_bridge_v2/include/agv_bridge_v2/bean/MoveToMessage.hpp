#ifndef AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP
#define AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>
#include <iostream>
#include "agv_bridge_v2/bean/BaseMessage.hpp"
#include "agv_bridge_v2/bean/Edge.hpp"

namespace agv_bridge
{
    // 移动命令消息（对应Java MoveToMessage）
    struct MoveToMessage : public BaseMessage
    {
        std::string agvId;
        std::string commandId;
        std::string nodeId;
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        Edge edgeInfo;         // 可能为null
        bool endPoint = false; // 对应JSON字段 "endPoint"
    };

    // ----- JSON序列化函数声明 -----

    void to_json(nlohmann::json &j, const ControlPoint &cp);
    void from_json(const nlohmann::json &j, ControlPoint &cp);

    void to_json(nlohmann::json &j, const Edge &edge);
    void from_json(const nlohmann::json &j, Edge &edge);

    void to_json(nlohmann::json &j, const MoveToMessage &msg);
    void from_json(const nlohmann::json &j, MoveToMessage &msg);

} // namespace agv_bridge

// 枚举的JSON序列化宏
namespace nlohmann
{
    template <>
    struct adl_serializer<agv_bridge::EdgeDirection>
    {
        static void to_json(json &j, const agv_bridge::EdgeDirection &dir)
        {
            switch (dir)
            {
            case agv_bridge::EdgeDirection::UNIDIRECTIONAL:
                j = "UNIDIRECTIONAL";
                break;
            case agv_bridge::EdgeDirection::BIDIRECTIONAL:
                j = "BIDIRECTIONAL";
                break;
            default:
                j = nullptr;
            }
        }
        static void from_json(const json &j, agv_bridge::EdgeDirection &dir)
        {
            auto s = j.get<std::string>();
            if (s == "UNIDIRECTIONAL")
                dir = agv_bridge::EdgeDirection::UNIDIRECTIONAL;
            else if (s == "BIDIRECTIONAL")
                dir = agv_bridge::EdgeDirection::BIDIRECTIONAL;
            else
                throw std::runtime_error("Invalid EdgeDirection: " + s);
        }
    };

    template <>
    struct adl_serializer<agv_bridge::EdgeType>
    {
        static void to_json(json &j, const agv_bridge::EdgeType &type)
        {
            switch (type)
            {
            case agv_bridge::EdgeType::STRAIGHT:
                j = "STRAIGHT";
                break;
            case agv_bridge::EdgeType::CURVE:
                j = "CURVE";
                break;
            case agv_bridge::EdgeType::ELEVATION:
                j = "ELEVATION";
                break;
            default:
                j = nullptr;
            }
        }
        static void from_json(const json &j, agv_bridge::EdgeType &type)
        {
            auto s = j.get<std::string>();
            if (s == "STRAIGHT")
                type = agv_bridge::EdgeType::STRAIGHT;
            else if (s == "CURVE")
                type = agv_bridge::EdgeType::CURVE;
            else if (s == "ELEVATION")
                type = agv_bridge::EdgeType::ELEVATION;
            else
                throw std::runtime_error("Invalid EdgeType: " + s);
        }
    };
} // namespace nlohmann

#endif // AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP