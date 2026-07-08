#include "agv_bridge_v2/bean/StatusUpdate.hpp"

void to_json(nlohmann::json &j, const StatusUpdate &s)
{
    nlohmann::json base = static_cast<const BaseMessage &>(s);
    // 然后添加子类字段
    j = base;
    j.update({{"agv_id", s.agv_id},
              // {"timestamp", s.timestamp},
              {"state", s.state},
              {"battery", s.battery},
              {"pose_initialized", s.pose_initialized}});
}

void from_json(const nlohmann::json &j, StatusUpdate &s)
{
    // 先调用基类的 from_json 解析基类字段
    static_cast<BaseMessage &>(s) = j.get<BaseMessage>();
    j.at("agv_id").get_to(s.agv_id);
    // j.at("timestamp").get_to(s.timestamp);
    j.at("state").get_to(s.state);
    j.at("battery").get_to(s.battery);
    j.at("pose_initialized").get_to(s.pose_initialized);
}