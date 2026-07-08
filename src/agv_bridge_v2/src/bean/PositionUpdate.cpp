#include "agv_bridge_v2/bean/PositionUpdate.hpp"

void to_json(nlohmann::json &j, const PositionUpdate &p)
{
    // 先序列化基类字段
    nlohmann::json base = static_cast<const BaseMessage &>(p);
    
    // 然后将基类字段和子类字段合并到一个扁平JSON对象中
    j = base;
    
    // 直接添加所有字段到顶层，不创建嵌套对象
    j.update({
        {"agv_id", p.agv_id},
        {"x", p.x},
        {"y", p.y},
        {"qx", p.qx},
        {"qy", p.qy},
        {"qz", p.qz},
        {"qw", p.qw},
        {"theta", p.theta},
        {"vx", p.vx},
        {"vy", p.vy},
        {"omega", p.omega},
        {"timestamp_ns", p.timestamp_ns}
    });
}

void from_json(const nlohmann::json &j, PositionUpdate &p)
{
    // 先解析基类字段
    static_cast<BaseMessage &>(p) = j.get<BaseMessage>();
    
    // 直接从顶层解析所有字段
    j.at("agv_id").get_to(p.agv_id);
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("qx").get_to(p.qx);
    j.at("qy").get_to(p.qy);
    j.at("qz").get_to(p.qz);
    j.at("qw").get_to(p.qw);
    j.at("theta").get_to(p.theta);
    j.at("vx").get_to(p.vx);
    j.at("vy").get_to(p.vy);
    j.at("omega").get_to(p.omega);
    j.at("timestamp_ns").get_to(p.timestamp_ns);
}