// BaseMessage.hpp
#pragma once
#include <string>
#include <nlohmann/json.hpp>

struct BaseMessage
{
    std::string requestId;
    std::string type;
    int64_t timestamp; // 通常由发送方自动填充

    BaseMessage() {}
    virtual ~BaseMessage() = default;
};

void to_json(nlohmann::json &j, const BaseMessage &p);
void from_json(const nlohmann::json &j, BaseMessage &p);