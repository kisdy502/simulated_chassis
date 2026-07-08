// Edge.h
#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace agv_bridge
{

    // 控制点结构体
    struct ControlPoint
    {
        double x;
        double y;

        ControlPoint() : x(0.0), y(0.0) {}
        ControlPoint(double x_, double y_) : x(x_), y(y_) {}
    };

    // 边方向枚举
    enum class EdgeDirection
    {
        UNIDIRECTIONAL,
        BIDIRECTIONAL
    };

    // 边类型枚举
    enum class EdgeType
    {
        STRAIGHT,
        CURVE,
        ELEVATION
    };

    // Edge结构体（对应Java的Edge类）
    struct Edge
    {
        std::string id;
        std::string sourceId;
        std::string targetId;
        double weight;           // 距离/成本
        EdgeDirection direction; // 方向
        EdgeType type;           // 类型
        double maxSpeed;         // 最大速度 (m/s)
        int priority;            // 优先级
        bool enabled;            // 是否启用
        double step;             // 内部用的
        bool reverse;            // 内部用的
        bool backUp;            // 是否倒车移动（backUp移动）

        // 贝塞尔曲线控制点（用于CURVE类型的边）
        std::vector<ControlPoint> controlPoints; // 不再使用optional包裹

        // 构造函数
        Edge()
            : weight(0.0),
              direction(EdgeDirection::BIDIRECTIONAL),
              type(EdgeType::STRAIGHT),
              maxSpeed(1.0),
              priority(1),
              enabled(true),
              step(0.1),
              reverse(false),
              backUp(false) {}

        Edge(const std::string &id_,
             const std::string &sourceId_,
             const std::string &targetId_,
             double weight_ = 0.0,
             EdgeDirection direction_ = EdgeDirection::BIDIRECTIONAL,
             EdgeType type_ = EdgeType::STRAIGHT,
             double maxSpeed_ = 1.0,
             int priority_ = 1,
             bool enabled_ = true)
            : id(id_),
              sourceId(sourceId_),
              targetId(targetId_),
              weight(weight_),
              direction(direction_),
              type(type_),
              maxSpeed(maxSpeed_),
              priority(priority_),
              enabled(enabled_),
              step(0.1),
              reverse(false) {}

        // 转换为字符串
        std::string toString() const
        {
            std::string dirStr = (direction == EdgeDirection::UNIDIRECTIONAL) ? "UNIDIRECTIONAL" : "BIDIRECTIONAL";
            std::string typeStr;
            switch (type)
            {
            case EdgeType::STRAIGHT:
                typeStr = "STRAIGHT";
                break;
            case EdgeType::CURVE:
                typeStr = "CURVE";
                break;
            case EdgeType::ELEVATION:
                typeStr = "ELEVATION";
                break;
            }

            return "Edge[id=" + id +
                   ", sourceId=" + sourceId +
                   ", targetId=" + targetId +
                   ", weight=" + std::to_string(weight) +
                   ", direction=" + dirStr +
                   ", type=" + typeStr +
                   ", maxSpeed=" + std::to_string(maxSpeed) +
                   ", priority=" + std::to_string(priority) +
                   ", enabled=" + (enabled ? "true" : "false") +
                   ", step=" + std::to_string(step) +
                   ", reverse=" + (reverse ? "true" : "false") +
                   ", backUp=" + (backUp ? "true" : "false") +
                   ", controlPoints=" + std::to_string(controlPoints.size()) + "]";
        }

        // 检查是否为贝塞尔曲线
        bool isBezierCurve() const
        {
            return type == EdgeType::CURVE && !controlPoints.empty();
        }

        // 获取贝塞尔曲线阶数
        int getBezierOrder() const
        {
            if (type != EdgeType::CURVE)
                return 0;
            return controlPoints.size(); // 1: 二阶, 2: 三阶
        }
    };

} // namespace agv_bridge

#endif // EDGE_H