// Edge.hpp —— 地图「边」的数据结构（导航侧使用，不含任何序列化逻辑）
//
// 上位机通过 FollowEdge action 把这些字段传进来，取代了原先的
// edgeInfo JSON + to_json/from_json 手写映射。
#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <vector>

namespace agv_bridge
{

    // 贝塞尔曲线控制点
    struct ControlPoint
    {
        double x = 0.0;
        double y = 0.0;

        ControlPoint() = default;
        ControlPoint(double x_, double y_) : x(x_), y(y_) {}
    };

    // 边类型
    enum class EdgeType
    {
        STRAIGHT,  // 直线
        CURVE,     // 贝塞尔曲线
        ELEVATION  // 坡道（当前按直线处理）
    };

    // 边：一次移动任务对应的地图拓扑信息
    struct Edge
    {
        std::string id;
        std::string sourceId;  // 边起点节点，用于自动判定行驶方向
        std::string targetId;  // 边终点节点，用于自动判定行驶方向
        EdgeType type = EdgeType::STRAIGHT;
        double maxSpeed = 1.0;  // m/s，会下发到 /speed_limit
        double step = 0.1;      // 路径采样步长 m
        bool reverse = false;   // true = 沿边反向行驶（控制点顺序反转）
        bool backUp = false;    // true = 倒车（保持当前朝向平移后退，绝不旋转）

        // 贝塞尔曲线控制点（type == CURVE 时使用）
        // ⚠️ 仅支持 1 个（二阶）或 2 个（三阶）；更多会退化成直线
        std::vector<ControlPoint> controlPoints;

        /// @brief 是否为可用的贝塞尔曲线（有类型且有控制点）
        bool isBezierCurve() const
        {
            return type == EdgeType::CURVE && !controlPoints.empty();
        }

        /// @brief 贝塞尔阶数：0 = 非曲线，1 = 二阶，2 = 三阶
        int getBezierOrder() const
        {
            if (type != EdgeType::CURVE)
                return 0;
            return static_cast<int>(controlPoints.size());
        }
    };

} // namespace agv_bridge

#endif // EDGE_H
