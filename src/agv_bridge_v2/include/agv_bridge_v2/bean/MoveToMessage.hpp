// MoveToMessage.hpp —— 一次「沿边移动」任务的内部表示
//
// 由 AgvNavServerNode 从 FollowEdge action goal 转换而来，供 NavigationManager 使用。
// 不再继承 BaseMessage，也不再做 JSON 序列化（对外契约由 .action/.msg IDL 定义）。
#ifndef AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP
#define AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP

#include <string>

#include "agv_bridge_v2/bean/Edge.hpp"

namespace agv_bridge
{
    struct MoveToMessage
    {
        std::string commandId;  // 上位机指令 ID，原样回传
        std::string nodeId;     // 目标节点 ID，原样回传

        // 目标点在 map 坐标系下的位姿
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        Edge edgeInfo;

        // true = 到达后还需按 theta 做最终旋转（见 NavigationManager::followPathResultCallback）
        bool endPoint = false;
    };

} // namespace agv_bridge

#endif // AGV_BRIDGE_V2_BEAN_MOVE_TO_MESSAGE_HPP
