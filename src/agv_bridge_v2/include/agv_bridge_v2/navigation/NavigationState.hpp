#pragma once
#include <string>
namespace agv_bridge {
    // 导航状态枚举,其中PRE_ROTATING =移动前，先旋转到目标角度，END_ROTATING，移动结束后，旋转到目标点
    enum class NavigationState
    {
        IDLE,
        PRE_ROTATING,
        EXECUTING,
        END_ROTATING,
        BACKING_UP,  // 🔴 新增:倒车状态
        COMPLETED,
        FAILED,
        CANCELLED
    };

    inline std::string navigationStateToString(NavigationState state)
    {
        switch (state)
        {
        case NavigationState::IDLE:
            return "IDLE";
        case NavigationState::PRE_ROTATING:
            return "PRE_ROTATING";
        case NavigationState::EXECUTING:
            return "EXECUTING";
        case NavigationState::END_ROTATING:
            return "END_ROTATING";
        case NavigationState::BACKING_UP:
            return "BACKING_UP";
        case NavigationState::COMPLETED:
            return "COMPLETED";
        case NavigationState::FAILED:
            return "FAILED";
        case NavigationState::CANCELLED:
            return "CANCELLED";
        default:
            return "UNKNOWN(" + std::to_string(static_cast<int>(state)) + ")";
        }
    }

} // namespace agv_bridge
