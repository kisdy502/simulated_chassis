#pragma once

#include <cmath>
#include <cctype>
#include <string>

namespace agv_bridge {
inline bool blankNavigationField(const std::string &value) {
    for (unsigned char c : value) if (!std::isspace(c)) return false;
    return true;
}

// Shared by the ROS action boundary and manager entry points.
template<typename Points>
std::string validateNavigationMode(const std::string &type, const std::string &edge_id,
                                  const Points &points, bool back_up, bool reverse) {
    if (blankNavigationField(type))
        return "参数错误：edge_type 不能为空或仅含空白；请明确填写 STRAIGHT、CURVE、ELEVATION 或 FREE，不再根据控制点推断";
    if (type != "STRAIGHT" && type != "CURVE" && type != "ELEVATION" && type != "FREE")
        return "参数错误：edge_type='" + type + "' 不受支持；请填写 STRAIGHT、CURVE、ELEVATION 或 FREE（区分大小写）";
    if (type != "FREE" && blankNavigationField(edge_id))
        return "参数错误：" + type + " 的 edge_id 不能为空或仅含空白；请填写有效的边 ID";
    if (type == "CURVE") {
        if (points.size() != 1 && points.size() != 2)
            return "参数错误：CURVE 的 control_points 数量为 " + std::to_string(points.size()) +
                   "；必须提供 1 个（二阶）或 2 个（三阶）控制点，不会退化为直线";
    } else if (!points.empty()) {
        return "参数错误：" + type + " 的 control_points 数量为 " + std::to_string(points.size()) +
               "；该模式必须省略控制点或传 []";
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!std::isfinite(points[i].x))
            return "参数错误：control_points[" + std::to_string(i) + "].x 不是有限数值；请填写有效的 map 坐标（米）";
        if (!std::isfinite(points[i].y))
            return "参数错误：control_points[" + std::to_string(i) + "].y 不是有限数值；请填写有效的 map 坐标（米）";
    }
    if (type == "FREE" && back_up)
        return "参数错误：FREE 的 back_up=true；自由导航不支持保持朝向倒车，请传 false 或改用 STRAIGHT 倒车";
    if (type == "FREE" && reverse)
        return "参数错误：FREE 的 reverse=true；自由导航不支持沿边反向，请传 false";
    return {};
}
// Transport acceptance is separate from business admission; callers return
// these errors in FollowEdge::Result without replacing the active task.
inline std::string navigationAdmissionError(bool busy, const std::string &command,
    const std::string &stage, bool stopped, const std::string &mode, const std::string &map)
{
    if (busy) return "导航请求未执行：已有任务占用，command_id=" + command +
        "，阶段=" + stage + "；请等待当前任务结束或取消并确认停止后再下发";
    if (stopped) return "导航请求未执行：机器人处于 stop 状态；请通过 /agv/set_control 恢复后再下发";
    if (mode != "NAVIGATION") return "导航请求未执行：当前 mode=" + mode +
        "，map=" + map + "；请等待建图或重定位完成并进入 NAVIGATION 模式";
    return {};
}
} // namespace agv_bridge
