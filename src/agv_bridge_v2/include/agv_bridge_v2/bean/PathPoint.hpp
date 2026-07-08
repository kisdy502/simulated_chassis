#pragma once
#include <nlohmann/json.hpp>

struct PathPoint
{
    double x;
    double y;
    double theta;  // 朝向（仅在 is_final 为 true 时有效）
    bool is_final; // 是否为终点（需要精确朝向）

    PathPoint(double x_, double y_, double theta_ = 0.0, bool final_ = false)
        : x(x_), y(y_), theta(theta_), is_final(final_) {}
};