#include "agv_bridge_v2/utils/TransformUtils.hpp"
#include <stdexcept>
#include <cmath>

namespace agv_bridge
{

double TransformUtils::quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    try
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    catch (const std::exception &e)
    {
        // 如果转换失败，使用数学公式计算
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
}

double TransformUtils::quaternion_to_yaw(double w, double x, double y, double z)
{
    // 使用atan2公式计算yaw角
    // yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion TransformUtils::yaw_to_quaternion(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    return q;
}

double TransformUtils::calculate_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double TransformUtils::calculate_angle(double start_x, double start_y, double end_x, double end_y)
{
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    return std::atan2(dy, dx);
}

double TransformUtils::normalize_angle(double angle)
{
    // 将角度限制在 [-π, π] 范围内
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double TransformUtils::angle_difference(double angle1, double angle2)
{
    // 计算两个角度之间的最小差值
    double diff = normalize_angle(angle1) - normalize_angle(angle2);
    return normalize_angle(diff);
}

geometry_msgs::msg::PoseStamped TransformUtils::transform_pose(
    const geometry_msgs::msg::PoseStamped &pose,
    const std::string &target_frame,
    const std::string &source_frame)
{
    // 注意：这是一个简化版本，实际应用中应该使用TF树进行坐标转换
    // 这里只是返回原始位姿，实际项目需要实现TF转换逻辑
    geometry_msgs::msg::PoseStamped transformed_pose = pose;
    transformed_pose.header.frame_id = target_frame;
    
    // 简化实现：假设源坐标系和目标坐标系相同
    // 实际项目中应该使用tf2::doTransform()
    // throw std::runtime_error("transform_pose not implemented with TF");
    
    return transformed_pose;
}

std::pair<double, double> TransformUtils::calculate_bezier_point(
    double t,
    double p0x, double p0y,
    const std::vector<double> &control_x,
    const std::vector<double> &control_y,
    double p1x, double p1y)
{
    if (control_x.empty() || control_y.empty())
    {
        // 直线插值
        double x = p0x + t * (p1x - p0x);
        double y = p0y + t * (p1y - p0y);
        return {x, y};
    }

    if (control_x.size() == 1)
    {
        // 二阶贝塞尔曲线
        double u = 1.0 - t;
        double x = u * u * p0x + 2.0 * u * t * control_x[0] + t * t * p1x;
        double y = u * u * p0y + 2.0 * u * t * control_y[0] + t * t * p1y;
        return {x, y};
    }
    else if (control_x.size() == 2)
    {
        // 三阶贝塞尔曲线
        double u = 1.0 - t;
        double u2 = u * u;
        double t2 = t * t;
        double x = u2 * u * p0x + 3.0 * u2 * t * control_x[0] + 3.0 * u * t2 * control_x[1] + t2 * t * p1x;
        double y = u2 * u * p0y + 3.0 * u2 * t * control_y[0] + 3.0 * u * t2 * control_y[1] + t2 * t * p1y;
        return {x, y};
    }
    else
    {
        // 不支持更高阶贝塞尔曲线
        throw std::invalid_argument("只支持二阶或三阶贝塞尔曲线");
    }
}

double TransformUtils::calculate_bezier_tangent(
    double t,
    double p0x, double p0y,
    const std::vector<double> &control_x,
    const std::vector<double> &control_y,
    double p1x, double p1y)
{
    if (control_x.empty() || control_y.empty())
    {
        // 直线方向
        double dx = p1x - p0x;
        double dy = p1y - p0y;
        return std::atan2(dy, dx);
    }

    if (control_x.size() == 1)
    {
        // 二阶贝塞尔曲线导数
        // P(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
        // P'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁)
        double dx = 2.0 * (1.0 - t) * (control_x[0] - p0x) + 2.0 * t * (p1x - control_x[0]);
        double dy = 2.0 * (1.0 - t) * (control_y[0] - p0y) + 2.0 * t * (p1y - control_y[0]);
        
        // 避免除零错误
        if (std::abs(dx) < 1e-10 && std::abs(dy) < 1e-10)
        {
            // 使用端点方向
            dx = p1x - p0x;
            dy = p1y - p0y;
        }
        
        return std::atan2(dy, dx);
    }
    else if (control_x.size() == 2)
    {
        // 三阶贝塞尔曲线导数
        // P(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
        // P'(t) = 3(1-t)²(P₁-P₀) + 6(1-t)t(P₂-P₁) + 3t²(P₃-P₂)
        double u = 1.0 - t;
        double dx = 3.0 * u * u * (control_x[0] - p0x) + 
                    6.0 * u * t * (control_x[1] - control_x[0]) + 
                    3.0 * t * t * (p1x - control_x[1]);
        double dy = 3.0 * u * u * (control_y[0] - p0y) + 
                    6.0 * u * t * (control_y[1] - control_y[0]) + 
                    3.0 * t * t * (p1y - control_y[1]);
        
        // 避免除零错误
        if (std::abs(dx) < 1e-10 && std::abs(dy) < 1e-10)
        {
            // 使用端点方向
            dx = p1x - p0x;
            dy = p1y - p0y;
        }
        
        return std::atan2(dy, dx);
    }
    else
    {
        // 不支持更高阶贝塞尔曲线
        throw std::invalid_argument("只支持二阶或三阶贝塞尔曲线");
    }
}

double TransformUtils::calculate_bezier_length(
    double p0x, double p0y,
    const std::vector<double> &control_x,
    const std::vector<double> &control_y,
    double p1x, double p1y,
    int segments)
{
    if (segments < 2)
    {
        segments = 2;
    }
    
    double length = 0.0;
    double prev_x = p0x;
    double prev_y = p0y;
    
    for (int i = 1; i <= segments; ++i)
    {
        double t = static_cast<double>(i) / segments;
        auto [x, y] = calculate_bezier_point(t, p0x, p0y, control_x, control_y, p1x, p1y);
        
        double dx = x - prev_x;
        double dy = y - prev_y;
        length += std::sqrt(dx * dx + dy * dy);
        
        prev_x = x;
        prev_y = y;
    }
    
    return length;
}

} // namespace agv_bridge