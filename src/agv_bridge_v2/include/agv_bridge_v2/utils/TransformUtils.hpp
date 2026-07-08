#pragma once
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

namespace agv_bridge
{

    class TransformUtils
    {
    public:
        /**
         * @brief 将几何消息四元数转换为偏航角（弧度）
         * @param q 四元数消息
         * @return 偏航角（弧度）
         */
        static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);

        /**
         * @brief 将四元数的四个分量转换为偏航角（弧度）
         * @param w 四元数 w 分量
         * @param x 四元数 x 分量
         * @param y 四元数 y 分量
         * @param z 四元数 z 分量
         * @return 偏航角（弧度）
         */
        static double quaternion_to_yaw(double w, double x, double y, double z);

        /**
         * @brief 将偏航角（弧度）转换为四元数
         * @param yaw 偏航角（弧度）
         * @return 四元数消息
         */
        static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

        /**
         * @brief 计算两点之间的欧几里得距离
         * @param x1 第一个点的 x 坐标
         * @param y1 第一个点的 y 坐标
         * @param x2 第二个点的 x 坐标
         * @param y2 第二个点的 y 坐标
         * @return 两点之间的距离
         */
        static double calculate_distance(double x1, double y1, double x2, double y2);

        /**
         * @brief 计算从起点到终点的角度（弧度）
         * @param start_x 起点的 x 坐标
         * @param start_y 起点的 y 坐标
         * @param end_x 终点的 x 坐标
         * @param end_y 终点的 y 坐标
         * @return 从起点指向终点的角度（弧度）
         */
        static double calculate_angle(double start_x, double start_y, double end_x, double end_y);

        /**
         * @brief 将角度限制在 [-π, π] 范围内
         * @param angle 输入角度（弧度）
         * @return 规范化后的角度（弧度）
         */
        static double normalize_angle(double angle);

        /**
         * @brief 计算两个角度之间的最小差值
         * @param angle1 第一个角度（弧度）
         * @param angle2 第二个角度（弧度）
         * @return 角度差值（弧度），在 [-π, π] 范围内
         */
        static double angle_difference(double angle1, double angle2);

        /**
         * @brief 将位姿转换到目标坐标系（简化版本）
         * @param pose 原始位姿
         * @param target_frame 目标坐标系
         * @param source_frame 源坐标系
         * @return 转换后的位姿（这里简化实现，实际应用中可能需要TF树）
         */
        static geometry_msgs::msg::PoseStamped transform_pose(
            const geometry_msgs::msg::PoseStamped &pose,
            const std::string &target_frame,
            const std::string &source_frame = "map");

        /**
         * @brief 计算贝塞尔曲线上的点
         * @param t 参数值 [0, 1]
         * @param p0x 起点 x
         * @param p0y 起点 y
         * @param control_x 控制点 x 坐标向量
         * @param control_y 控制点 y 坐标向量
         * @param p1x 终点 x
         * @param p1y 终点 y
         * @return 贝塞尔曲线上的点 (x, y)
         */
        static std::pair<double, double> calculate_bezier_point(
            double t,
            double p0x, double p0y,
            const std::vector<double> &control_x,
            const std::vector<double> &control_y,
            double p1x, double p1y);

        /**
         * @brief 计算贝塞尔曲线在参数 t 处的切线方向
         * @param t 参数值 [0, 1]
         * @param p0x 起点 x
         * @param p0y 起点 y
         * @param control_x 控制点 x 坐标向量
         * @param control_y 控制点 y 坐标向量
         * @param p1x 终点 x
         * @param p1y 终点 y
         * @return 切线方向（弧度）
         */
        static double calculate_bezier_tangent(
            double t,
            double p0x, double p0y,
            const std::vector<double> &control_x,
            const std::vector<double> &control_y,
            double p1x, double p1y);

        /**
         * @brief 计算贝塞尔曲线的近似长度
         * @param p0x 起点 x
         * @param p0y 起点 y
         * @param control_x 控制点 x 坐标向量
         * @param control_y 控制点 y 坐标向量
         * @param p1x 终点 x
         * @param p1y 终点 y
         * @param segments 分段数量
         * @return 近似长度
         */
        static double calculate_bezier_length(
            double p0x, double p0y,
            const std::vector<double> &control_x,
            const std::vector<double> &control_y,
            double p1x, double p1y,
            int segments = 100);
    };

} // namespace agv_bridge
