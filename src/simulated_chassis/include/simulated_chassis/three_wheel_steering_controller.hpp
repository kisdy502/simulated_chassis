// three_wheel_steering_controller.hpp
#ifndef THREE_WHEEL_STEERING_CONTROLLER_HPP
#define THREE_WHEEL_STEERING_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <cmath>

namespace three_wheel_controller
{

    struct WheelConfig
    {
        double x;     // 轮中心X位置
        double y;     // 轮中心Y位置
        double angle; // 安装角度（底盘坐标系）
    };

    class ThreeWheelSteeringController : public controller_interface::ControllerInterface
    {
    public:
        ThreeWheelSteeringController();

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // 订阅 cmd_vel
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        geometry_msgs::msg::Twist cmd_vel_;
        std::mutex cmd_vel_mutex_;

        // 发布 odom
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // 关节接口
        std::vector<std::string> steering_joint_names_;
        std::vector<std::string> drive_joint_names_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_cmds_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmds_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_states_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> drive_states_;

        // 配置
        std::array<WheelConfig, 3> wheels_;
        double wheel_radius_;
        double max_steering_speed_;
        double max_wheel_speed_;
        bool allow_reverse_steering_;

        // 里程计状态
        double odom_x_, odom_y_, odom_theta_;
        rclcpp::Time last_odom_time_;

        // 运动学计算
        void computeWheelCommands(double vx, double vy, double omega,
                                  std::array<double, 3> &steering_angles,
                                  std::array<double, 3> &wheel_speeds);

        void updateOdometry(const rclcpp::Duration &period);

        // 角度归一化
        static double normalizeAngle(double angle);

        // 最优转向：选择转小角度还是转大角度+反转轮速
        static double optimizeSteering(double current, double target, bool &reverse);
    };

} // namespace three_wheel_controller

#endif