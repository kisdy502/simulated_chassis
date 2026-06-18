#ifndef SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
#define SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

namespace three_wheel_controller
{

  class ThreeWheelSteeringController : public controller_interface::ControllerInterface
  {
  public:
    ThreeWheelSteeringController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  protected:
    void computeKinematics(double vx, double vy, double omega,
                           double &front_angle, double &left_angle, double &right_angle,
                           double &front_speed, double &left_speed, double &right_speed);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    geometry_msgs::msg::Twist::SharedPtr last_cmd_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_cmds_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmds_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;

    double wheel_radius_;
    double chassis_radius_;
    double odom_x_, odom_y_, odom_yaw_;
    rclcpp::Time last_cmd_time_;
    static constexpr double CMD_TIMEOUT = 0.5;  // 0.5秒超时
  };

} // namespace three_wheel_controller

#endif // SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
