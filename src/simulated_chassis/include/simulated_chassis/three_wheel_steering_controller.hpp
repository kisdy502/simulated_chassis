#ifndef SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
#define SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

namespace three_wheel_controller
{

  struct WheelConfig
  {
    std::string steering_joint_name;
    std::string wheel_joint_name;
    double x;                  // 轮心相对于base_link的X坐标 (m)
    double y;                  // 轮心相对于base_link的Y坐标 (m)
    double max_steering_angle; // 最大转向角 (rad)，默认 π/2
  };

  struct WheelState
  {
    double steering_angle{0.0}; // 当前舵角 (rad)
    double wheel_velocity{0.0}; // 当前轮速 (rad/s)
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

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  protected:
    // 核心运动学：基于实际轮心坐标（长方形布局）
    void computeKinematics(double vx, double vy, double omega,
                           std::array<double, 3> &steering_angles,
                           std::array<double, 3> &wheel_speeds);

    // 前向运动学：从实际舵角+轮速反算底盘速度（用于里程计闭环）
    void computeForwardKinematics(const std::array<double, 3> &steering_angles,
                                  const std::array<double, 3> &wheel_velocities,
                                  double &vx, double &vy, double &omega);

    // 舵角最短路径归一化：将目标角度映射到 [-π, π] 并选择最短旋转方向
    double normalizeSteeringAngle(double current, double target) const;

    // 速度限制与饱和
    void limitVelocities(std::array<double, 3> &wheel_speeds) const;

    // 后退优化：优先反转轮速而非旋转舵轮180°
    void optimizeReverse(std::array<double, 3> &steering_angles,
                         std::array<double, 3> &wheel_speeds,
                         const std::array<double, 3> &current_angles);

    // 发布里程计（含TF）
    void publishOdometry(const rclcpp::Time &time, double vx, double vy, double omega);

    // 读取当前关节状态
    bool readCurrentWheelStates();

    // 参数声明
    void declareParameters();

    // ========== 运行时状态 ==========
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    geometry_msgs::msg::Twist::SharedPtr last_cmd_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME}; // 防止重复时间戳

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_cmds_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmds_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;

    std::vector<WheelConfig> wheel_configs_;
    double wheel_radius_;
    double chassis_radius_;
    double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};
    static constexpr double CMD_TIMEOUT = 0.5; // 0.5秒超时

    double max_linear_velocity_{1.5};        // m/s
    double max_angular_velocity_{1.0};       // rad/s
    double max_wheel_speed_{15.0};           // rad/s (约1.5m/s / 0.1m)
    double cmd_timeout_{0.5};                // s
    bool enable_reverse_optimization_{true}; // 是否启用后退优化
    bool publish_tf_{false};
    std::string odom_frame_id_{"odom"};
    std::string base_frame_id_{"base_link"};

    // 状态接口缓存
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_state_ifaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> drive_state_ifaces_;

    // 里程计
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_; // 可选

    // 上一周期的舵角（用于最短路径计算）
    std::array<double, 3> prev_steering_angles_{0.0, 0.0, 0.0};

    // 协方差矩阵 [x, y, z, roll, pitch, yaw]
    // 注意：z, roll, pitch 是平面机器人不可观测量，设为极大值
    double pose_covariance_[6] = {
        0.01,    // x: 位置初始不确定度 0.01 m² (10cm)
        0.01,    // y: 位置初始不确定度 0.01 m² (10cm)
        99999.0, // z: 不可观，极大值
        99999.0, // roll: 不可观，极大值
        99999.0, // pitch: 不可观，极大值
        0.01     // yaw: 角度初始不确定度 0.01 rad² (~5.7°)
    };

    double twist_covariance_[6] = {
        0.1,     // vx: 速度初始不确定度 0.1 (m/s)²
        0.1,     // vy: 速度初始不确定度 0.1 (m/s)²
        99999.0, // vz: 不可观
        99999.0, // vroll: 不可观
        99999.0, // vpitch: 不可观
        0.1      // omega: 角速度初始不确定度 0.1 (rad/s)²
    };
    // x-y 协方差耦合项（独立变量）
    double xy_coupling_ = 0.0;
  };

} // namespace three_wheel_controller

#endif // SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
