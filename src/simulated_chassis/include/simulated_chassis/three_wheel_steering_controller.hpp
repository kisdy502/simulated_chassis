#ifndef SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
#define SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <memory>
#include <array>
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

    // 速度限制与饱和
    void limitVelocities(std::array<double, 3> &wheel_speeds) const;

    // 有界等价解：在 (α, v) 与 (α±π, -v) 中选择可达且转角最小的解
    void optimizeReverse(std::array<double, 3> &steering_angles,
                         std::array<double, 3> &wheel_speeds,
                         const std::array<double, 3> &current_angles);

    // 舵轮未对齐期间抑制驱动轮速，避免轮子横向拖拽底盘；返回 true=门开（允许驱动）
    bool scaleWheelSpeedsForSteeringAlignment(
        const std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds,
        const std::array<double, 3> &current_angles) const;

    // 调试日志：指令/逆解原始解/最终命令/实际状态/对齐门/里程计估计。
    // 对齐门开关是边沿触发（发生即打），全量状态按 debug_log_period 周期打印。
    void logDebugState(const rclcpp::Time &time,
                       double vx, double vy, double omega,
                       const std::array<double, 3> &raw_angles,
                       const std::array<double, 3> &target_angles,
                       const std::array<double, 3> &target_speeds,
                       bool gate_open);

    // 发布里程计（含TF）
    void publishOdometry(const rclcpp::Time &time, double vx, double vy, double omega);

    // 读取当前关节状态
    bool readCurrentWheelStates();

    // 读取最新底盘指令，处理超时并限制速度。
    void getLimitedCommand(const rclcpp::Time &time,
                           double &vx, double &vy, double &omega);

    // 向 ros2_control 命令接口写入舵角和驱动轮速。
    void writeWheelCommands(const std::array<double, 3> &steering_angles,
                            const std::array<double, 3> &wheel_speeds);

    // 用真实关节状态反算底盘速度，积分并发布里程计。
    void updateOdometryFromWheelStates(const rclcpp::Time &time,
                                       const rclcpp::Duration &period);

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
    double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};

    double max_linear_velocity_{1.5};        // m/s
    double max_angular_velocity_{1.0};       // rad/s
    double max_wheel_speed_{15.0};           // rad/s (约1.5m/s / 0.1m)
    double cmd_timeout_{0.8};                // s
    bool enable_reverse_optimization_{true}; // 是否启用后退优化
    double steering_hold_velocity_threshold_{0.01}; // 轮心线速度低于此值时保持当前舵角 (m/s)
    double alignment_full_speed_angle_{0.174532925}; // 舵角误差 <= 10° 时才允许驱动
    double creep_wheel_speed_{0.5};                  // 对齐期间轮子蠕动转速 rad/s，0=恢复完全停车（会死锁）
    double reverse_switch_hysteresis_{0.087266463};  // 切换轮速方向的 5° 惩罚
    bool publish_tf_{false};
    std::string odom_frame_id_{"odom"};
    std::string base_frame_id_{"base_link"};

    // 状态接口缓存
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_state_ifaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> drive_state_ifaces_;

    // 里程计
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_; // 可选

    // 实际舵角只由 state interface 更新，不得被目标命令覆盖。
    std::array<double, 3> actual_steering_angles_{0.0, 0.0, 0.0};
    // 实际驱动轮速同样只来自 state interface。
    std::array<double, 3> actual_wheel_velocities_{0.0, 0.0, 0.0};
    // 上一周期下发的目标舵角，仅用于调试/状态记录。
    std::array<double, 3> commanded_steering_angles_{0.0, 0.0, 0.0};
    std::array<int, 3> selected_drive_directions_{1, 1, 1};

    // ========== 调试日志状态 ==========
    double debug_log_period_{0.0};                              // 全量状态日志周期(s)，0=关闭
    rclcpp::Time last_debug_log_time_{0, 0, RCL_ROS_TIME};
    bool gate_open_last_{true};                                 // 对齐门上一周期状态（边沿日志用）
    std::array<bool, 3> steering_saturated_{false, false, false}; // 本周期舵角被限位滞回吸收
    double last_est_vx_{0.0}, last_est_vy_{0.0}, last_est_wz_{0.0}; // 里程计反算的最新底盘速度

  };

} // namespace three_wheel_controller

#endif // SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_
