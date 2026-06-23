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
    bool publish_tf_{true};
    std::string odom_frame_id_{"odom"};
    std::string base_frame_id_{"base_link"};

    // // 命令接口缓存（按 wheel_configs_ 顺序）
    // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_cmd_ifaces_;
    // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> drive_cmd_ifaces_;

    // 状态接口缓存
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_state_ifaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> drive_state_ifaces_;

    // 里程计
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_; // 可选

    // 上一周期的舵角（用于最短路径计算）
    std::array<double, 3> prev_steering_angles_{0.0, 0.0, 0.0};
  };

} // namespace three_wheel_controller

#endif // SIMULATED_CHASSIS__THREE_WHEEL_STEERING_CONTROLLER_HPP_


/************************/

#include "simulated_chassis/three_wheel_steering_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace three_wheel_controller
{

  ThreeWheelSteeringController::ThreeWheelSteeringController()
  {
    wheel_configs_ = {
        {"wheel_front_steering_joint", "wheel_front_wheel_joint", 0.3, 0.0, M_PI / 2.0},
        {"wheel_left_steering_joint", "wheel_left_wheel_joint", -0.15, 0.26, M_PI / 2.0},
        {"wheel_right_steering_joint", "wheel_right_wheel_joint", -0.15, -0.26, M_PI / 2.0}};
  }

  controller_interface::InterfaceConfiguration
  ThreeWheelSteeringController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &wheel : wheel_configs_)
    {
      config.names.push_back(wheel.steering_joint_name + "/position");
      config.names.push_back(wheel.wheel_joint_name + "/velocity");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration
  ThreeWheelSteeringController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &wheel : wheel_configs_)
    {
      config.names.push_back(wheel.steering_joint_name + "/position");
      config.names.push_back(wheel.steering_joint_name + "/velocity");
      config.names.push_back(wheel.wheel_joint_name + "/position");
      config.names.push_back(wheel.wheel_joint_name + "/velocity");
    }
    return config;
  }

  controller_interface::CallbackReturn
  ThreeWheelSteeringController::on_init()
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void ThreeWheelSteeringController::declareParameters()
  {
    auto node = get_node();
    if (!node)
      return;

    for (size_t i = 0; i < 3; ++i)
    {
      std::string prefix = "wheel" + std::to_string(i) + ".";
      node->declare_parameter<double>(prefix + "x", wheel_configs_[i].x);
      node->declare_parameter<double>(prefix + "y", wheel_configs_[i].y);
      node->declare_parameter<double>(prefix + "max_steering_angle", wheel_configs_[i].max_steering_angle);
    }

    node->declare_parameter<double>("wheel_radius", wheel_radius_);
    node->declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    node->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
    node->declare_parameter<double>("max_wheel_speed", max_wheel_speed_);
    node->declare_parameter<double>("cmd_timeout", cmd_timeout_);
    node->declare_parameter<bool>("enable_reverse_optimization", enable_reverse_optimization_);
    node->declare_parameter<bool>("publish_tf", publish_tf_);
    node->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
    node->declare_parameter<std::string>("base_frame_id", base_frame_id_);
  }

  controller_interface::CallbackReturn
  ThreeWheelSteeringController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto node = get_node();
    if (!node)
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    declareParameters();

    for (size_t i = 0; i < 3; ++i)
    {
      std::string prefix = "wheel" + std::to_string(i) + ".";
      node->get_parameter(prefix + "x", wheel_configs_[i].x);
      node->get_parameter(prefix + "y", wheel_configs_[i].y);
      node->get_parameter(prefix + "max_steering_angle", wheel_configs_[i].max_steering_angle);
    }
    node->get_parameter("wheel_radius", wheel_radius_);
    node->get_parameter("max_linear_velocity", max_linear_velocity_);
    node->get_parameter("max_angular_velocity", max_angular_velocity_);
    node->get_parameter("max_wheel_speed", max_wheel_speed_);
    node->get_parameter("cmd_timeout", cmd_timeout_);
    node->get_parameter("enable_reverse_optimization", enable_reverse_optimization_);
    node->get_parameter("publish_tf", publish_tf_);
    node->get_parameter("odom_frame_id", odom_frame_id_);
    node->get_parameter("base_frame_id", base_frame_id_);

    if (max_wheel_speed_ < 0.01)
    {
      max_wheel_speed_ = max_linear_velocity_ / wheel_radius_;
    }

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::QoS(10).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          last_cmd_ = msg;
          last_cmd_time_ = get_node()->now();
        });

    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    if (publish_tf_)
    {
      tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }

    last_cmd_time_ = node->now();

    RCLCPP_INFO(node->get_logger(),
                "ThreeWheelSteeringController configured: wheel_radius=%.3f, max_v=%.2f m/s, max_w=%.2f rad/s",
                wheel_radius_, max_linear_velocity_, max_angular_velocity_);
    for (size_t i = 0; i < 3; ++i)
    {
      RCLCPP_INFO(node->get_logger(),
                  "  Wheel[%zu]: (%+.3f, %+.3f), max_steer=%.1f deg",
                  i, wheel_configs_[i].x, wheel_configs_[i].y,
                  wheel_configs_[i].max_steering_angle * 180.0 / M_PI);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  ThreeWheelSteeringController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    steering_cmd_ifaces_.clear();
    drive_cmd_ifaces_.clear();
    steering_state_ifaces_.clear();
    drive_state_ifaces_.clear();

    auto find_cmd = [&](const std::string &interface_name) -> hardware_interface::LoanedCommandInterface *
    {
      auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                             [&](const auto &iface)
                             { return iface.get_name() == interface_name; });
      return (it != command_interfaces_.end()) ? &(*it) : nullptr;
    };

    auto find_state = [&](const std::string &interface_name) -> hardware_interface::LoanedStateInterface *
    {
      auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                             [&](const auto &iface)
                             { return iface.get_name() == interface_name; });
      return (it != state_interfaces_.end()) ? &(*it) : nullptr;
    };

    for (const auto &wheel : wheel_configs_)
    {
      auto *steer = find_cmd(wheel.steering_joint_name + "/position");
      auto *drive = find_cmd(wheel.wheel_joint_name + "/velocity");
      if (!steer || !drive)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface for %s", wheel.steering_joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      steering_cmd_ifaces_.push_back(std::ref(*steer));
      drive_cmd_ifaces_.push_back(std::ref(*drive));
    }

    for (const auto &wheel : wheel_configs_)
    {
      auto *steer_pos = find_state(wheel.steering_joint_name + "/position");
      auto *steer_vel = find_state(wheel.steering_joint_name + "/velocity");
      auto *wheel_pos = find_state(wheel.wheel_joint_name + "/position");
      auto *wheel_vel = find_state(wheel.wheel_joint_name + "/velocity");

      if (!steer_pos || !steer_vel || !wheel_pos || !wheel_vel)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface for %s", wheel.steering_joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      steering_state_ifaces_.push_back(std::ref(*steer_pos));
      steering_state_ifaces_.push_back(std::ref(*steer_vel));
      drive_state_ifaces_.push_back(std::ref(*wheel_pos));
      drive_state_ifaces_.push_back(std::ref(*wheel_vel));
    }

    last_cmd_ = nullptr;
    odom_x_ = odom_y_ = odom_yaw_ = 0.0;
    prev_steering_angles_ = {0.0, 0.0, 0.0};

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  ThreeWheelSteeringController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto &iface : drive_cmd_ifaces_)
    {
      iface.get().set_value(0.0);
    }
    steering_cmd_ifaces_.clear();
    drive_cmd_ifaces_.clear();
    steering_state_ifaces_.clear();
    drive_state_ifaces_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type
  ThreeWheelSteeringController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    const double dt = period.seconds();

    if (!readCurrentWheelStates())
    {
      return controller_interface::return_type::ERROR;
    }

    double vx = 0.0, vy = 0.0, omega = 0.0;
    if (last_cmd_)
    {
      double elapsed = (time - last_cmd_time_).seconds();
      if (elapsed < cmd_timeout_)
      {
        vx = last_cmd_->linear.x;
        vy = last_cmd_->linear.y;
        omega = last_cmd_->angular.z;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Command timeout (%.2f s), stopping", elapsed);
        last_cmd_ = nullptr;
      }
    }

    double v_norm = std::hypot(vx, vy);
    if (v_norm > max_linear_velocity_)
    {
      double scale = max_linear_velocity_ / v_norm;
      vx *= scale;
      vy *= scale;
    }
    omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

    std::array<double, 3> steering_angles{0.0, 0.0, 0.0};
    std::array<double, 3> wheel_speeds{0.0, 0.0, 0.0};

    computeKinematics(vx, vy, omega, steering_angles, wheel_speeds);

    if (enable_reverse_optimization_)
    {
      optimizeReverse(steering_angles, wheel_speeds, prev_steering_angles_);
    }

    for (size_t i = 0; i < 3; ++i)
    {
      steering_angles[i] = normalizeSteeringAngle(prev_steering_angles_[i], steering_angles[i]);
      steering_angles[i] = std::clamp(steering_angles[i],
                                      -wheel_configs_[i].max_steering_angle,
                                      wheel_configs_[i].max_steering_angle);
    }

    limitVelocities(wheel_speeds);

    for (size_t i = 0; i < 3; ++i)
    {
      steering_cmd_ifaces_[i].get().set_value(steering_angles[i]);
      drive_cmd_ifaces_[i].get().set_value(wheel_speeds[i]);
    }

    prev_steering_angles_ = steering_angles;

    publishOdometry(time, vx, vy, omega);

    return controller_interface::return_type::OK;
  }

  void ThreeWheelSteeringController::computeKinematics(
      double vx, double vy, double omega,
      std::array<double, 3> &steering_angles,
      std::array<double, 3> &wheel_speeds)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      const auto &wc = wheel_configs_[i];
      double vxi = vx - omega * wc.y;
      double vyi = vy + omega * wc.x;
      steering_angles[i] = std::atan2(vyi, vxi);
      wheel_speeds[i] = std::hypot(vxi, vyi) / wheel_radius_;
    }
  }

  double ThreeWheelSteeringController::normalizeSteeringAngle(double current, double target) const
  {
    auto normalize_pi = [](double angle)
    {
      while (angle > M_PI)
        angle -= 2.0 * M_PI;
      while (angle < -M_PI)
        angle += 2.0 * M_PI;
      return angle;
    };

    target = normalize_pi(target);
    current = normalize_pi(current);
    double diff = target - current;

    while (diff > M_PI)
    {
      diff -= 2.0 * M_PI;
      target -= 2.0 * M_PI;
    }
    while (diff < -M_PI)
    {
      diff += 2.0 * M_PI;
      target += 2.0 * M_PI;
    }

    return target;
  }

  void ThreeWheelSteeringController::optimizeReverse(
      std::array<double, 3> &steering_angles,
      std::array<double, 3> &wheel_speeds,
      const std::array<double, 3> &current_angles)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      double diff = std::abs(steering_angles[i] - current_angles[i]);
      while (diff > M_PI)
        diff -= 2.0 * M_PI;
      diff = std::abs(diff);

      if (diff > M_PI / 2.0)
      {
        wheel_speeds[i] = -wheel_speeds[i];
        if (steering_angles[i] > 0)
          steering_angles[i] -= M_PI;
        else
          steering_angles[i] += M_PI;
      }
    }
  }

  void ThreeWheelSteeringController::limitVelocities(std::array<double, 3> &wheel_speeds) const
  {
    double max_speed = 0.0;
    for (const auto &s : wheel_speeds)
    {
      max_speed = std::max(max_speed, std::abs(s));
    }

    if (max_speed > max_wheel_speed_ && max_speed > 1e-6)
    {
      double scale = max_wheel_speed_ / max_speed;
      for (auto &s : wheel_speeds)
      {
        s *= scale;
      }
    }
  }

  bool ThreeWheelSteeringController::readCurrentWheelStates()
  {
    if (steering_state_ifaces_.size() < 6 || drive_state_ifaces_.size() < 6)
    {
      return false;
    }

    for (size_t i = 0; i < 3; ++i)
    {
      prev_steering_angles_[i] = steering_state_ifaces_[i * 2].get().get_value();
    }
    return true;
  }

  void ThreeWheelSteeringController::publishOdometry(
      const rclcpp::Time &time, double vx, double vy, double omega)
  {
    static rclcpp::Time last_time = time;
    double dt = (time - last_time).seconds();
    last_time = time;

    if (dt <= 0 || dt > 1.0)
    {
      dt = 0.01;
    }

    odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
    odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
    odom_yaw_ += omega * dt;

    while (odom_yaw_ > M_PI)
      odom_yaw_ -= 2.0 * M_PI;
    while (odom_yaw_ < -M_PI)
      odom_yaw_ += 2.0 * M_PI;

    odom_msg_.header.stamp = time;
    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = base_frame_id_;

    odom_msg_.pose.pose.position.x = odom_x_;
    odom_msg_.pose.pose.position.y = odom_y_;
    odom_msg_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);

    odom_msg_.twist.twist.linear.x = vx;
    odom_msg_.twist.twist.linear.y = vy;
    odom_msg_.twist.twist.angular.z = omega;

    odom_pub_->publish(odom_msg_);

    if (publish_tf_ && tf_pub_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom_msg_.header;
      tf.child_frame_id = odom_msg_.child_frame_id;
      tf.transform.translation.x = odom_x_;
      tf.transform.translation.y = odom_y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom_msg_.pose.pose.orientation;

      tf2_msgs::msg::TFMessage tf_msg;
      tf_msg.transforms.push_back(tf);
      tf_pub_->publish(tf_msg);
    }
  }

} // namespace three_wheel_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    three_wheel_controller::ThreeWheelSteeringController,
    controller_interface::ControllerInterface)
