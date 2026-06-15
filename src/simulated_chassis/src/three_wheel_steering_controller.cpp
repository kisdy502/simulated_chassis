#include "simulated_chassis/three_wheel_steering_controller.hpp"
#include <cmath>

namespace three_wheel_controller
{

    ThreeWheelSteeringController::ThreeWheelSteeringController()
        : wheel_radius_(0.1), chassis_radius_(0.5), odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0) {}

    controller_interface::InterfaceConfiguration
    ThreeWheelSteeringController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "wheel_front_steering_joint/position",
            "wheel_left_steering_joint/position",
            "wheel_right_steering_joint/position",
            "wheel_front_wheel_joint/velocity",
            "wheel_left_wheel_joint/velocity",
            "wheel_right_wheel_joint/velocity",
        };
        return config;
    }

    controller_interface::InterfaceConfiguration
    ThreeWheelSteeringController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "wheel_front_steering_joint/position",
            "wheel_left_steering_joint/position",
            "wheel_right_steering_joint/position",
            "wheel_front_wheel_joint/velocity",
            "wheel_left_wheel_joint/velocity",
            "wheel_right_wheel_joint/velocity",
        };
        return config;
    }

    controller_interface::CallbackReturn
    ThreeWheelSteeringController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    ThreeWheelSteeringController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        auto node = get_node();
        if (!node)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                last_cmd_ = msg;
            });

        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
ThreeWheelSteeringController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  steering_cmds_.clear();
  drive_cmds_.clear();

  auto find_cmd = [&](const std::string & interface_name) -> hardware_interface::LoanedCommandInterface * {
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & iface) {
        // 修正：用 get_name() 获取完整接口名，不是 get_interface_name()
        return iface.get_name() == interface_name;
      });
    if (it != command_interfaces_.end()) {
      return &(*it);
    }
    return nullptr;
  };

  std::vector<std::string> steering_names = {
    "wheel_front_steering_joint/position",
    "wheel_left_steering_joint/position",
    "wheel_right_steering_joint/position",
  };
  for (const auto & name : steering_names) {
    auto * iface = find_cmd(name);
    if (!iface) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    steering_cmds_.push_back(std::ref(*iface));
  }

  std::vector<std::string> drive_names = {
    "wheel_front_wheel_joint/velocity",
    "wheel_left_wheel_joint/velocity",
    "wheel_right_wheel_joint/velocity",
  };
  for (const auto & name : drive_names) {
    auto * iface = find_cmd(name);
    if (!iface) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    drive_cmds_.push_back(std::ref(*iface));
  }

  last_cmd_ = nullptr;
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}


    controller_interface::CallbackReturn
    ThreeWheelSteeringController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        steering_cmds_.clear();
        drive_cmds_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    ThreeWheelSteeringController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        if (last_cmd_)
        {
            vx = last_cmd_->linear.x;
            vy = last_cmd_->linear.y;
            omega = last_cmd_->angular.z;
        }

        double front_angle, left_angle, right_angle;
        double front_speed, left_speed, right_speed;

        computeKinematics(vx, vy, omega,
                          front_angle, left_angle, right_angle,
                          front_speed, left_speed, right_speed);

        // 安全写入：activate 已确保 size==3，这里再检查一次防意外
        if (steering_cmds_.size() == 3)
        {
            steering_cmds_[0].get().set_value(front_angle);
            steering_cmds_[1].get().set_value(left_angle);
            steering_cmds_[2].get().set_value(right_angle);
        }

        if (drive_cmds_.size() == 3)
        {
            drive_cmds_[0].get().set_value(front_speed);
            drive_cmds_[1].get().set_value(left_speed);
            drive_cmds_[2].get().set_value(right_speed);
        }

        // 简化里程计：速度积分
        double dt = period.seconds();
        odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
        odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
        odom_yaw_ += omega * dt;

        auto node = get_node();
        if (odom_pub_ && node)
        {
            odom_msg_.header.stamp = time;
            odom_msg_.header.frame_id = "odom";
            odom_msg_.child_frame_id = "base_link";
            odom_msg_.pose.pose.position.x = odom_x_;
            odom_msg_.pose.pose.position.y = odom_y_;
            odom_msg_.pose.pose.position.z = 0.0;
            odom_msg_.pose.pose.orientation.x = 0.0;
            odom_msg_.pose.pose.orientation.y = 0.0;
            odom_msg_.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
            odom_msg_.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
            odom_msg_.twist.twist.linear.x = vx;
            odom_msg_.twist.twist.linear.y = vy;
            odom_msg_.twist.twist.angular.z = omega;
            odom_pub_->publish(odom_msg_);
        }

        return controller_interface::return_type::OK;
    }

    void ThreeWheelSteeringController::computeKinematics(
        double vx, double vy, double omega,
        double &front_angle, double &left_angle, double &right_angle,
        double &front_speed, double &left_speed, double &right_speed)
    {
        // 120-degree symmetric layout
        const double theta_f = 0.0;
        const double theta_l = 2.0 * M_PI / 3.0;
        const double theta_r = 4.0 * M_PI / 3.0;

        auto wheel_v = [&](double angle, double &v_x, double &v_y)
        {
            v_x = vx - omega * chassis_radius_ * std::sin(angle);
            v_y = vy + omega * chassis_radius_ * std::cos(angle);
        };

        double vfx, vfy, vlx, vly, vrx, vry;
        wheel_v(theta_f, vfx, vfy);
        wheel_v(theta_l, vlx, vly);
        wheel_v(theta_r, vrx, vry);

        front_angle = std::atan2(vfy, vfx);
        left_angle = std::atan2(vly, vlx);
        right_angle = std::atan2(vry, vrx);

        front_speed = std::hypot(vfx, vfy) / wheel_radius_;
        left_speed = std::hypot(vlx, vly) / wheel_radius_;
        right_speed = std::hypot(vrx, vry) / wheel_radius_;
    }

} // namespace three_wheel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    three_wheel_controller::ThreeWheelSteeringController,
    controller_interface::ControllerInterface)
