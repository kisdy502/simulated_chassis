
#include "simulated_chassis/three_wheel_steering_controller.hpp"
#include <cmath>

namespace three_wheel_controller
{

    ThreeWheelSteeringController::ThreeWheelSteeringController()
    {
        // 默认轮位配置：基于你的URDF坐标
        // 前轮: (0.3, 0), 左后轮: (-0.15, 0.26), 右后轮: (-0.15, -0.26)
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

    void ThreeWheelSteeringController::declareParameters()
    {
        auto node = get_node();
        if (!node)
            return;

        for (size_t i = 0; i < 3; ++i)
        {
            std::string prefix = "wheel" + std::to_string(i) + ".";
            if (!node->has_parameter(prefix + "x"))
                node->declare_parameter<double>(prefix + "x", wheel_configs_[i].x);
            if (!node->has_parameter(prefix + "y"))
                node->declare_parameter<double>(prefix + "y", wheel_configs_[i].y);
            if (!node->has_parameter(prefix + "max_steering_angle"))
                node->declare_parameter<double>(prefix + "max_steering_angle", wheel_configs_[i].max_steering_angle);
        }

        if (!node->has_parameter("wheel_radius"))
            node->declare_parameter<double>("wheel_radius", wheel_radius_);
        if (!node->has_parameter("max_linear_velocity"))
            node->declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        if (!node->has_parameter("max_angular_velocity"))
            node->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
        if (!node->has_parameter("max_wheel_speed"))
            node->declare_parameter<double>("max_wheel_speed", max_wheel_speed_);
        if (!node->has_parameter("cmd_timeout"))
            node->declare_parameter<double>("cmd_timeout", cmd_timeout_);
        if (!node->has_parameter("enable_reverse_optimization"))
            node->declare_parameter<bool>("enable_reverse_optimization", enable_reverse_optimization_);
        if (!node->has_parameter("publish_tf"))
            node->declare_parameter<bool>("publish_tf", publish_tf_);
        if (!node->has_parameter("odom_frame_id"))
            node->declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
        if (!node->has_parameter("base_frame_id"))
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
        // 读取参数
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

        // 根据最大线速度重新计算轮速上限（如果用户没指定）
        if (max_wheel_speed_ < 0.01)
        {
            max_wheel_speed_ = max_linear_velocity_ / wheel_radius_;
        }

        cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", 10,
            [this, node](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                last_cmd_ = msg;
                last_cmd_time_ = node->now(); // 记录收到指令的时间
            });

        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);

        if (publish_tf_)
        {
            tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        }
        last_cmd_time_ = node->now(); // 初始化
        last_cmd_ = std::make_shared<geometry_msgs::msg::Twist>();  // 全0

        RCLCPP_INFO(get_node()->get_logger(), "last_cmd_time=%.3f", last_cmd_time_.seconds());

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
        steering_cmds_.clear();
        drive_cmds_.clear();
        steering_state_ifaces_.clear();
        drive_state_ifaces_.clear();

        // ===== 打印所有可用命令接口 =====
        RCLCPP_INFO(get_node()->get_logger(), "=== Available COMMAND interfaces ===");
        for (const auto &iface : command_interfaces_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  [CMD] %s", iface.get_name().c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(), "=======================================");

        // ===== 打印所有可用状态接口 =====
        RCLCPP_INFO(get_node()->get_logger(), "=== Available STATE interfaces ===");
        for (const auto &iface : state_interfaces_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "  [STATE] %s", iface.get_name().c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(), "=======================================");

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
            steering_cmds_.push_back(std::ref(*steer));
            drive_cmds_.push_back(std::ref(*drive));
        }

        for (const auto &wheel : wheel_configs_)
        {
            auto *steer_pos = find_state(wheel.steering_joint_name + "/position");
            // auto *steer_vel = find_state(wheel.steering_joint_name + "/velocity");
            // auto *wheel_pos = find_state(wheel.wheel_joint_name + "/position");
            auto *wheel_vel = find_state(wheel.wheel_joint_name + "/velocity");

            // ✅ 打印每个接口的查找结果
            RCLCPP_INFO(get_node()->get_logger(), 
                        "  steer_pos: %s, wheel_vel: %s",
                        steer_pos ? "OK" : "NULL",
                        wheel_vel ? "OK" : "NULL");

            if (!steer_pos || !wheel_vel)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface for %s", wheel.steering_joint_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            steering_state_ifaces_.push_back(std::ref(*steer_pos));
            drive_state_ifaces_.push_back(std::ref(*wheel_vel));
        }

        // ✅ 打印 steering_state_ifaces_ 的内容
        RCLCPP_INFO(get_node()->get_logger(), "steering_state_ifaces_ size: %zu", steering_state_ifaces_.size());
        for (size_t i = 0; i < steering_state_ifaces_.size(); ++i)
        {
            RCLCPP_INFO(get_node()->get_logger(), 
                        "  steering_state_ifaces_[%zu]: %s", 
                        i, 
                        steering_state_ifaces_[i].get().get_name().c_str());
        }

        // ✅ 打印 drive_state_ifaces_ 的内容
        RCLCPP_INFO(get_node()->get_logger(), "drive_state_ifaces_ size: %zu", drive_state_ifaces_.size());
        for (size_t i = 0; i < drive_state_ifaces_.size(); ++i)
        {
            RCLCPP_INFO(get_node()->get_logger(), 
                        "  drive_state_ifaces_[%zu]: %s", 
                        i, 
                        drive_state_ifaces_[i].get().get_name().c_str());
        }

        // ✅ 打印 steering_cmds_ 的内容
        RCLCPP_INFO(get_node()->get_logger(), "steering_cmds_ size: %zu", steering_cmds_.size());
        for (size_t i = 0; i < steering_cmds_.size(); ++i)
        {
            RCLCPP_INFO(get_node()->get_logger(), 
                        "  steering_cmds_[%zu]: %s", 
                        i, 
                        steering_cmds_[i].get().get_name().c_str());
        }

        // ✅ 打印 drive_cmds_ 的内容
        RCLCPP_INFO(get_node()->get_logger(), "drive_cmds_ size: %zu", drive_cmds_.size());
        for (size_t i = 0; i < drive_cmds_.size(); ++i)
        {
            RCLCPP_INFO(get_node()->get_logger(), 
                        "  drive_cmds_[%zu]: %s", 
                        i, 
                        drive_cmds_[i].get().get_name().c_str());
        }

        odom_x_ = odom_y_ = odom_yaw_ = 0.0;
        prev_steering_angles_ = {0.0, 0.0, 0.0};

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
        // 1. 读取当前关节状态
        if (!readCurrentWheelStates())
        {
            return controller_interface::return_type::ERROR;
        }

        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        if (last_cmd_)
        {
            rclcpp::Time now = get_node()->now();
            double dt = (now - last_cmd_time_).seconds();
            RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                 "time=%.3f, last_cmd_time=%.3f, dt=%.3f",
                                 now.seconds(), last_cmd_time_.seconds(), dt);
            if (dt < CMD_TIMEOUT) // 0.5秒超时
            {
                vx = last_cmd_->linear.x;
                vy = last_cmd_->linear.y;
                omega = last_cmd_->angular.z;
            }
            else
            {
                last_cmd_->linear.x = 0.0;
                last_cmd_->linear.y = 0.0;
                last_cmd_->angular.z = 0.0;
                // 可选：打一次日志
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                                    "Command timeout, zeroing velocity");
            }

            // 3. 输入速度限制（保护）
            double v_norm = std::hypot(vx, vy);
            if (v_norm > max_linear_velocity_)
            {
                double scale = max_linear_velocity_ / v_norm;
                vx *= scale;
                vy *= scale;
            }
            omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);

            // double front_angle, left_angle, right_angle;
            // double front_speed, left_speed, right_speed;

            // computeKinematics(vx, vy, omega,
            //                   front_angle, left_angle, right_angle,
            //                   front_speed, left_speed, right_speed);

            std::array<double, 3> steering_angles{0.0, 0.0, 0.0};
            std::array<double, 3> wheel_speeds{0.0, 0.0, 0.0};

            computeKinematics(vx, vy, omega, steering_angles, wheel_speeds);

            // 5. 后退优化：优先反转轮速而非旋转舵轮180° ，舵轮转角限制在正负90度，必须反转轮速
            optimizeReverse(steering_angles, wheel_speeds, prev_steering_angles_);
            // if (enable_reverse_optimization_)
            // {
            //     optimizeReverse(steering_angles, wheel_speeds, prev_steering_angles_);
            // }

            // 6. 舵角最短路径归一化
            for (size_t i = 0; i < 3; ++i)
            {
                steering_angles[i] = normalizeSteeringAngle(prev_steering_angles_[i], steering_angles[i]);
                // 限制在最大转向角范围内
                steering_angles[i] = std::clamp(steering_angles[i],
                                                -wheel_configs_[i].max_steering_angle,
                                                wheel_configs_[i].max_steering_angle);
            }

            // 7. 轮速限制
            limitVelocities(wheel_speeds);

            // 8. 写入硬件
            for (size_t i = 0; i < 3; ++i)
            {
                steering_cmds_[i].get().set_value(steering_angles[i]);
                drive_cmds_[i].get().set_value(wheel_speeds[i]);
            }

            // 9. 记录当前舵角供下一周期使用
            prev_steering_angles_ = steering_angles;

            // 积分
            const double dt2 = period.seconds();

            odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt2;
            odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt2;
            odom_yaw_ += omega * dt2;

            while (odom_yaw_ > M_PI)
                odom_yaw_ -= 2.0 * M_PI;
            while (odom_yaw_ < -M_PI)
                odom_yaw_ += 2.0 * M_PI;

            // 10. 里程计积分与发布 gazebo已经在否是~/odom he ~/tf了，这里先不发试试
            publishOdometry(time, odom_x_, odom_y_, omega);

            // auto node = get_node();
            // if (odom_pub_ && node)
            // {
            //     odom_msg_.header.stamp = time;
            //     odom_msg_.header.frame_id = "odom";
            //     odom_msg_.child_frame_id = "base_link";
            //     odom_msg_.pose.pose.position.x = odom_x_;
            //     odom_msg_.pose.pose.position.y = odom_y_;
            //     odom_msg_.pose.pose.position.z = 0.0;
            //     odom_msg_.pose.pose.orientation.x = 0.0;
            //     odom_msg_.pose.pose.orientation.y = 0.0;
            //     odom_msg_.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
            //     odom_msg_.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
            //     odom_msg_.twist.twist.linear.x = vx;
            //     odom_msg_.twist.twist.linear.y = vy;
            //     odom_msg_.twist.twist.angular.z = omega;
            //     odom_pub_->publish(odom_msg_);
            // }

            return controller_interface::return_type::OK;
        }

        return controller_interface::return_type::OK;
    }

    // void ThreeWheelSteeringController::computeKinematics(
    //     double vx, double vy, double omega,
    //     double &front_angle, double &left_angle, double &right_angle,
    //     double &front_speed, double &left_speed, double &right_speed)
    // {
    //     // 120-degree symmetric layout
    //     const double theta_f = 0.0;
    //     const double theta_l = 2.0 * M_PI / 3.0;
    //     const double theta_r = 4.0 * M_PI / 3.0;

    //     auto wheel_v = [&](double angle, double &v_x, double &v_y)
    //     {
    //         v_x = vx - omega * chassis_radius_ * std::sin(angle);
    //         v_y = vy + omega * chassis_radius_ * std::cos(angle);
    //     };

    //     double vfx, vfy, vlx, vly, vrx, vry;
    //     wheel_v(theta_f, vfx, vfy);
    //     wheel_v(theta_l, vlx, vly);
    //     wheel_v(theta_r, vrx, vry);

    //     front_angle = std::atan2(vfy, vfx);
    //     left_angle = std::atan2(vly, vlx);
    //     right_angle = std::atan2(vry, vrx);

    //     front_speed = std::hypot(vfx, vfy) / wheel_radius_;
    //     left_speed = std::hypot(vlx, vly) / wheel_radius_;
    //     right_speed = std::hypot(vrx, vry) / wheel_radius_;
    // }

    // ==================== 核心算法 ====================

    /**
     * @brief 基于实际轮心坐标的运动学逆解
     *
     * 每个轮子的理想速度：
     *   v_i = [vx - ω·y_i,  vy + ω·x_i]
     *
     * 舵角：atan2(vy_i, vx_i)
     * 轮速：|v_i| / wheel_radius
     */
    void ThreeWheelSteeringController::computeKinematics(
        double vx, double vy, double omega,
        std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            const auto &wc = wheel_configs_[i];

            // 轮心处的线速度（刚体运动学）
            double vxi = vx - omega * wc.y;
            double vyi = vy + omega * wc.x;

            steering_angles[i] = std::atan2(vyi, vxi);
            wheel_speeds[i] = std::hypot(vxi, vyi) / wheel_radius_;

            RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Wheel[%zu]: vxi=%.3f, vyi=%.3f, angle=%.1f°",
            i, vxi, vyi, steering_angles[i] * 180.0 / M_PI);
            
        }
    }

    /**
     * @brief 舵角最短路径归一化
     *
     * 将目标角度映射到与当前角度差值最小的等效角度
     * 例如：current=179°, target=-179° → 实际转 +2°（到181°）
     */
    double ThreeWheelSteeringController::normalizeSteeringAngle(double current, double target) const
    {
        // 先归一化到 [-π, π]
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

        // 找到最短路径的等效角度
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

    /**
     * @brief 后退优化
     *
     * 当目标舵角与当前舵角差值 > 90° 时，
     * 选择反转轮速（wheel_speed *= -1）而非旋转舵轮180°
     */
    void ThreeWheelSteeringController::optimizeReverse(
        std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds,
        const std::array<double, 3> &current_angles)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            double diff = std::abs(steering_angles[i] - current_angles[i]);
            // 取最小角度差（考虑周期性）
            while (diff > M_PI)
                diff -= 2.0 * M_PI;
            diff = std::abs(diff);

            if (diff > M_PI / 2.0)
            {
                // 反转轮速，调整舵角 ±180°
                wheel_speeds[i] = -wheel_speeds[i];
                if (steering_angles[i] > 0)
                    steering_angles[i] -= M_PI;
                else
                    steering_angles[i] += M_PI;
            }
        }
    }

    /**
     * @brief 轮速饱和限制
     *
     * 如果某个轮子超速，等比例缩放所有轮子的速度
     * 保持运动学一致性
     */
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
        //  RCLCPP_INFO(get_node()->get_logger(), 
        //         "steering_state_ifaces_.size()=%zu, drive_state_ifaces_.size()=%zu",
        //         steering_state_ifaces_.size(), drive_state_ifaces_.size());
        if (steering_state_ifaces_.size() < 3 || drive_state_ifaces_.size() < 3)
        {
             return false;
        }

        for (size_t i = 0; i < 3; ++i)
        {
             prev_steering_angles_[i] = steering_state_ifaces_[i].get().get_value();
        }
        return true;
    }

    void ThreeWheelSteeringController::publishOdometry(
        const rclcpp::Time &time, double vx, double vy, double omega)
    {
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