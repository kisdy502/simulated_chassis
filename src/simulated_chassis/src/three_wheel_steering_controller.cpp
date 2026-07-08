
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

        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        if (publish_tf_)
        {
            tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        }
        last_cmd_time_ = node->now();                              // 初始化
        last_cmd_ = std::make_shared<geometry_msgs::msg::Twist>(); // 全0

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
            double dt = (time - last_cmd_time_).seconds();
            RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 10000,
                                 "time=%.3f, last_cmd_time=%.3f, dt=%.3f",
                                 time.seconds(), last_cmd_time_.seconds(), dt);
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
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 15000,
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

            std::array<double, 3> steering_angles{0.0, 0.0, 0.0};
            std::array<double, 3> wheel_speeds{0.0, 0.0, 0.0};

            computeKinematics(vx, vy, omega, steering_angles, wheel_speeds);

            // 5. 后退优化：优先反转轮速而非旋转舵轮180° ，舵轮转角限制在正负90度，必须反转轮速
            optimizeReverse(steering_angles, wheel_speeds, prev_steering_angles_);

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
                bool steer_ok = steering_cmds_[i].get().set_value(steering_angles[i]);
                bool drive_ok = drive_cmds_[i].get().set_value(wheel_speeds[i]);

                if (!steer_ok)
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                         "Failed to set steering command for wheel[%zu]", i);
                if (!drive_ok)
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                         "Failed to set drive command for wheel[%zu]", i);
            }

            // 9. 记录当前舵角供下一周期使用
            prev_steering_angles_ = steering_angles;

            // 10. 里程计 — 从轮子 state interface 读实际舵角+轮速，前向运动学反算
            std::array<double, 3> actual_steering{0, 0, 0};
            std::array<double, 3> actual_wheel_vel{0, 0, 0};
            for (size_t i = 0; i < 3; ++i)
            {
                auto steer_opt = steering_state_ifaces_[i].get().get_optional();
                auto wheel_opt = drive_state_ifaces_[i].get().get_optional();
                if (!steer_opt.has_value() || !wheel_opt.has_value())
                {
                    RCLCPP_ERROR(get_node()->get_logger(),
                                 "State interface unavailable for wheel[%zu]", i);
                    return controller_interface::return_type::ERROR;
                }
                actual_steering[i] = steer_opt.value();
                actual_wheel_vel[i] = wheel_opt.value();
            }

            double est_vx, est_vy, est_omega;
            computeForwardKinematics(actual_steering, actual_wheel_vel, est_vx, est_vy, est_omega);

            // ========== 精确积分（必须用 EST 值）==========
            double dt2 = period.seconds();
            if (dt2 > 0.0 && dt2 < 1.0)
            {
                if (std::abs(est_omega) < 1e-6)
                {
                    odom_x_ += est_vx * std::cos(odom_yaw_) * dt2 - est_vy * std::sin(odom_yaw_) * dt2;
                    odom_y_ += est_vx * std::sin(odom_yaw_) * dt2 + est_vy * std::cos(odom_yaw_) * dt2;
                }
                else
                {
                    double dtheta = est_omega * dt2;
                    double v = std::hypot(est_vx, est_vy);
                    double R = v / est_omega;
                    double theta0 = std::atan2(est_vy, est_vx) + odom_yaw_;

                    odom_x_ += R * (std::sin(theta0 + dtheta) - std::sin(theta0));
                    odom_y_ += -R * (std::cos(theta0 + dtheta) - std::cos(theta0));
                    odom_yaw_ += dtheta;
                }

                while (odom_yaw_ > M_PI)
                    odom_yaw_ -= 2.0 * M_PI;
                while (odom_yaw_ < -M_PI)
                    odom_yaw_ += 2.0 * M_PI;
            }

            publishOdometry(time, est_vx, est_vy, est_omega);
            return controller_interface::return_type::OK;
        }

        return controller_interface::return_type::OK;
    }

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

            // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Wheel[%zu]: vxi=%.3f, vyi=%.3f, angle=%.1f°",
            //                      i, vxi, vyi, steering_angles[i] * 180.0 / M_PI);
        }
    }

    /**
     * @brief 前向运动学：从实际舵角+轮速（rad/s）反算底盘速度
     *
     * 每个轮子的刚体约束：v_wheel_i * cos(α_i) = vx - ω·y_i
     *                      v_wheel_i * sin(α_i) = vy + ω·x_i
     * 3轮6方程 → 最小二乘求解 vx, vy, ω
     */
    void ThreeWheelSteeringController::computeForwardKinematics(
        const std::array<double, 3> &steering_angles,
        const std::array<double, 3> &wheel_velocities,
        double &vx, double &vy, double &omega)
    {
        // ========== 1. 构建加权最小二乘系统 ==========
        // A^T A * [vx, vy, omega]^T = A^T b
        // 其中 A 是 6x3 矩阵，b 是 6x1 向量

        double ATA[3][3] = {{0}};            // A^T A
        double ATb[3] = {0};                 // A^T b
        // double weights[3] = {1.0, 1.0, 1.0}; // ← 移到这，初始化1.0

        for (size_t i = 0; i < 3; ++i)
        {
            double xi = wheel_configs_[i].x;
            double yi = wheel_configs_[i].y;
            double alpha = steering_angles[i];
            double v_w = wheel_velocities[i] * wheel_radius_;

            double c = std::cos(alpha);
            double s = std::sin(alpha);

            // === 权重计算：舵角可靠性 ===
            double w = std::abs(c); // |cos(α)|: 90°时→0, 0°时→1
            if (w < 0.2)
                w = 0.2; // 最低保留20%

            // 异常检测：cos²+sin² 应≈1
            double dir_norm = c * c + s * s;
            if (std::abs(dir_norm - 1.0) > 0.15)
            {
                w = 0.05; // 编码器异常，几乎不用
                // RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                //                      "Wheel[%zu] steering angle invalid: cos²+sin²=%.3f", i, dir_norm);
            }

            double w2 = w * w; // 权重平方（因为 A^T A 和 A^T b 都要乘 w）

            // 约束1: v_w * c = vx - ω*yi  →  [1, 0, -yi] * [vx,vy,ω]^T = v_w*c
            // 约束2: v_w * s = vy + ω*xi  →  [0, 1,  xi] * [vx,vy,ω]^T = v_w*s

            // A^T A 累加
            ATA[0][0] += w2 * 1.0;
            ATA[0][1] += 0.0;
            ATA[0][2] += w2 * (-yi);
            ATA[1][0] += 0.0;
            ATA[1][1] += w2 * 1.0;
            ATA[1][2] += w2 * xi;
            ATA[2][0] += w2 * (-yi);
            ATA[2][1] += w2 * xi;
            ATA[2][2] += w2 * (xi * xi + yi * yi);

            // A^T b 累加
            ATb[0] += w2 * v_w * c;
            ATb[1] += w2 * v_w * s;
            ATb[2] += w2 * (-yi * v_w * c + xi * v_w * s);
            // weights[i] = w;
        }

        // ========== 2. 求解 3x3 线性系统 (ATA * x = ATb) ==========
        // 用克拉默法则 / 伴随矩阵求逆

        double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) - ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) + ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

        if (std::abs(det) < 1e-9)
        {
            // RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
            //                      "Forward kinematics singular! det=%.2e", det);
            vx = vy = omega = 0.0;
            return;
        }

        double inv_det = 1.0 / det;

        // 伴随矩阵
        double invATA[3][3];
        invATA[0][0] = (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) * inv_det;
        invATA[0][1] = -(ATA[0][1] * ATA[2][2] - ATA[0][2] * ATA[2][1]) * inv_det;
        invATA[0][2] = (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]) * inv_det;
        invATA[1][0] = -(ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) * inv_det;
        invATA[1][1] = (ATA[0][0] * ATA[2][2] - ATA[0][2] * ATA[2][0]) * inv_det;
        invATA[1][2] = -(ATA[0][0] * ATA[1][2] - ATA[0][2] * ATA[1][0]) * inv_det;
        invATA[2][0] = (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]) * inv_det;
        invATA[2][1] = -(ATA[0][0] * ATA[2][1] - ATA[0][1] * ATA[2][0]) * inv_det;
        invATA[2][2] = (ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]) * inv_det;

        vx = invATA[0][0] * ATb[0] + invATA[0][1] * ATb[1] + invATA[0][2] * ATb[2];
        vy = invATA[1][0] * ATb[0] + invATA[1][1] * ATb[1] + invATA[1][2] * ATb[2];
        omega = invATA[2][0] * ATb[0] + invATA[2][1] * ATb[1] + invATA[2][2] * ATb[2];

        // ========== 3. 调试日志（5秒一次）==========
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        //                      "\n===== Forward Kinematics Debug (5s) =====\n"
        //                      "  [Raw Input]\n"
        //                      "    Wheel0: steer=%+.4f rad (%+.2f°), wheel_vel=%+.4f rad/s, weight=%.3f\n"
        //                      "    Wheel1: steer=%+.4f rad (%+.2f°), wheel_vel=%+.4f rad/s, weight=%.3f\n"
        //                      "    Wheel2: steer=%+.4f rad (%+.2f°), wheel_vel=%+.4f rad/s, weight=%.3f\n"
        //                      "  [Computed]\n"
        //                      "    vx=%+.6f m/s, vy=%+.6f m/s, omega=%+.6f rad/s (%.4f°/s)\n"
        //                      "    v_norm=%.6f m/s, yaw_rate=%.4f°/s\n"
        //                      "========================================",
        //                      steering_angles[0], steering_angles[0] * 180.0 / M_PI, wheel_velocities[0], weights[0],
        //                      steering_angles[1], steering_angles[1] * 180.0 / M_PI, wheel_velocities[1], weights[1],
        //                      steering_angles[2], steering_angles[2] * 180.0 / M_PI, wheel_velocities[2], weights[2],
        //                      vx, vy, omega, omega * 180.0 / M_PI,
        //                      std::hypot(vx, vy), omega * 180.0 / M_PI);
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
            auto opt = steering_state_ifaces_[i].get().get_optional();
            if (!opt.has_value())
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Cannot read steering state for wheel[%zu]", i);
                return false;
            }
            prev_steering_angles_[i] = opt.value();
        }
        return true;
    }

    void ThreeWheelSteeringController::publishOdometry(
        const rclcpp::Time &time, double vx, double vy, double omega)
    {
        // 防止重复时间戳导致 Cartographer 崩溃 (map_by_time.h)
        if (time == last_odom_time_)
        {
            return;
        }
        const double dt = (time - last_odom_time_).seconds();
        last_odom_time_ = time;

        // ========== 协方差计算 ==========
        // 基础噪声参数（根据你的编码器精度调整）
        const double k_linear = 0.05;  // 线速度 5% 噪声
        const double k_angular = 0.10; // 角速度 10% 噪声
        // const double k_steer = 0.02;   // 舵角噪声对速度的贡献 (rad)
        // 速度大小
        double v = std::hypot(vx, vy);

        // 当前速度的标准差
        double sigma_vx = k_linear * std::abs(vx) + 0.01;
        double sigma_vy = k_linear * std::abs(vy) + 0.01;
        double sigma_omega = k_angular * std::abs(omega) + 0.02;

        // 舵角误差导致的速度不确定性（转弯时更明显）
        if (std::abs(omega) > 0.01)
        {
            // 转弯时，方向不确定性增大
            sigma_vy += 0.02 * v * std::abs(omega);
        }

        // 填充速度协方差
        twist_covariance_[0] = sigma_vx * sigma_vx; // vx
        twist_covariance_[1] = 0.0;                 // vx-vy 耦合
        twist_covariance_[2] = 0.0;
        twist_covariance_[3] = 0.0;
        twist_covariance_[4] = 0.0;
        twist_covariance_[5] = sigma_omega * sigma_omega; // omega

        // ========== 2. 传播位姿协方差 ==========
        if (dt > 0.0 && dt < 1.0)
        {
            // 速度误差传播到位置误差
            // 简化模型：位置误差 = 速度误差 * dt
            double sigma_x_from_vx = sigma_vx * dt;
            double sigma_y_from_vy = sigma_vy * dt;

            // 角度误差累积（陀螺积分漂移）
            double sigma_yaw_from_omega = sigma_omega * dt;

            // 旋转带来的耦合误差（转弯时 x 和 y 方向会耦合）
            double coupling = 0.0;
            if (std::abs(omega) > 0.01 && v > 0.1)
            {
                // 转弯半径 R = v / omega，误差导致半径变化
                double R = v / std::abs(omega);
                double sigma_R = 0.02 * R; // 半径估计误差 2%
                coupling = sigma_R * sigma_R * dt * dt * 0.5;
            }

            // 更新协方差（考虑速度越高，累积越快）
            double speed_factor = 1.0 + 0.5 * v; // 速度越高，协方差增长越快

            pose_covariance_[0] += (sigma_x_from_vx * sigma_x_from_vx + coupling) * speed_factor;
            pose_covariance_[1] += (sigma_y_from_vy * sigma_y_from_vy + coupling) * speed_factor;
            pose_covariance_[5] += sigma_yaw_from_omega * sigma_yaw_from_omega * speed_factor;

            // ✅ 更新 x-y 耦合项（使用独立变量）
            if (std::abs(omega) > 0.01)
            {
                xy_coupling_ += coupling * std::sin(omega * dt) * speed_factor;
            }
            else
            {
                xy_coupling_ *= 0.99; // 直线时衰减
            }

            // 静止时协方差缓慢收敛（有界）
            if (v < 0.01 && std::abs(omega) < 0.005)
            {
                // 静止状态：协方差收敛到基础噪声水平
                const double BASE_COV = 0.0001; // 基础协方差（10cm²）
                for (int i = 0; i < 6; ++i)
                {
                    if (i == 3 || i == 4)
                        continue;                 // roll, pitch 保持高值
                    pose_covariance_[i] *= 0.999; // 缓慢衰减
                    if (pose_covariance_[i] < BASE_COV)
                    {
                        pose_covariance_[i] = BASE_COV;
                    }
                }
                xy_coupling_ *= 0.99; // 静止时耦合也衰减
            }

            // 上限保护
            const double MAX_COV = 100.0; // 最大协方差（10m²）
            for (int i = 0; i < 6; ++i)
            {
                if (std::isnan(pose_covariance_[i]) || !std::isfinite(pose_covariance_[i]))
                {
                    pose_covariance_[i] = 0.01;
                }
                if (pose_covariance_[i] > MAX_COV)
                {
                    pose_covariance_[i] = MAX_COV;
                }
            }
        }

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

        // ========== 4. 填充协方差矩阵 ==========
        // Pose covariance (6x6, row-major)
        // 只填对角线和必要的耦合项
        // 对角线
        odom_msg_.pose.covariance[0] = pose_covariance_[0];  // x
        odom_msg_.pose.covariance[7] = pose_covariance_[1];  // y
        odom_msg_.pose.covariance[14] = pose_covariance_[2]; // z (固定)
        odom_msg_.pose.covariance[21] = 99999.0;             // roll (不可观测)
        odom_msg_.pose.covariance[28] = 99999.0;             // pitch (不可观测)
        odom_msg_.pose.covariance[35] = pose_covariance_[5]; // yaw

        // ✅ 耦合项使用独立变量
        if (std::abs(xy_coupling_) > 0.001)
        {
            odom_msg_.pose.covariance[1] = xy_coupling_; // x-y
            odom_msg_.pose.covariance[6] = xy_coupling_; // y-x
        }

        // Twist covariance (同样6x6)
        // 只填 vx, vy, omega
        odom_msg_.twist.covariance[0] = twist_covariance_[0];  // vx
        odom_msg_.twist.covariance[7] = twist_covariance_[1];  // vy
        odom_msg_.twist.covariance[14] = 99999.0;              // vz
        odom_msg_.twist.covariance[21] = 99999.0;              // vroll
        odom_msg_.twist.covariance[28] = 99999.0;              // vpitch
        odom_msg_.twist.covariance[35] = twist_covariance_[5]; // omega

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

        // ========== 5. 可选的调试日志 ==========
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //                      "Odom stamp=%.3f | Cov: x=%.4f, y=%.4f, yaw=%.4f | "
        //                      "Pose: x=%.3f, y=%.3f, yaw=%.1f° | Twist: vx=%.3f, vy=%.3f, omega=%.3f",
        //                      time.seconds(),
        //                      pose_covariance_[0], pose_covariance_[1], pose_covariance_[5],
        //                      odom_x_, odom_y_, odom_yaw_ * 180.0 / M_PI,
        //                      vx, vy, omega);
    }

} // namespace three_wheel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    three_wheel_controller::ThreeWheelSteeringController,
    controller_interface::ControllerInterface)