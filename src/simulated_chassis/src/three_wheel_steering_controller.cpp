
#include "simulated_chassis/three_wheel_steering_controller.hpp"
#include <cmath>
#include <limits>

namespace three_wheel_controller
{

    ThreeWheelSteeringController::ThreeWheelSteeringController()
    {
        // 默认轮位配置：基于你的URDF坐标
        // 前轮: (0.3, 0), 左后轮: (-0.15, 0.20), 右后轮: (-0.15, -0.20)
        wheel_configs_ = {
            {"wheel_front_steering_joint", "wheel_front_wheel_joint", 0.3, 0.0, M_PI / 2.0},
            {"wheel_left_steering_joint", "wheel_left_wheel_joint", -0.15, 0.20, M_PI / 2.0},
            {"wheel_right_steering_joint", "wheel_right_wheel_joint", -0.15, -0.20, M_PI / 2.0}};
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
        if (!node->has_parameter("steering_hold_velocity_threshold"))
            node->declare_parameter<double>("steering_hold_velocity_threshold", steering_hold_velocity_threshold_);
        if (!node->has_parameter("alignment_full_speed_angle"))
            node->declare_parameter<double>("alignment_full_speed_angle", alignment_full_speed_angle_);
        if (!node->has_parameter("reverse_switch_hysteresis"))
            node->declare_parameter<double>("reverse_switch_hysteresis", reverse_switch_hysteresis_);
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
        node->get_parameter("steering_hold_velocity_threshold", steering_hold_velocity_threshold_);
        node->get_parameter("alignment_full_speed_angle", alignment_full_speed_angle_);
        node->get_parameter("reverse_switch_hysteresis", reverse_switch_hysteresis_);
        node->get_parameter("publish_tf", publish_tf_);
        node->get_parameter("odom_frame_id", odom_frame_id_);
        node->get_parameter("base_frame_id", base_frame_id_);

        if (alignment_full_speed_angle_ < 0.0 ||
            reverse_switch_hysteresis_ < 0.0 ||
            steering_hold_velocity_threshold_ < 0.0)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid steering alignment/hold/hysteresis parameters");
            return controller_interface::CallbackReturn::ERROR;
        }

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

        // 按关节名查找接口，不依赖 ros2_control 提供的接口顺序。
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
            auto *wheel_vel = find_state(wheel.wheel_joint_name + "/velocity");

            if (!steer_pos || !wheel_vel)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface for %s", wheel.steering_joint_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            steering_state_ifaces_.push_back(std::ref(*steer_pos));
            drive_state_ifaces_.push_back(std::ref(*wheel_vel));
        }

        odom_x_ = odom_y_ = odom_yaw_ = 0.0;
        actual_steering_angles_ = {0.0, 0.0, 0.0};
        actual_wheel_velocities_ = {0.0, 0.0, 0.0};
        commanded_steering_angles_ = {0.0, 0.0, 0.0};
        selected_drive_directions_ = {1, 1, 1};

        RCLCPP_INFO(get_node()->get_logger(),
                    "Three-wheel steering controller activated with 3 steering and 3 drive interfaces");

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
        // 阶段1：采集真实关节状态。后续的等价解选择和对齐判断
        // 都必须使用这份状态，不能使用上一周期的目标命令。
        if (!readCurrentWheelStates())
        {
            return controller_interface::return_type::ERROR;
        }

        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        getLimitedCommand(time, vx, vy, omega);

        // 阶段2：逆运动学生成每个轮组的目标舵角和轮速。
        std::array<double, 3> steering_angles{0.0, 0.0, 0.0};
        std::array<double, 3> wheel_speeds{0.0, 0.0, 0.0};
        computeKinematics(vx, vy, omega, steering_angles, wheel_speeds);

        // 阶段3：在 (α, v) 与 (α±π, -v) 中，基于真实舵角选择转动最小的可达解。
        // 零速时 optimizeReverse() 会保持最后的真实舵角，不强制回零。
        optimizeReverse(steering_angles, wheel_speeds, actual_steering_angles_);

        // 阶段4：严格“先摆舵，后驱动”。任一舵轮未对齐时，三个驱动轮都为零速。
        scaleWheelSpeedsForSteeringAlignment(
            steering_angles, wheel_speeds, actual_steering_angles_);
        limitVelocities(wheel_speeds);

        // 阶段5：下发命令。目标舵角只用于记录，不会写回真实舵角缓存。
        writeWheelCommands(steering_angles, wheel_speeds);
        commanded_steering_angles_ = steering_angles;

        // 阶段6：用真实舵角和真实轮速反算底盘运动，更新里程计。
        updateOdometryFromWheelStates(time, period);

        return controller_interface::return_type::OK;
    }

    void ThreeWheelSteeringController::getLimitedCommand(
        const rclcpp::Time &time, double &vx, double &vy, double &omega)
    {
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;

        // 没有指令或指令超时时安全停车。不修改 last_cmd_ 消息本身，
        // 避免控制循环与 ROS 订阅回调同时写入同一个对象。
        if (!last_cmd_ || (time - last_cmd_time_).seconds() >= cmd_timeout_)
        {
            return;
        }

        vx = last_cmd_->linear.x;
        vy = last_cmd_->linear.y;
        omega = last_cmd_->angular.z;

        // 线速度按向量等比限幅，保留 vx/vy 的方向。
        const double linear_speed = std::hypot(vx, vy);
        if (linear_speed > max_linear_velocity_)
        {
            const double scale = max_linear_velocity_ / linear_speed;
            vx *= scale;
            vy *= scale;
        }
        omega = std::clamp(omega, -max_angular_velocity_, max_angular_velocity_);
    }

    void ThreeWheelSteeringController::writeWheelCommands(
        const std::array<double, 3> &steering_angles,
        const std::array<double, 3> &wheel_speeds)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            steering_cmds_[i].get().set_value(steering_angles[i]);
            drive_cmds_[i].get().set_value(wheel_speeds[i]);
        }
    }

    void ThreeWheelSteeringController::updateOdometryFromWheelStates(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        double estimated_vx = 0.0;
        double estimated_vy = 0.0;
        double estimated_omega = 0.0;
        computeForwardKinematics(
            actual_steering_angles_, actual_wheel_velocities_,
            estimated_vx, estimated_vy, estimated_omega);

        const double dt = period.seconds();
        if (dt > 0.0 && dt < 1.0)
        {
            if (std::abs(estimated_omega) < 1e-6)
            {
                odom_x_ += (estimated_vx * std::cos(odom_yaw_) -
                            estimated_vy * std::sin(odom_yaw_)) * dt;
                odom_y_ += (estimated_vx * std::sin(odom_yaw_) +
                            estimated_vy * std::cos(odom_yaw_)) * dt;
            }
            else
            {
                const double delta_yaw = estimated_omega * dt;
                const double speed = std::hypot(estimated_vx, estimated_vy);
                const double radius = speed / estimated_omega;
                const double heading = std::atan2(estimated_vy, estimated_vx) + odom_yaw_;

                odom_x_ += radius * (std::sin(heading + delta_yaw) - std::sin(heading));
                odom_y_ += -radius * (std::cos(heading + delta_yaw) - std::cos(heading));
                odom_yaw_ += delta_yaw;
            }

            while (odom_yaw_ > M_PI)
                odom_yaw_ -= 2.0 * M_PI;
            while (odom_yaw_ < -M_PI)
                odom_yaw_ += 2.0 * M_PI;
        }

        publishOdometry(time, estimated_vx, estimated_vy, estimated_omega);
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
        double ATA[3][3] = {{0}};
        double ATb[3] = {0};

        for (size_t i = 0; i < 3; ++i)
        {
            double xi = wheel_configs_[i].x;
            double yi = wheel_configs_[i].y;
            double alpha = steering_angles[i];
            double v_w = wheel_velocities[i] * wheel_radius_;

            double c = std::cos(alpha);
            double s = std::sin(alpha);

            // 约束方程与逆运动学一致：
            // v_w * cos(α) = vx - ω*yi
            // v_w * sin(α) = vy + ω*xi

            // 六个约束使用相同权重。
            constexpr double w2 = 1.0;

            // === X约束: vx - ω*yi = v_w * cos(α) ===
            ATA[0][0] += w2;
            ATA[0][2] += w2 * (-yi);
            ATA[2][0] += w2 * (-yi);
            ATA[2][2] += w2 * yi * yi;
            ATb[0] += w2 * v_w * c;
            ATb[2] += w2 * (-yi) * v_w * c;

            // === Y约束: vy + ω*xi = v_w * sin(α) ===
            ATA[1][1] += w2;
            ATA[1][2] += w2 * xi;
            ATA[2][1] += w2 * xi;
            ATA[2][2] += w2 * xi * xi;
            ATb[1] += w2 * v_w * s;
            ATb[2] += w2 * xi * v_w * s;
        }

        // ========== 求解线性系统 ==========
        double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) - ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) + ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

        if (std::abs(det) < 1e-9)
        {
            vx = vy = omega = 0.0;
            return;
        }

        double inv_det = 1.0 / det;
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

    }

    /**
     * @brief 在有限舵角内选择等价的舵角/轮速
     *
     * 滚动方向满足 v*[cos(α), sin(α)] =
     * (-v)*[cos(α±π), sin(α±π)]。枚举 α+kπ，只在物理限位内
     * 比较真实转角，不对 revolute 关节使用跨限位的周期“捷径”。
     */
    void ThreeWheelSteeringController::optimizeReverse(
        std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds,
        const std::array<double, 3> &current_angles)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            const double linear_speed = std::abs(wheel_speeds[i]) * wheel_radius_;
            if (linear_speed < steering_hold_velocity_threshold_)
            {
                steering_angles[i] = std::clamp(
                    current_angles[i],
                    -wheel_configs_[i].max_steering_angle,
                    wheel_configs_[i].max_steering_angle);
                wheel_speeds[i] = 0.0;
                continue;
            }

            const double raw_angle = steering_angles[i];
            const double raw_speed = wheel_speeds[i];
            const double max_steer = wheel_configs_[i].max_steering_angle;
            double best_cost = std::numeric_limits<double>::infinity();
            double best_angle = std::clamp(raw_angle, -max_steer, max_steer);
            int best_direction = 0;

            // atan2 输出 [-π, π]，k∈[-2,2] 已覆盖所有可能落入舵角限位的等价解。
            for (int k = -2; k <= 2; ++k)
            {
                const double candidate = raw_angle + static_cast<double>(k) * M_PI;
                const int direction = (std::abs(k) % 2 == 0) ? 1 : -1;
                const double bounded_candidate = std::clamp(candidate, -max_steer, max_steer);
                const double limit_overflow = std::abs(candidate - bounded_candidate);

                // 在±90°附近允许保持上次轮速符号，将几度的越界量吸收在限位上。
                // 这避免目标方向在 90° 两侧微小波动时，舵轮在 +90°/-90° 间往返跳变。
                const bool inside_limit = limit_overflow <= 1e-9;
                const bool keep_direction_in_hysteresis =
                    direction == selected_drive_directions_[i] &&
                    limit_overflow <= reverse_switch_hysteresis_;
                if (!inside_limit && !keep_direction_in_hysteresis)
                    continue;

                double cost = std::abs(bounded_candidate - current_angles[i]);

                // 等价解成本非常接近时，优先保持上次轮速符号，避免在边界抖动。
                if (direction != selected_drive_directions_[i])
                    cost += reverse_switch_hysteresis_;

                // 关闭“优化”时优先正向轮速，但超出舵角限位时仍必须反转。
                if (!enable_reverse_optimization_ && direction < 0)
                    cost += 100.0;

                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_angle = bounded_candidate;
                    best_direction = direction;
                }
            }

            // ±90° 限位对任意平面速度都应存在等价解；无解时安全停止该轮。
            if (best_direction == 0)
            {
                steering_angles[i] = std::clamp(current_angles[i], -max_steer, max_steer);
                wheel_speeds[i] = 0.0;
                continue;
            }

            steering_angles[i] = best_angle;
            wheel_speeds[i] = raw_speed * static_cast<double>(best_direction);
            selected_drive_directions_[i] = best_direction;
        }
    }

    void ThreeWheelSteeringController::scaleWheelSpeedsForSteeringAlignment(
        const std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds,
        const std::array<double, 3> &current_angles) const
    {
        double max_error = 0.0;
        for (size_t i = 0; i < 3; ++i)
        {
            const double error = std::abs(steering_angles[i] - current_angles[i]);
            max_error = std::max(max_error, error);
        }

        // 严格的“先摆舵，再驱动”：只要一个轮组还没有对齐，
        // 三个驱动轮都保持零速，避免破坏三轮运动学比例。
        if (max_error > alignment_full_speed_angle_)
        {
            wheel_speeds.fill(0.0);
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
            actual_steering_angles_[i] = steering_state_ifaces_[i].get().get_value();
            actual_wheel_velocities_[i] = drive_state_ifaces_[i].get().get_value();
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
        last_odom_time_ = time;

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

        // ===== 协方差：仿真用常值即可 =====
        // 值小 = Cartographer 更信任里程计；值大 = 更依赖激光匹配
        // 0.01 表示约 10cm 的不确定度，调试时可调整

        // Pose covariance
        std::fill(std::begin(odom_msg_.pose.covariance), std::end(odom_msg_.pose.covariance), 0.0);
        odom_msg_.pose.covariance[0] = 0.01;     // x
        odom_msg_.pose.covariance[7] = 0.01;     // y
        odom_msg_.pose.covariance[14] = 99999.0; // z (不可观测)
        odom_msg_.pose.covariance[21] = 99999.0; // roll
        odom_msg_.pose.covariance[28] = 99999.0; // pitch
        odom_msg_.pose.covariance[35] = 0.02;    // yaw

        // Twist covariance
        std::fill(std::begin(odom_msg_.twist.covariance), std::end(odom_msg_.twist.covariance), 0.0);
        odom_msg_.twist.covariance[0] = 0.01;     // vx
        odom_msg_.twist.covariance[7] = 0.01;     // vy
        odom_msg_.twist.covariance[14] = 99999.0; // vz
        odom_msg_.twist.covariance[21] = 99999.0; // vroll
        odom_msg_.twist.covariance[28] = 99999.0; // vpitch
        odom_msg_.twist.covariance[35] = 0.02;    // omega

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

        RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "Odom stamp=%.3f, pose=(%.3f, %.3f, %.1f deg), twist=(%.3f, %.3f, %.3f)",
                              time.seconds(), odom_x_, odom_y_, odom_yaw_ * 180.0 / M_PI,
                              vx, vy, omega);
    }

} // namespace three_wheel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    three_wheel_controller::ThreeWheelSteeringController,
    controller_interface::ControllerInterface)
