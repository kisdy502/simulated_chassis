// three_wheel_steering_controller.cpp
#include "three_wheel_steering_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace three_wheel_controller
{

    ThreeWheelSteeringController::ThreeWheelSteeringController()
        : odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0) {}

    controller_interface::CallbackReturn ThreeWheelSteeringController::on_init()
    {
        try
        {
            auto_declare<double>("wheel_base", 0.5);
            auto_declare<std::vector<double>>("wheel_angles", {0.0, 2.094, -2.094});
            auto_declare<std::vector<double>>("wheel_positions_x", {0.5, -0.25, -0.25});
            auto_declare<std::vector<double>>("wheel_positions_y", {0.0, 0.433, -0.433});
            auto_declare<double>("wheel_radius", 0.1);
            auto_declare<double>("max_steering_speed", 3.0);
            auto_declare<double>("max_wheel_speed", 10.0);
            auto_declare<bool>("allow_reverse_steering", true);
            auto_declare<std::vector<std::string>>("steering_joints",
                                                   {"wheel_front_steering_joint", "wheel_left_steering_joint", "wheel_right_steering_joint"});
            auto_declare<std::vector<std::string>>("drive_joints",
                                                   {"wheel_front_wheel_joint", "wheel_left_wheel_joint", "wheel_right_wheel_joint"});
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception during init: %s\n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeWheelSteeringController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        // 读取配置
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        max_steering_speed_ = get_node()->get_parameter("max_steering_speed").as_double();
        max_wheel_speed_ = get_node()->get_parameter("max_wheel_speed").as_double();
        allow_reverse_steering_ = get_node()->get_parameter("allow_reverse_steering").as_bool();

        auto angles = get_node()->get_parameter("wheel_angles").as_double_array();
        auto pos_x = get_node()->get_parameter("wheel_positions_x").as_double_array();
        auto pos_y = get_node()->get_parameter("wheel_positions_y").as_double_array();

        for (int i = 0; i < 3; ++i)
        {
            wheels_[i] = {pos_x[i], pos_y[i], angles[i]};
        }

        steering_joint_names_ = get_node()->get_parameter("steering_joints").as_string_array();
        drive_joint_names_ = get_node()->get_parameter("drive_joints").as_string_array();

        // 创建订阅和发布
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
                cmd_vel_ = *msg;
            });

        odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ThreeWheelSteeringController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        // 获取关节命令接口
        for (const auto &name : steering_joint_names_)
        {
            steering_cmds_.push_back(command_interfaces_[name + "/position"]);
        }
        for (const auto &name : drive_joint_names_)
        {
            drive_cmds_.push_back(command_interfaces_[name + "/velocity"]);
        }

        // 获取关节状态接口
        for (const auto &name : steering_joint_names_)
        {
            steering_states_.push_back(state_interfaces_[name + "/position"]);
        }
        for (const auto &name : drive_joint_names_)
        {
            drive_states_.push_back(state_interfaces_[name + "/velocity"]);
        }

        last_odom_time_ = get_node()->now();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ThreeWheelSteeringController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // 获取当前速度指令
        geometry_msgs::msg::Twist cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
            cmd = cmd_vel_;
        }

        // 计算三个舵轮的转向角和轮速
        std::array<double, 3> target_steering;
        std::array<double, 3> target_wheel_speeds;

        computeWheelCommands(cmd.linear.x, cmd.linear.y, cmd.angular.z,
                             target_steering, target_wheel_speeds);

        // 发送命令到关节（考虑转向优化）
        for (int i = 0; i < 3; ++i)
        {
            double current_steering = steering_states_[i].get().get_value();

            bool reverse = false;
            double optimized_steering = target_steering[i];

            if (allow_reverse_steering_)
            {
                optimized_steering = optimizeSteering(current_steering, target_steering[i], reverse);
            }

            // 如果反转，轮速取反
            if (reverse)
            {
                target_wheel_speeds[i] = -target_wheel_speeds[i];
            }

            // 限制轮速
            target_wheel_speeds[i] = std::clamp(target_wheel_speeds[i],
                                                -max_wheel_speed_, max_wheel_speed_);

            // 发送命令
            steering_cmds_[i].get().set_value(optimized_steering);
            drive_cmds_[i].get().set_value(target_wheel_speeds[i]);
        }

        // 更新里程计
        updateOdometry(period);

        return controller_interface::return_type::OK;
    }

    void ThreeWheelSteeringController::computeWheelCommands(
        double vx, double vy, double omega,
        std::array<double, 3> &steering_angles,
        std::array<double, 3> &wheel_speeds)
    {

        for (int i = 0; i < 3; ++i)
        {
            const auto &wheel = wheels_[i];

            // 轮中心速度（机器人坐标系）
            double vwx = vx - omega * wheel.y;
            double vwy = vy + omega * wheel.x;

            // 转向角 = 速度方向
            steering_angles[i] = atan2(vwy, vwx);

            // 轮速 = 速度大小 / 轮半径
            double v = sqrt(vwx * vwx + vwy * vwy);
            wheel_speeds[i] = v / wheel_radius_;

            // 如果速度接近0，保持当前转向角（避免抖动）
            if (v < 0.01)
            {
                steering_angles[i] = steering_states_[i].get().get_value();
                wheel_speeds[i] = 0.0;
            }
        }
    }

    double ThreeWheelSteeringController::optimizeSteering(double current, double target, bool &reverse)
    {
        // 归一化到 [-π, π]
        current = normalizeAngle(current);
        target = normalizeAngle(target);

        // 直接转的角度差
        double diff_direct = normalizeAngle(target - current);

        // 反转后目标角
        double target_reverse = normalizeAngle(target + M_PI);
        double diff_reverse = normalizeAngle(target_reverse - current);

        // 选择转得少的方案
        if (std::abs(diff_direct) <= std::abs(diff_reverse))
        {
            reverse = false;
            return current + diff_direct;
        }
        else
        {
            reverse = true;
            return current + diff_reverse;
        }
    }

    double ThreeWheelSteeringController::normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    void ThreeWheelSteeringController::updateOdometry(const rclcpp::Duration &period)
    {
        double dt = period.seconds();

        // 从关节状态计算底盘速度
        // 简化：用指令速度积分（实际应该用轮速反馈）
        double vx = cmd_vel_.linear.x;
        double vy = cmd_vel_.linear.y;
        double omega = cmd_vel_.angular.z;

        // 积分
        double dx = vx * cos(odom_theta_) - vy * sin(odom_theta_);
        double dy = vx * sin(odom_theta_) + vy * cos(odom_theta_);

        odom_x_ += dx * dt;
        odom_y_ += dy * dt;
        odom_theta_ += omega * dt;
        odom_theta_ = normalizeAngle(odom_theta_);

        // 发布 odom
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = get_node()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist = cmd_vel_; // 简化，实际用反馈计算

        odom_pub_->publish(odom);

        // 发布 TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = odom.child_frame_id;
        tf.transform.translation.x = odom_x_;
        tf.transform.translation.y = odom_y_;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

} // namespace three_wheel_controller

PLUGINLIB_EXPORT_CLASS(three_wheel_controller::ThreeWheelSteeringController,
                       controller_interface::ControllerInterface)
