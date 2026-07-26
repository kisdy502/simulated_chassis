#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GamepadTeleopNode : public rclcpp::Node
{
public:
  GamepadTeleopNode() : Node("gamepad_teleop_node")
  {
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 0);
    this->declare_parameter<int>("axis_lateral", 6);
    this->declare_parameter<int>("axis_dpad_linear", 7);
    this->declare_parameter<double>("deadzone", 0.1);
    this->declare_parameter<double>("dominant_threshold", 0.5); // 主导方向阈值
    this->declare_parameter<double>("cross_suppress_ratio",
                                    0.2); // 交叉抑制比例
    this->declare_parameter<int>("btn_stop", 0);
    this->declare_parameter<int>("btn_speed_up", 3);
    this->declare_parameter<int>("btn_speed_down", 4);
    this->declare_parameter<std::string>("cmd_topic", cmd_topic_);
    // Add: 看门狗参数
    this->declare_parameter<double>("watchdog_timeout", 0.2); // 超时秒数
    this->declare_parameter<double>("min_publish_cmd", 0.05); // 最小发布阈值
    // Add: 加速度限制参数（避免阶跃式速度指令触发 IMU 扰动和轮子打滑）
    this->declare_parameter<double>("max_linear_accel", 0.6);   // m/s^2
    this->declare_parameter<double>("max_angular_accel", 1.5);  // rad/s^2

    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("axis_lateral", axis_lateral_);
    this->get_parameter("axis_dpad_linear", axis_dpad_linear_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("dominant_threshold", dominant_threshold_);
    this->get_parameter("cross_suppress_ratio", cross_suppress_ratio_);
    this->get_parameter("btn_stop", btn_stop_);
    this->get_parameter("btn_speed_up", btn_speed_up_);
    this->get_parameter("btn_speed_down", btn_speed_down_);
    this->get_parameter("cmd_topic", cmd_topic_);
    this->get_parameter("max_linear_accel", max_linear_accel_);
    this->get_parameter("max_angular_accel", max_angular_accel_);

    RCLCPP_INFO(this->get_logger(),
                "Gamepad Teleop: deadzone=%.2f, dominant_threshold=%.2f, "
                "cross_suppress=%.2f",
                deadzone_, dominant_threshold_, cross_suppress_ratio_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleopNode::joyCallback, this,
                  std::placeholders::_1));

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    // Add: 启动看门狗定时器，10Hz 检查
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GamepadTeleopNode::watchdogCallback, this));

    // Add: 初始化时间戳（避免初期判断误差）
    last_nonzero_cmd_time_ = this->now();
    prev_cmd_time_ = this->now();
  }

private:
  // ========== 核心优化：主导方向抑制交叉抖动 ==========
  void applyCrossSuppression(double &linear, double &angular)
  {
    double abs_linear = std::abs(linear);
    double abs_angular = std::abs(angular);

    if (abs_linear > dominant_threshold_ && abs_angular > dominant_threshold_)
    {
      return; // 都很大，不抑制
    }

    // 线性主导：angular < percentage of cross_suppress_ratio_ of linear → 抑制
    // angular
    if (abs_linear > dominant_threshold_ && abs_angular > deadzone_ &&
        abs_angular < cross_suppress_ratio_ * abs_linear)
    {
      angular = 0.0;
    }
    else if (abs_linear > dominant_threshold_ && abs_angular <= deadzone_)
    {
      angular = 0.0;
    }

    // 角速度主导：linear < 30% of angular → 抑制 linear
    else if (abs_angular > dominant_threshold_ && abs_linear > deadzone_ &&
             abs_linear < cross_suppress_ratio_ * abs_angular)
    {
      linear = 0.0;
    }

    else if (abs_angular > dominant_threshold_ && abs_linear <= deadzone_)
    {
      linear = 0.0;
    }
  }

  void applySoftCrossSuppression(double &linear, double &angular)
  {
    double abs_linear = std::abs(linear);
    double abs_angular = std::abs(angular);
    double total = abs_linear + abs_angular;
    if (total < 1e-6)
      return;

    // 计算各自占比 (0~1)
    double linear_ratio = abs_linear / total;
    double angular_ratio = abs_angular / total;

    const double major_threshold = 0.7; // 当某一方占比超过70%时，视为主导
    const double suppress_factor = 0.3; // 抑制强度，0=完全抑制，1=无抑制

    if (linear_ratio > major_threshold)
    {
      // 线性主导，轻微抑制角速度
      double strength =
          (linear_ratio - major_threshold) / (1.0 - major_threshold);
      angular *= (1.0 - suppress_factor * strength);
    }
    else if (angular_ratio > major_threshold)
    {
      double strength =
          (angular_ratio - major_threshold) / (1.0 - major_threshold);
      linear *= (1.0 - suppress_factor * strength);
    }
    // 否则两者都不占绝对主导，不抑制
  }

  // void applyEllipticalSuppression(double &linear, double &angular) {
  //   // 将归一化后的线性、角速度看作平面点，施加一个椭圆约束
  //   double norm_linear = linear / max_linear_;
  //   double norm_angular = angular / max_angular_;
  //   double radius = std::hypot(norm_linear, norm_angular);
  //   if (radius <= 1.0)
  //     return; // 在安全区内

  //   // 超出椭圆边界则等比例缩放回边界上
  //   double scale = 1.0 / radius;
  //   linear *= scale;
  //   angular *= scale;
  // }

  // 自适应动态阈值
  void applyAdaptiveSuppression(double &linear, double &angular)
  {
    double abs_l = std::abs(linear);
    double abs_a = std::abs(angular);
    double mag = std::hypot(abs_l, abs_a); // 综合强度

    // 小幅度时完全无抑制
    if (mag < 0.3)
      return;

    // 大幅度时采用软抑制，抑制强度随着幅度增大而增强
    double suppress_strength = std::clamp((mag - 0.3) / 0.7, 0.0, 1.0);
    double ratio = abs_l / (abs_a + 1e-6);
    if (ratio > 2.0)
    {
      angular *= (1.0 - 0.5 * suppress_strength);
    }
    else if (ratio < 0.5)
    {
      linear *= (1.0 - 0.5 * suppress_strength);
    }
  }

  // 平滑滤波：低通滤波器，抑制高频抖动
  double lowPassFilter(double current, double &prev, double alpha = 0.7)
  {
    double filtered =
        alpha * current + (1.0 - alpha) * prev; // 30%旧值，70%新值
    prev = filtered;
    return filtered;
  }

  void publishVelocity(const geometry_msgs::msg::Twist &twist)
  {
    vel_pub_->publish(twist);
    if (twist.linear.x == 0.0 && twist.angular.z == 0.0)
    {
      RCLCPP_INFO(this->get_logger(), "发送了停止速度控制！");
    }
  }

  double applyDeadzone(double value)
  {
    if (std::abs(value) < deadzone_)
      return 0.0;
    return value;
  }

  // 限加速度：把 target 限制在 prev ± max_accel * dt 范围内
  // 避免摇杆阶跃导致速度突变 → IMU 扰动 + 轮子打滑 + SLAM 配准失败
  double limitAccel(double target, double prev, double max_accel, double dt)
  {
    if (dt <= 0 || dt > 0.5)
      dt = 0.05; // 异常 dt 用默认值兜底
    double max_change = max_accel * dt;
    double delta = target - prev;
    if (std::abs(delta) > max_change)
    {
      return prev + std::copysign(max_change, delta);
    }
    return target;
  }

  double getLinearScale() const
  {
    switch (speed_level_)
    {
    case 0:
      return 0.3;
    case 1:
      return 0.7;
    case 2:
      return 1.2;
    default:
      return 0.7;
    }
  }

    double getAngularScale() const
    {
        switch (speed_level_)
        {
        case 0:
            return 0.6;
        case 1:
            return 1.0;
        case 2:
            return 1.5;
        default:
            return 1.0;
        }
    }

    double getLateralScale() const
    {
        return getLinearScale();
    }
  /**
   * 如果是阿克曼就机器人，只有角速度，没有线速度，是无法移动的，这是由其运动模型决定的
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // const bool has_input = hasAnyInput(msg);
    // auto twist = geometry_msgs::msg::Twist();
    // // RCLCPP_INFO(this->get_logger(), "has_input=%s",
    // //             has_input ? "true" : "false");
    // if (!has_input) {
    //   if (joy_no_trigger_send_zero < 6) {
    //     joy_no_trigger_send_zero++;
    //     publishVelocity(twist);
    //   }
    //   return;
    // }
    // joy_no_trigger_send_zero = 0;

    // 紧急停止
    if (btn_stop_ >= 0 && btn_stop_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_stop_] == 1)
      {
        publishStop(); // 直接发停止，并更新时间
        return;
      }
    }

    // 速度档位
    handleSpeedButtons(msg);

    if (axis_linear_ >= static_cast<int>(msg->axes.size()) ||
        axis_angular_ >= static_cast<int>(msg->axes.size()) ||
        axis_lateral_ >= static_cast<int>(msg->axes.size()) ||
        axis_dpad_linear_ >= static_cast<int>(msg->axes.size()))
    {
      return;
    }

    // 获取原始输入：左摇杆前后 + D-pad 前后叠加
    double raw_linear = applyDeadzone(msg->axes[axis_linear_]);
    if (axis_dpad_linear_ >= 0)
    {
      double dpad_linear = applyDeadzone(msg->axes[axis_dpad_linear_]);
      raw_linear = std::clamp(raw_linear + dpad_linear, -1.0, 1.0);
    }
    double raw_angular = applyDeadzone(msg->axes[axis_angular_]);
    double raw_lateral = applyDeadzone(msg->axes[axis_lateral_]);

    // 低通滤波（抑制高频抖动）
    double filtered_linear = lowPassFilter(raw_linear, prev_linear_);
    double filtered_angular = lowPassFilter(raw_angular, prev_angular_);
    double filtered_lateral = lowPassFilter(raw_lateral, prev_lateral_);
    // RCLCPP_INFO(this->get_logger(), "raw_linear=%.2f, raw_angular=%.2f,
    // filtered_linear=%.2f, filtered_angular=%.2f",
    //             raw_linear, raw_angular, filtered_linear, filtered_angular);
    // 核心：非线性加权
    applySoftCrossSuppression(filtered_linear, filtered_angular);
    // RCLCPP_INFO(this->get_logger(), "after setting raw_linear=%.2f,
    // raw_angular=%.2f, filtered_linear=%.2f, filtered_angular=%.2f",
    //             raw_linear, raw_angular, filtered_linear, filtered_angular);
    // 再次应用死区（抑制后可能产生新的微小值）
    filtered_linear = applyDeadzone(filtered_linear);
    filtered_angular = applyDeadzone(filtered_angular);
    filtered_lateral = applyDeadzone(filtered_lateral);

    // 速度缩放
    double linear_cmd = filtered_linear * getLinearScale();
    double angular_cmd = filtered_angular * getAngularScale();
    double lateral_cmd = filtered_lateral * getLateralScale();

    // ✅ 最小发布阈值：低于阈值的指令直接置零
    if (std::abs(linear_cmd) < min_publish_cmd_)
      linear_cmd = 0.0;
    if (std::abs(lateral_cmd) < min_publish_cmd_)
      lateral_cmd = 0.0;
    if (std::abs(angular_cmd) < min_publish_cmd_)
      angular_cmd = 0.0;

    // ✅ 限加速度：从上一帧实际下发的指令限幅过渡到当前指令
    // 注意 dt 要用本帧相对上一帧的时间差，否则停止后第一帧会瞬变
    rclcpp::Time now = this->now();
    double dt = (now - prev_cmd_time_).seconds();
    prev_cmd_time_ = now;
    linear_cmd = limitAccel(linear_cmd, prev_cmd_linear_, max_linear_accel_, dt);
    lateral_cmd = limitAccel(lateral_cmd, prev_cmd_lateral_, max_linear_accel_, dt);
    angular_cmd = limitAccel(angular_cmd, prev_cmd_angular_, max_angular_accel_, dt);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_cmd;
    twist.linear.y = lateral_cmd;
    twist.angular.z = angular_cmd;

    // 记录本帧实际下发的指令，供下一帧限加速度使用
    prev_cmd_linear_ = linear_cmd;
    prev_cmd_lateral_ = lateral_cmd;
    prev_cmd_angular_ = angular_cmd;

    bool has_cmd =
        (std::abs(linear_cmd) > 1e-6 || std::abs(lateral_cmd) > 1e-6 ||
         std::abs(angular_cmd) > 1e-6);

    if (has_cmd)
    {
      publishVelocity(twist);
      last_nonzero_cmd_time_ = this->now(); // 更新最后有效指令时间
      stopped_ = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Moving: vx=%.2f, vy=%.2f, ω=%.2f", linear_cmd, lateral_cmd, angular_cmd);
    }
    else
    {
      // 当前无有效指令，但立刻发一次停止还是留给看门狗？
      // 为了快速响应，如果之前是运动状态，这里直接发一次停止
      if (!stopped_)
      {
        publishStop();
      }
      // 如果已经是 stopped_ == true，不需要重复发，看门狗也不动作
    }
  }

  void handleSpeedButtons(const sensor_msgs::msg::Joy::SharedPtr &msg)
  {
    if (btn_speed_up_ >= 0 &&
        btn_speed_up_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_speed_up_] == 1 && !speed_up_pressed_)
      {
        if (speed_level_ < 2)
        {
          speed_level_++;
          RCLCPP_INFO(this->get_logger(), "Speed UP -> level %d (linear=%.1f)",
                      speed_level_, getLinearScale());
        }
        speed_up_pressed_ = true;
      }
      else if (msg->buttons[btn_speed_up_] == 0)
      {
        speed_up_pressed_ = false;
      }
    }

    if (btn_speed_down_ >= 0 &&
        btn_speed_down_ < static_cast<int>(msg->buttons.size()))
    {
      if (msg->buttons[btn_speed_down_] == 1 && !speed_down_pressed_)
      {
        if (speed_level_ > 0)
        {
          speed_level_--;
          RCLCPP_INFO(this->get_logger(),
                      "Speed DOWN -> level %d (linear=%.1f)", speed_level_,
                      getLinearScale());
        }
        speed_down_pressed_ = true;
      }
      else if (msg->buttons[btn_speed_down_] == 0)
      {
        speed_down_pressed_ = false;
      }
    }
  }

  // ========== 停止发布辅助函数 ==========
  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    vel_pub_->publish(stop);
    last_nonzero_cmd_time_ = this->now(); // 更新，防止看门狗立即重复触发
    // 停止后下一帧必须从 0 开始过渡，否则限加速度会卡在旧值
    prev_cmd_linear_ = 0.0;
    prev_cmd_angular_ = 0.0;
    prev_cmd_lateral_ = 0.0;
    prev_cmd_time_ = this->now();
    stopped_ = true;
    RCLCPP_INFO(this->get_logger(), "看门狗/强制停止已发送");
  }

  // ========== 看门狗回调 ==========
  void watchdogCallback()
  {
    if (stopped_)
      return; // 已停止，不重复

    double elapsed = (this->now() - last_nonzero_cmd_time_).seconds();
    if (elapsed >= watchdog_timeout_)
    {
      publishStop();
      RCLCPP_WARN(this->get_logger(), "看门狗超时 (%.2fs) -> 强制停止",
                  elapsed);
    }
  }

  int axis_linear_;
  int axis_angular_;
  int axis_lateral_;
  int axis_dpad_linear_ = 7;
  double deadzone_ = 0.1;
  double dominant_threshold_ = 0.5;   // 主导方向阈值
  double cross_suppress_ratio_ = 0.3; // 交叉抑制比例
  int btn_stop_;
  int btn_speed_up_;
  int btn_speed_down_;
  int speed_level_ = 1;
  bool speed_up_pressed_ = false;
  bool speed_down_pressed_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  int joy_no_trigger_send_zero = 6;
  std::string cmd_topic_ = "/cmd_vel";

  // 低通滤波状态
  double prev_linear_ = 0.0;
  double prev_angular_ = 0.0;
  double prev_lateral_ = 0.0;
  // ✅ 看门狗相关新增变量
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_nonzero_cmd_time_;
  double watchdog_timeout_ = 0.2;
  double min_publish_cmd_ = 0.05;
  bool stopped_ = true;

  // ✅ 加速度限制相关变量
  double max_linear_accel_ = 0.6;   // m/s^2
  double max_angular_accel_ = 1.5;  // rad/s^2
  double prev_cmd_linear_ = 0.0;    // 上一帧实际下发的线速度
  double prev_cmd_angular_ = 0.0;   // 上一帧实际下发的角速度
  double prev_cmd_lateral_ = 0.0;   // 上一帧实际下发的侧向速度
  rclcpp::Time prev_cmd_time_;      // 上一帧下发时间
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}