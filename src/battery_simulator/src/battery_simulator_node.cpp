#include "battery_simulator/model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>

using namespace std::chrono_literals;
class BatterySimulator : public rclcpp::Node {
 public:
  BatterySimulator() : Node("battery_simulator") {
    model_.soc = declare_parameter("initial_soc", 80.0);
    model_.idle_rate = declare_parameter("idle_discharge_pct_per_hour", 12.0);
    model_.moving_rate = declare_parameter("moving_discharge_pct_per_hour", 30.0);
    model_.charge_rate = declare_parameter("charge_pct_per_hour", 60.0);
    capacity_ = declare_parameter("capacity_ah", 40.0);
    voltage_ = declare_parameter("nominal_voltage", 48.0);
    file_ = declare_parameter<std::string>("state_file", "~/.ros/battery_simulator/soc.state");
    model_.validate();
    if (!std::isfinite(capacity_) || capacity_ <= 0 || !std::isfinite(voltage_) || voltage_ <= 0)
      throw std::invalid_argument("capacity_ah and nominal_voltage must be finite and positive");
    if (file_.rfind("~/", 0) == 0) {
      const auto home = std::getenv("HOME");
      if (!home) throw std::runtime_error("HOME is required for state_file");
      file_ = std::string(home) + file_.substr(1);
    }
    if (!file_.empty() && std::filesystem::exists(file_)) {
      std::ifstream input(file_); double saved;
      if (!(input >> saved) || !std::isfinite(saved) || saved < 0 || saved > 100)
        throw std::runtime_error("Invalid saved SOC; inspect state_file: " + file_);
      model_.soc = saved;
    }
    battery_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
    connected_ = create_publisher<std_msgs::msg::Bool>("/battery/charging_connected", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &v = msg->twist.twist;
        moving_ = !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) ||
          !std::isfinite(v.angular.z) || std::hypot(v.linear.x, v.linear.y) > 0.02 ||
          std::abs(v.angular.z) > 0.02;
        last_odom_ = std::chrono::steady_clock::now();
        if (moving_ && model_.connected) { model_.connected = false; publish(); }
      });
    cmd_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (std::abs(msg->linear.x) > 0.001 || std::abs(msg->linear.y) > 0.001 ||
            std::abs(msg->angular.z) > 0.001) {
          if (model_.connected) { model_.connected = false; publish(); }
          last_motion_command_ = std::chrono::steady_clock::now();
        }
      });
    service_ = create_service<std_srvs::srv::SetBool>("/battery/set_charging",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        tick();
        const auto now = std::chrono::steady_clock::now();
        if (req->data && (moving_ || now - last_odom_ > 2s || now - last_motion_command_ < 1s)) {
          res->message = "不能开始充电：机器人仍在运动、最近收到移动指令或 /odom 超过2秒未更新；请先到达充电点并停车";
          return;
        }
        model_.connected = req->data;
        publish(); save();
        res->success = true;
        res->message = req->data ? "仿真充电连接已建立，电量将随仿真时间增加" : "仿真充电连接已断开，已转为放电";
      });
    last_ros_ = now().seconds();
    timer_ = create_wall_timer(1s, [this] { tick(); publish(); });
    saver_ = create_wall_timer(10s, [this] { save(); });
    // Never restore a charging connection automatically after a process restart.
    publish();
  }
  ~BatterySimulator() override { save(); }
 private:
  void tick() {
    const double time = now().seconds();
    const double dt = time - last_ros_; last_ros_ = time;
    model_.advance(dt, moving_);
  }
  void publish() {
    using B = sensor_msgs::msg::BatteryState;
    B msg; msg.header.stamp = now(); msg.header.frame_id = "base_link";
    msg.percentage = model_.soc / 100.0;
    msg.capacity = msg.design_capacity = capacity_;
    msg.charge = capacity_ * msg.percentage;
    msg.voltage = voltage_;
    msg.current = capacity_ * model_.rate(moving_) / 100.0;
    msg.present = true;
    msg.power_supply_status = model_.connected ?
      (model_.soc >= 100.0 ? B::POWER_SUPPLY_STATUS_FULL : B::POWER_SUPPLY_STATUS_CHARGING) :
      B::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_->publish(msg);
    std_msgs::msg::Bool contact; contact.data = model_.connected; connected_->publish(contact);
  }
  void save() noexcept {
    if (file_.empty()) return;
    try {
      const std::filesystem::path path(file_);
      if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
      std::ofstream output(file_ + ".tmp");
      output << std::setprecision(17) << model_.soc << '\n'; output.close();
      if (!output) throw std::runtime_error("write failed");
      std::filesystem::rename(file_ + ".tmp", file_);
    } catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "SOC 保存失败: %s", e.what()); }
  }
  battery_simulator::Model model_;
  double capacity_, voltage_, last_ros_ = 0;
  bool moving_ = true;
  std::string file_;
  std::chrono::steady_clock::time_point last_odom_{}, last_motion_command_{};
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_, saver_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<BatterySimulator>()); }
  catch (const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("battery_simulator"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
