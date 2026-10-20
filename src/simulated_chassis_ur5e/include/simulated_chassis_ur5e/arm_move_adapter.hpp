#pragma once

#include <agv_bridge_v2_interfaces/srv/arm_move.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace simulated_chassis_ur5e
{
class ArmMoveAdapter : public rclcpp::Node
{
public:
  explicit ArmMoveAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Clock = std::chrono::steady_clock;
  using ArmMove = agv_bridge_v2_interfaces::srv::ArmMove;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  struct Limit {double lower; double upper; double velocity;};
  void updateJoints(const sensor_msgs::msg::JointState & message);
  void pollController();
  void move(const ArmMove::Request & request, ArmMove::Response & response);

  std::vector<std::string> joints_;
  std::vector<int64_t> real_arm_indices_;
  std::vector<Limit> limits_;
  std::vector<double> current_;
  Clock::time_point joint_received_{};
  Clock::time_point controller_received_{};
  Clock::time_point controller_requested_{};
  bool controller_active_{false};
  bool controller_pending_{false};
  int64_t controller_request_id_{0};
  uint64_t controller_generation_{0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Service<ArmMove>::SharedPtr service_;
  rclcpp::Client<ListControllers>::SharedPtr controller_client_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
};
}  // namespace simulated_chassis_ur5e
