#include "simulated_chassis_ur5e/arm_move_adapter.hpp"

#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulated_chassis_ur5e
{
ArmMoveAdapter::ArmMoveAdapter(const rclcpp::NodeOptions & options)
: Node("arm_move_adapter", options),
  joints_{"ur5e_elbow_joint", "ur5e_shoulder_lift_joint", "ur5e_shoulder_pan_joint",
    "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"}
{
  // Index order is UR5e lexical order. A real 7-axis request is joint_1..7;
  // The extra upper-arm roll is joint_3; preserve flange rotation at joint_7.
  real_arm_indices_ = declare_parameter<std::vector<int64_t>>(
    "real_arm_joint_indices", {3, 1, 0, 4, 5, 6});
  if (real_arm_indices_.size() != joints_.size()) {
    throw std::runtime_error("real_arm_joint_indices must contain 6 unique indices in [0, 6]");
  }
  auto sorted = real_arm_indices_;
  std::sort(sorted.begin(), sorted.end());
  if (sorted.front() < 0 || sorted.back() > 6 ||
    std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
  {
    throw std::runtime_error("real_arm_joint_indices must contain 6 unique indices in [0, 6]");
  }
  urdf::Model model;
  if (!model.initString(declare_parameter<std::string>("robot_description", ""))) {
    throw std::runtime_error("arm_move_adapter requires the combined robot_description");
  }
  for (const auto & name : joints_) {
    const auto joint = model.getJoint(name);
    if (!joint || !joint->limits || !std::isfinite(joint->limits->lower) ||
      !std::isfinite(joint->limits->upper) || joint->limits->lower > joint->limits->upper ||
      !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0.0)
    {
      throw std::runtime_error("Missing or invalid URDF limits for " + name);
    }
    limits_.push_back({joint->limits->lower, joint->limits->upper, joint->limits->velocity});
  }
  joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {updateJoints(*message);});
  trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);
  controller_client_ = create_client<ListControllers>("/controller_manager/list_controllers");
  controller_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {pollController();});
  service_ = create_service<ArmMove>("/agv/arm_move",
    [this](const ArmMove::Request::SharedPtr request, ArmMove::Response::SharedPtr response) {
      move(*request, *response);
    });
  RCLCPP_INFO(get_logger(),
    "/agv/arm_move ready; lexical order: elbow, shoulder_lift, shoulder_pan, wrist_1, wrist_2, wrist_3");
}

void ArmMoveAdapter::updateJoints(const sensor_msgs::msg::JointState & message)
{
  if (message.name.size() != message.position.size()) return;
  std::vector<double> values;
  for (const auto & name : joints_) {
    const auto found = std::find(message.name.begin(), message.name.end(), name);
    if (found == message.name.end() || std::count(message.name.begin(), message.name.end(), name) != 1) return;
    const auto value = message.position[std::distance(message.name.begin(), found)];
    if (!std::isfinite(value)) return;
    values.push_back(value);
  }
  current_ = std::move(values);
  joint_received_ = Clock::now();
}

void ArmMoveAdapter::pollController()
{
  const auto now = Clock::now();
  if (controller_pending_) {
    if (now - controller_requested_ < std::chrono::seconds(2)) return;
    controller_client_->remove_pending_request(controller_request_id_);
    controller_pending_ = false;
    controller_active_ = false;
    ++controller_generation_;
  }
  if (!controller_client_->service_is_ready()) {
    controller_active_ = false;
    return;
  }
  controller_pending_ = true;
  controller_requested_ = now;
  const auto generation = ++controller_generation_;
  const auto request = controller_client_->async_send_request(std::make_shared<ListControllers::Request>(),
    [this, generation](rclcpp::Client<ListControllers>::SharedFuture future) {
      if (generation != controller_generation_) return;
      controller_pending_ = false;
      controller_active_ = false;
      for (const auto & controller : future.get()->controller) {
        if (controller.name == "joint_trajectory_controller" && controller.state == "active") {
          controller_active_ = true;
        }
      }
      controller_received_ = Clock::now();
    });
  controller_request_id_ = request.request_id;
}

void ArmMoveAdapter::move(const ArmMove::Request & request, ArmMove::Response & response)
{
  response.success = false;
  if (request.arm != "left" && request.arm != "right") {
    response.message = request.arm == "waist" ? "waist not equipped" : "invalid arm";
    return;
  }
  const bool real_arm = request.joints.size() == 7;
  if (request.joints.size() != joints_.size() && !real_arm) {
    response.message = "expected 6 or 7 joints";
    return;
  }
  if (!std::isfinite(request.duration_sec) ||
    !std::all_of(request.joints.begin(), request.joints.end(), [](double v) {return std::isfinite(v);}))
  {
    response.message = "joints and duration_sec must be finite";
    return;
  }
  const auto now = Clock::now();
  if (current_.size() != joints_.size() || now - joint_received_ > std::chrono::seconds(2)) {
    response.message = "joint states unavailable or stale";
    return;
  }
  if (!controller_active_ || now - controller_received_ > std::chrono::seconds(2) ||
    trajectory_publisher_->get_subscription_count() == 0)
  {
    response.message = "joint_trajectory_controller not ready";
    return;
  }
  double duration = std::clamp(request.duration_sec > 0.0 ? request.duration_sec : 3.0, 0.5, 300.0);
  std::vector<double> target;
  bool adjusted = false;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double value = request.joints[real_arm ? real_arm_indices_[i] : i];
    target.push_back(std::clamp(value, limits_[i].lower, limits_[i].upper));
    adjusted = adjusted || target.back() != value;
    // Positions only => JTC linear interpolation. Stretch duration rather than
    // assuming the position controller automatically enforces URDF speed limits.
    const double minimum = std::abs(target.back() - current_[i]) / limits_[i].velocity;
    if (minimum > duration) {
      duration = minimum;
      adjusted = true;
    }
  }
  if (duration > 300.0) {
    response.message = "motion requires more than 300 seconds within joint velocity limits";
    return;
  }
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joints_;
  trajectory_msgs::msg::JointTrajectoryPoint start, finish;
  start.positions = current_;
  finish.positions = target;
  finish.time_from_start = rclcpp::Duration::from_seconds(duration);
  trajectory.points = {start, finish};
  trajectory_publisher_->publish(trajectory);
  response.success = true;
  response.message = "accepted";
  if (real_arm) {
    for (int64_t index = 0; index < 7; ++index) {
      if (std::find(real_arm_indices_.begin(), real_arm_indices_.end(), index) == real_arm_indices_.end()) {
        response.message += " (UR5e alias; joint_" + std::to_string(index + 1) + " not simulated)";
      }
    }
  }
  RCLCPP_INFO(get_logger(), "arm_move[%s -> ur5e] accepted, duration=%.3fs%s",
    request.arm.c_str(), duration, adjusted ? " (target/duration adjusted to URDF limits)" : "");
}
}  // namespace simulated_chassis_ur5e
