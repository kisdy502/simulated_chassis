// Simulation-only parallel gripper: accepted service requests become JTC actions.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using Trajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<Trajectory>;
using Clock = std::chrono::steady_clock;

class GripperControl : public rclcpp::Node
{
public:
  GripperControl() : Node("gripper_control")
  {
    max_width_ = declare_parameter("max_width", 0.08);
    speed_ = declare_parameter("finger_speed", 0.04);
    duration_ = declare_parameter("command_duration", 2.0);
    if (!std::isfinite(max_width_) || max_width_ <= 0 || max_width_ > 0.08 ||
      !std::isfinite(speed_) || speed_ <= 0 || !std::isfinite(duration_) || duration_ <= 0)
    {
      throw std::invalid_argument("Invalid gripper dimensions/speed/duration");
    }
    width_pub_ = create_publisher<std_msgs::msg::Float64>("/gripper/width", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/gripper/status", rclcpp::QoS(1).transient_local());
    client_ = rclcpp_action::create_client<Trajectory>(this, "/gripper_controller/follow_joint_trajectory");
    joints_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & message) {
        std::array<double, 2> values{};
        for (std::size_t i = 0; i < names_.size(); ++i) {
          const auto found = std::find(message.name.begin(), message.name.end(), names_[i]);
          if (found == message.name.end()) {return;}
          const auto index = static_cast<std::size_t>(found - message.name.begin());
          if (index >= message.position.size() || !std::isfinite(message.position[index]) ||
            message.position[index] < -0.002 || message.position[index] > max_width_ / 2 + 0.002)
          {return;}
          values[i] = message.position[index];
        }
        positions_ = values;
        joint_time_ = Clock::now();
        have_state_ = true;
        std_msgs::msg::Float64 width;
        width.data = std::clamp(values[0] + values[1], 0.0, max_width_);
        width_pub_->publish(width);
        if (!busy_ && (status_ == "waiting" || status_ == "state_stale")) {status("idle");}
      });
    service_ = create_service<std_srvs::srv::SetBool>("/gripper/set_closed",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (!fresh()) {response->message = "No fresh measured finger state"; return;}
        if (busy_) {response->message = "Gripper command in progress"; return;}
        if (!client_->action_server_is_ready()) {response->message = "Gripper controller unavailable"; return;}
        const double target = request->data ? 0.0 : max_width_ / 2;
        const double delta = std::max(std::abs(target - positions_[0]), std::abs(target - positions_[1]));
        const double seconds = std::max(duration_, delta / speed_);
        Trajectory::Goal goal;
        goal.trajectory.joint_names.assign(names_.begin(), names_.end());
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(positions_.begin(), positions_.end());
        point.time_from_start = rclcpp::Duration::from_seconds(0.0);
        goal.trajectory.points.push_back(point);
        point.positions = {target, target};
        point.time_from_start = rclcpp::Duration::from_seconds(seconds);
        goal.trajectory.points.push_back(point);
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
        busy_ = true;
        deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds + 5.0));
        const auto command = ++generation_;
        rclcpp_action::Client<Trajectory>::SendGoalOptions options;
        options.goal_response_callback = [this, command](GoalHandle::SharedPtr handle) {
          if (command != generation_) {
            if (handle) {client_->async_cancel_goal(handle);}
            return;
          }
          handle_ = handle;
          if (!handle) {busy_ = false; status("rejected");}
        };
        options.result_callback = [this, command](const GoalHandle::WrappedResult & result) {
          if (command != generation_) {return;}
          busy_ = false;
          handle_.reset();
          status(result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            result.result->error_code == Trajectory::Result::SUCCESSFUL ? "succeeded" : "aborted");
        };
        try {
          status("moving");
          client_->async_send_goal(goal, options);
          response->success = true;
          response->message = "Accepted; observe /gripper/status and /gripper/width for execution";
        } catch (const std::exception & error) {
          busy_ = false;
          status("rejected");
          response->message = error.what();
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (busy_ && (Clock::now() > deadline_ || !fresh())) {
        if (handle_) {client_->async_cancel_goal(handle_);}
        handle_.reset();
        ++generation_;  // Ignore late results; cancel a late accepted goal in its callback.
        busy_ = false;
        status(fresh() ? "timeout" : "state_stale");
      } else if (!busy_ && !fresh()) {status("state_stale");}
    });
    status("waiting");
  }

private:
  bool fresh() const {return have_state_ && Clock::now() - joint_time_ < std::chrono::seconds(2);}
  void status(const std::string & value)
  {
    if (value == status_) {return;}
    status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }
  const std::array<std::string, 2> names_{"ur5e_gripper_left_joint", "ur5e_gripper_right_joint"};
  std::array<double, 2> positions_{};
  double max_width_, speed_, duration_;
  bool busy_{false}, have_state_{false};
  uint64_t generation_{0};
  Clock::time_point joint_time_{}, deadline_{};
  std::string status_;
  GoalHandle::SharedPtr handle_;
  rclcpp_action::Client<Trajectory>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr width_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperControl>());
  rclcpp::shutdown();
  return 0;
}
