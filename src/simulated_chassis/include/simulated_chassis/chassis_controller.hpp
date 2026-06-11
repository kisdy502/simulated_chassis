#ifndef SIMULATED_CHASSIS_CHASSIS_CONTROLLER_HPP_
#define SIMULATED_CHASSIS_CHASSIS_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace simulated_chassis {

class ChassisController {
public:
  explicit ChassisController(rclcpp::Node::SharedPtr node);
  ~ChassisController() = default;

  void setTargetVelocity(const geometry_msgs::msg::Twist & cmd);
  void update();

private:
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Twist current_velocity_;
};

}  // namespace simulated_chassis

#endif  // SIMULATED_CHASSIS_CHASSIS_CONTROLLER_HPP_
