#include "simulated_chassis_ur5e/arm_move_adapter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<simulated_chassis_ur5e::ArmMoveAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_move_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
