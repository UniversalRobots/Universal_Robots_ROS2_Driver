#include "ur_robot_driver/robot_state_helper.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("robot_state_helper");
  ur_robot_driver::RobotStateHelper state_helper(node);

  rclcpp::spin(node);

  return 0;
}
