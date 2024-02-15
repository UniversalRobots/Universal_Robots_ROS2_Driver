#ifndef UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_
#define UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_client_library/ur/datatypes.h"

namespace ur_robot_driver
{
class RobotStateHelper
{
public:
  using SetModeGoalHandle = rclcpp_action::ServerGoalHandle<ur_dashboard_msgs::action::SetMode>;

  RobotStateHelper(const rclcpp::Node::SharedPtr& node);
  RobotStateHelper() = delete;
  virtual ~RobotStateHelper() = default;

private:
  rclcpp::Node::SharedPtr node_;

  void robotModeCallback(ur_dashboard_msgs::msg::RobotMode::SharedPtr msg);
  void safetyModeCallback(ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg);

  void updateRobotState(bool called_from_thread = false);

  void doTransition(bool called_from_thread = false);

  bool safeDashboardTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv, bool called_from_thread = false);

  void setModeAcceptCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle);
  rclcpp_action::GoalResponse setModeGoalCallback(const rclcpp_action::GoalUUID& uuid,
                                                  std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal);
  rclcpp_action::CancelResponse
  setModeCancelCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle);

  void setModeExecute(const std::shared_ptr<SetModeGoalHandle> goal_handle);

  void startActionServer();
  bool is_started_;

  std::shared_ptr<ur_dashboard_msgs::action::SetMode::Result> result_;
  std::shared_ptr<ur_dashboard_msgs::action::SetMode::Feedback> feedback_;
  std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal_;
  std::shared_ptr<SetModeGoalHandle> current_goal_handle_;

  urcl::RobotMode robot_mode_;
  urcl::SafetyMode safety_mode_;

  rclcpp_action::Server<ur_dashboard_msgs::action::SetMode>::SharedPtr set_mode_as_;

  rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr safety_mode_sub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_safety_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr brake_release_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_program_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_program_srv_;
};
}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_
