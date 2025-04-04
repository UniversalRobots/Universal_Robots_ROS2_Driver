// Copyright 2024, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_
#define UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_client_library/ur/datatypes.h"
#include "ur_client_library/primary/primary_client.h"

namespace ur_robot_driver
{
class RobotStateHelper
{
public:
  using SetModeGoalHandle = rclcpp_action::ServerGoalHandle<ur_dashboard_msgs::action::SetMode>;

  explicit RobotStateHelper(const rclcpp::Node::SharedPtr& node);
  RobotStateHelper() = delete;
  virtual ~RobotStateHelper() = default;

private:
  rclcpp::Node::SharedPtr node_;

  void robotModeCallback(ur_dashboard_msgs::msg::RobotMode::SharedPtr msg);
  void safetyModeCallback(ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg);

  void updateRobotState();

  bool recoverFromSafety();
  bool doTransition(const urcl::RobotMode target_mode);
  bool jumpToRobotMode(const urcl::RobotMode target_mode);

  bool safeDashboardTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv);

  bool stopProgram();

  void setModeAcceptCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle);
  rclcpp_action::GoalResponse setModeGoalCallback(const rclcpp_action::GoalUUID& uuid,
                                                  std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal);
  rclcpp_action::CancelResponse setModeCancelCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle);

  void setModeExecute(const std::shared_ptr<SetModeGoalHandle> goal_handle);

  bool headless_mode_;

  std::shared_ptr<ur_dashboard_msgs::action::SetMode::Result> result_;
  std::shared_ptr<ur_dashboard_msgs::action::SetMode::Feedback> feedback_;
  std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal_;
  std::shared_ptr<SetModeGoalHandle> current_goal_handle_;

  std::atomic<urcl::RobotMode> robot_mode_;
  std::atomic<urcl::SafetyMode> safety_mode_;
  std::atomic<bool> error_ = false;
  std::atomic<bool> in_action_;
  std::atomic<bool> program_running_;
  std::mutex goal_mutex_;

  std::string robot_ip_;
  urcl::comm::INotifier notifier_;
  std::shared_ptr<urcl::primary_interface::PrimaryClient> primary_client_;

  rclcpp_action::Server<ur_dashboard_msgs::action::SetMode>::SharedPtr set_mode_as_;

  rclcpp::CallbackGroup::SharedPtr robot_mode_sub_cb_;

  rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr safety_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr program_running_sub;

  rclcpp::CallbackGroup::SharedPtr service_cb_grp_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_safety_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_program_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_srv_;
};
}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__ROBOT_STATE_HELPER_HPP_
