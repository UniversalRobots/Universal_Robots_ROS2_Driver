// Copyright 2025, Universal Robots A/S
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2025-01-07
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__TOOL_CONTACT_CONTROLLER_HPP_
#define UR_CONTROLLERS__TOOL_CONTACT_CONTROLLER_HPP_

#include <chrono>

#include <controller_interface/chainable_controller_interface.hpp>
#include "std_msgs/msg/bool.hpp"
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/duration.hpp>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

#include <ur_msgs/action/tool_contact.hpp>
#include "tool_contact_controller_parameters.hpp"

namespace ur_controllers
{
class ToolContactController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  /// Virtual method that each chainable controller should implement to export its read-only
  /// chainable interfaces.
  /**
   * Each chainable controller implements this methods where all its state(read only) interfaces are
   * exported. The method has the same meaning as `export_state_interfaces` method from
   * hardware_interface::SystemInterface or hardware_interface::ActuatorInterface.
   *
   * \returns list of StateInterfaces that other controller can use as their inputs.
   */
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  /// Virtual method that each chainable controller should implement to export its read/write
  /// chainable interfaces.
  /**
   * Each chainable controller implements this methods where all input (command) interfaces are
   * exported. The method has the same meaning as `export_command_interface` method from
   * hardware_interface::SystemInterface or hardware_interface::ActuatorInterface.
   *
   * \returns list of CommandInterfaces that other controller can use as their outputs.
   */
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  /// Virtual method that each chainable controller should implement to switch chained mode.
  /**
   * Each chainable controller implements this methods to switch between "chained" and "external"
   * mode. In "chained" mode all external interfaces like subscriber and service servers are
   * disabled to avoid potential concurrency in input commands.
   *
   * \param[in] flag marking a switch to or from chained mode.
   *
   * \returns true if controller successfully switched between "chained" and "external" mode.
   * \default returns true so the method don't have to be overridden if controller can always switch
   * chained mode.
   */
  bool on_set_chained_mode(bool chained_mode) override;

  /// Update reference from input topics when not in chained mode.
  /**
   * Each chainable controller implements this method to update reference from subscribers when not
   * in chained mode.
   *
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;

  /// Execute calculations of the controller and update command interfaces.
  /**
   * Update method for chainable controllers.
   * In this method is valid to assume that \reference_interfaces_ hold the values for calculation
   * of the commands in the current control step.
   * This means that this method is called after \update_reference_from_subscribers if controller is
   * not in chained mode.
   *
   * \returns return_type::OK if calculation and writing of interface is successfully, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

private:
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ur_msgs::action::ToolContact>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  RealtimeGoalHandleBuffer rt_active_goal_;         ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;  ///< Timer to frequently check on the running goal
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(std::chrono::milliseconds(50));

  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID& /*uuid*/,
                                                     std::shared_ptr<const ur_msgs::action::ToolContact::Goal> goal);

  void
  goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::ToolContact>> goal_handle);

  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::ToolContact>> goal_handle);

  void failed_update();

  double tool_contact_enable;
  double tool_contact_active;

  std::atomic<bool> tool_contact_active_ = false;
  std::atomic<bool> change_requested_ = false;
  std::atomic<bool> logged_once_ = false;

  double old_reference_val = 0.0;

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> tool_contact_result_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> tool_contact_status_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> reference_interface_;

  rclcpp_action::Server<ur_msgs::action::ToolContact>::SharedPtr tool_contact_action_server_;

  std::shared_ptr<tool_contact_controller::ParamListener> tool_contact_param_listener_;
  tool_contact_controller::Params tool_contact_params_;

  static constexpr double TOOL_CONTACT_STANDBY = 1.0;
  static constexpr double TOOL_CONTACT_WAITING_BEGIN = 2.0;
  static constexpr double TOOL_CONTACT_EXECUTING = 3.0;
  static constexpr double TOOL_CONTACT_FAILURE_BEGIN = 4.0;
  static constexpr double TOOL_CONTACT_WAITING_END = 5.0;
  static constexpr double TOOL_CONTACT_SUCCESS_END = 6.0;
  static constexpr double TOOL_CONTACT_FAILURE_END = 7.0;
};
}  // namespace ur_controllers

#endif