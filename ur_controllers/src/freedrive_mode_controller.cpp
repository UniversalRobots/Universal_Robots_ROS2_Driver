
// Copyright 2023, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
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
 * \author  Vincenzo Di Pentima dipentima@fzi.de
 * \date    2024-09-16
 */
//----------------------------------------------------------------------

#include <rclcpp/logging.hpp>

#include <ur_controllers/freedrive_mode_controller.hpp>
namespace ur_controllers
{
controller_interface::CallbackReturn FreedriveModeController::on_init()
{
  // Even if the only param I use is tf_prefix, I still need it
  try {
    // Create the parameter listener and get the parameters
    freedrive_param_listener_ = std::make_shared<freedrive_mode_controller::ParamListener>(get_node());
    freedrive_params_ = freedrive_param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration FreedriveModeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = freedrive_params_.tf_prefix;

  auto joint_names = freedrive_params_.joints;
  for (auto& joint_name : joint_names) {
    config.names.emplace_back(joint_name + "/position"); //hardware_interface::HW_IF_POSITION
    config.names.emplace_back(joint_name + "/velocity"); //hardware_interface::HW_IF_VELOCITY
  }

  config.names.push_back(tf_prefix + "freedrive_mode_controller/freedrive_mode_abort");

  // Get the command interfaces needed for freedrive mode from the hardware interface
  // config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_mode_async_success");
  // config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_mode_cmd");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::FreedriveModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // I'm not sure I need these interfaces
  std::copy(joint_state_interface_names_.cbegin(), joint_state_interface_names_.cend(), std::back_inserter(config.names));

  // This doesn't exist for me, so I will comment it for now
  //config.names.push_back(freedrive_params_.speed_scaling_interface_name);

  return config;
}

controller_interface::return_type ur_controllers::FreedriveModeController::update(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  const auto active_goal = *rt_active_goal_.readFromRT();

  if (active_goal && trajectory_active_) {
    // Check if the freedrive mode has been aborted from the hardware interface. E.g. the robot was stopped on the teach
    // pendant.
    if (abort_command_interface_->get().get_value() == 1.0) {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory aborted by hardware, aborting action.");
      std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
          std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      active_goal->setAborted(result);
      end_goal();
      return controller_interface::return_type::OK;
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_configure(const rclcpp_lifecycle::State& previous_state)
{

  start_action_server();
  freedrive_active_ = false;

  const auto logger = get_node()->get_logger();

  if (!freedrive_param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  freedrive_param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  freedrive_params_ = /freedrive_param_listener_->get_params();

  // Joint interfaces handling
  joint_state_interface_names_.clear();

  joint_state_interface_names_.reserve(number_of_joints_ * state_interface_types_.size());

  auto joint_names_internal = joint_names_.readFromRT();
  for (const auto& joint_name : *joint_names_internal) {
    for (const auto& interface_type : state_interface_types_) {
      joint_state_interface_names_.emplace_back(joint_name + "/" + interface_type);
    }
  }
  return ControllerInterface::on_configure(previous_state);
}

void FreedriveModeController::start_action_server(void)
{
  freedrive_mode_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      get_node(), std::string(get_node()->get_name()) + "/freedrive_mode",
      std::bind(&FreedriveModeController::goal_received_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&FreedriveModeController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&FreedriveModeController::goal_accepted_callback, this, std::placeholders::_1));
  return;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_activate(const rclcpp_lifecycle::State& state)
{

  // clear out vectors in case of restart
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  for (auto& interface_name : joint_state_interface_names_) {
    auto interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                     [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (interface_it != state_interfaces_.end()) {
      if (interface_it->get_interface_name() == "position") {
        joint_position_state_interface_.emplace_back(*interface_it);

      } else if (interface_it->get_interface_name() == "velocity") {
        joint_velocity_state_interface_.emplace_back(*interface_it);
      }
    }
  }

  {
    const std::string interface_name = freedrive_params_.tf_prefix + "freedrive_mode_controller/"
                                                                       "freedrive_mode_abort";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      abort_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_deactivate(const rclcpp_lifecycle::State&)
{
  abort_command_interface_->get().set_value(1.0);
  if (freedrive_active_) {
    const auto active_goal = *rt_active_goal_.readFromRT();
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
        std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_string("Deactivating freedrive mode, since the controller is being deactivated.");
    active_goal->setAborted(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    end_goal();
  }
  return CallbackReturn::SUCCESS;
}

void FreedriveModeController::readFreedriveModeCmd(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Process the freedrive_mode command.
  if(msg->data)
  {
    freedrive_mode_enable_ =  true;
    RCLCPP_INFO(get_node()->get_logger(), "Received command to start Freedrive Mode.");
  } else{
    freedrive_mode_enable_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Received command to stop Freedrive Mode.");
  }
}

bool FreedriveModeController::enableFreedriveMode()
{
  // reset success flag
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  // Shouldn't I have a command set to 1 start it? Like it happens for the disable

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be set.");
  const auto maximum_retries = freedrive_params_.check_io_successful_retries;
  int retries = 0;
  while (command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries) {
      resp->success = false;
    }
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the freedrive mode.");
    return false;
  }

  return true;
}

bool FreedriveModeController::disableFreedriveMode()
{
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_CMD].set_value(1);

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be disabled.");
  while (command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    // Asynchronous wait until the hardware interface has set the freedrive mode
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  bool success = static_cast<bool>(command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value());
  if (success) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been disabled successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not disable freedrive mode.");
    return false;
  }
  return true;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::FreedriveModeController, controller_interface::ControllerInterface)
find_package(cartesian_controllers REQUIRED)
