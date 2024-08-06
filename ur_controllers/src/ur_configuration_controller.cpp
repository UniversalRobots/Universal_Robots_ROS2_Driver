// Copyright 2024, Universal Robots A/S
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
 * \date    2024-07-11
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#include <ur_controllers/ur_configuration_controller.hpp>
#include <realtime_tools/realtime_box.h>
namespace ur_controllers
{

controller_interface::CallbackReturn URConfigurationController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
URConfigurationController::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  param_listener_ = std::make_shared<ur_configuration_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();

  get_robot_software_version_srv_ = get_node()->create_service<ur_msgs::srv::GetRobotSoftwareVersion>(
      "~/get_robot_software_version", std::bind(&URConfigurationController::getRobotSoftwareVersion, this,
                                                std::placeholders::_1, std::placeholders::_2));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration URConfigurationController::command_interface_configuration() const
{
  // No command interfaces currently
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration URConfigurationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  config.names.emplace_back(tf_prefix + "get_robot_software_version/get_version_major");
  config.names.emplace_back(tf_prefix + "get_robot_software_version/get_version_minor");
  config.names.emplace_back(tf_prefix + "get_robot_software_version/get_version_build");
  config.names.emplace_back(tf_prefix + "get_robot_software_version/get_version_bugfix");

  return config;
}

controller_interface::return_type URConfigurationController::update(const rclcpp::Time& /* time */,
                                                                    const rclcpp::Duration& /* period */)
{
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
URConfigurationController::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  robot_software_version_.set([this](const std::shared_ptr<VersionInformation> ptr) {
    ptr->major = state_interfaces_[StateInterfaces::ROBOT_VERSION_MAJOR].get_value();
    ptr->minor = state_interfaces_[StateInterfaces::ROBOT_VERSION_MINOR].get_value();
    ptr->build = state_interfaces_[StateInterfaces::ROBOT_VERSION_BUILD].get_value();
    ptr->bugfix = state_interfaces_[StateInterfaces::ROBOT_VERSION_BUGFIX].get_value();
  });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
URConfigurationController::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool URConfigurationController::getRobotSoftwareVersion(
    ur_msgs::srv::GetRobotSoftwareVersion::Request::SharedPtr /*req*/,
    ur_msgs::srv::GetRobotSoftwareVersion::Response::SharedPtr resp)
{
  std::shared_ptr<VersionInformation> temp;
  return robot_software_version_.tryGet([resp](const std::shared_ptr<VersionInformation> ptr) {
    resp->major = ptr->major;
    resp->minor = ptr->minor;
    resp->build = ptr->build;
    resp->bugfix = ptr->bugfix;
  });
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::URConfigurationController, controller_interface::ControllerInterface)
