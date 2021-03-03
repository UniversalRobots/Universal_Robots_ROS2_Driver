// Copyright (c) 2021 PickNik LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ur_controllers/tool_io_controller.h"

namespace ur_controllers
{
controller_interface::InterfaceConfiguration ur_controllers::ToolIOController::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration();
}

controller_interface::InterfaceConfiguration ur_controllers::ToolIOController::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration();
}

controller_interface::return_type ur_controllers::ToolIOController::init(const std::string &controller_name) {
    return ControllerInterface::init(controller_name);
}

controller_interface::return_type ur_controllers::ToolIOController::update() {
    return controller_interface::return_type::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::ToolIOController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    return LifecycleNodeInterface::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::ToolIOController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return LifecycleNodeInterface::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::ToolIOController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return LifecycleNodeInterface::on_deactivate(previous_state);
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ToolIOController, controller_interface::ControllerInterface)

