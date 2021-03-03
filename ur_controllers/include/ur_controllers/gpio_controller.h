// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-17
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_TOOLIOCONTROLLER_H
#define UR_ROBOT_DRIVER_TOOLIOCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ur_controllers
{
class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type init(const std::string& controller_name) override;

  controller_interface::return_type update() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
};
}  // namespace ur_controllers

#endif  // UR_ROBOT_DRIVER_TOOLIOCONTROLLER_H
