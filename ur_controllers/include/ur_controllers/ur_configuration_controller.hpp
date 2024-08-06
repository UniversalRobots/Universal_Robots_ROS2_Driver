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

#ifndef UR_CONTROLLERS__UR_CONFIGURATION_CONTROLLER_HPP_
#define UR_CONTROLLERS__UR_CONFIGURATION_CONTROLLER_HPP_

// TODO(fmauch): Currently, the realtime_box_best_effort doesn't include this
#include <functional>
#include <realtime_tools/realtime_box_best_effort.h>  // NOLINT

#include <memory>

#include <controller_interface/controller_interface.hpp>

#include "ur_msgs/srv/get_robot_software_version.hpp"
#include "ur_configuration_controller_parameters.hpp"

namespace ur_controllers
{

// Struct to hold version information
struct VersionInformation
{
  uint32_t major = 0, minor = 0, build = 0, bugfix = 0;
};

// Enum for indexing into state interfaces.
enum StateInterfaces
{
  ROBOT_VERSION_MAJOR = 0,
  ROBOT_VERSION_MINOR = 1,
  ROBOT_VERSION_BUILD = 2,
  ROBOT_VERSION_BUGFIX = 3,
};

class URConfigurationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  realtime_tools::RealtimeBoxBestEffort<std::shared_ptr<VersionInformation>> robot_software_version_{
    std::make_shared<VersionInformation>()
  };

  rclcpp::Service<ur_msgs::srv::GetRobotSoftwareVersion>::SharedPtr get_robot_software_version_srv_;

  bool getRobotSoftwareVersion(ur_msgs::srv::GetRobotSoftwareVersion::Request::SharedPtr req,
                               ur_msgs::srv::GetRobotSoftwareVersion::Response::SharedPtr resp);

  std::shared_ptr<ur_configuration_controller::ParamListener> param_listener_;
  ur_configuration_controller::Params params_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__UR_CONFIGURATION_CONTROLLER_HPP_
