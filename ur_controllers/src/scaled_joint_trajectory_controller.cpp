// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------

#include <memory>

#include "ur_controllers/scaled_joint_trajectory_controller.hpp"

namespace ur_controllers
{

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  // Translate speed scaling state interface parameter
  scaled_param_listener_ = std::make_shared<scaled_joint_trajectory_controller::ParamListener>(get_node());
  scaled_params_ = scaled_param_listener_->get_params();
  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    get_node()->declare_parameter("speed_scaling.state_interface", scaled_params_.speed_scaling_interface_name);
    get_node()->set_parameter(
        rclcpp::Parameter("speed_scaling.state_interface", scaled_params_.speed_scaling_interface_name));
  }
  return JointTrajectoryController::on_init();
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
