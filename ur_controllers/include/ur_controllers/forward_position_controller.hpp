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
 * \date    2024-03-11
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_
#define UR_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_

#include <memory>
// #include "controller_interface/controller_interface.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "forward_position_controller_parameters.hpp"

namespace forward_controller
{
class ForwardPositionController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ForwardPositionController() = default;
  ~ForwardPositionController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  void start_interpolation();

private:
  std::shared_ptr<forward_position_controller::ParamListener> forward_param_listener_;
  forward_position_controller::Params forward_params_;
};
}  // namespace forward_controller
#endif  // UR_CONTROLLERS__FORWARD_POSITION_CONTROLLER_HPP_
