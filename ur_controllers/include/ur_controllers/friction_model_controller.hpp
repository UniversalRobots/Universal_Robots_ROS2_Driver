// Copyright 2026, Universal Robots A/S
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
 * \author  Rune Søe-Knudsen rsk@universal-robots.com
 * \date    2026-03-02
 *
 */
//----------------------------------------------------------------------

#pragma once
#include <array>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_box_best_effort.hpp>
#include <ur_msgs/srv/set_friction_model_parameters.hpp>

#include "ur_controllers/friction_model_controller_parameters.hpp"

namespace ur_controllers
{
namespace friction_model
{
enum CommandInterfaces
{
  FRICTION_MODEL_VISCOUS_0 = 0u,
  FRICTION_MODEL_VISCOUS_1 = 1,
  FRICTION_MODEL_VISCOUS_2 = 2,
  FRICTION_MODEL_VISCOUS_3 = 3,
  FRICTION_MODEL_VISCOUS_4 = 4,
  FRICTION_MODEL_VISCOUS_5 = 5,
  FRICTION_MODEL_COULOMB_0 = 6,
  FRICTION_MODEL_COULOMB_1 = 7,
  FRICTION_MODEL_COULOMB_2 = 8,
  FRICTION_MODEL_COULOMB_3 = 9,
  FRICTION_MODEL_COULOMB_4 = 10,
  FRICTION_MODEL_COULOMB_5 = 11,
  FRICTION_MODEL_ASYNC_SUCCESS = 12,
};
}  // namespace friction_model

struct FrictionModelParameters
{
  std::array<double, 6> viscous_scale;
  std::array<double, 6> coulomb_scale;
};

class FrictionModelController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
  bool setFrictionModelParameters(const ur_msgs::srv::SetFrictionModelParameters::Request::SharedPtr req,
                                  ur_msgs::srv::SetFrictionModelParameters::Response::SharedPtr resp);

  rclcpp::Service<ur_msgs::srv::SetFrictionModelParameters>::SharedPtr set_friction_model_parameters_srv_;

  std::shared_ptr<friction_model_controller::ParamListener> param_listener_;
  friction_model_controller::Params params_;

  realtime_tools::RealtimeBoxBestEffort<FrictionModelParameters> friction_model_params_buffer_;
  std::atomic<bool> change_requested_;
  std::atomic<double> async_state_;
  std::atomic<bool> async_result_;

  static constexpr double ASYNC_WAITING = 2.0;
  bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace ur_controllers
