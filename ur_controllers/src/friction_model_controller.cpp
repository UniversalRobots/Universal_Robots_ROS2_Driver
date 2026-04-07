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

#include <algorithm>
#include <limits>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>

#include <ur_controllers/friction_model_controller.hpp>

namespace ur_controllers
{
controller_interface::CallbackReturn FrictionModelController::on_init()
{
  try {
    param_listener_ = std::make_shared<friction_model_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FrictionModelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // The order of the following has to match the CommandInterfaces enum!
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "friction_model/viscous_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "friction_model/coulomb_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "friction_model/async_success");

  return config;
}

controller_interface::InterfaceConfiguration FrictionModelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn
FrictionModelController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  FrictionModelParameters initial_parameters;
  initial_parameters.viscous_scale = {
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()
  };

  initial_parameters.coulomb_scale = {
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()
  };

  friction_model_params_buffer_.set(initial_parameters);

  try {
    set_friction_model_parameters_srv_ = get_node()->create_service<ur_msgs::srv::SetFrictionModelParameters>(
        "~/set_friction_model_parameters", std::bind(&FrictionModelController::setFrictionModelParameters, this,
                                                     std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FrictionModelController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  change_requested_ = false;
  async_result_ = false;
  async_state_ = std::numeric_limits<double>::quiet_NaN();

  auto current_params = friction_model_params_buffer_.get();
  if (std::all_of(current_params.viscous_scale.begin(), current_params.viscous_scale.end(),
                  [](double v) { return std::isfinite(v); }) &&
      std::all_of(current_params.coulomb_scale.begin(), current_params.coulomb_scale.end(),
                  [](double v) { return std::isfinite(v); })) {
    RCLCPP_INFO(get_node()->get_logger(), "Restoring friction model parameters from before deactivation. If this is "
                                          "not desired, please set new parameters using the service.");
    change_requested_ = true;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FrictionModelController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FrictionModelController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  set_friction_model_parameters_srv_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrictionModelController::update(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  if (async_state_ == ASYNC_WAITING) {
    async_state_ =
        command_interfaces_[friction_model::FRICTION_MODEL_ASYNC_SUCCESS].get_optional().value_or(ASYNC_WAITING);
    if (async_state_ != ASYNC_WAITING) {
      async_result_ = async_state_ == 1.0;
    }
  }

  if (change_requested_) {
    const auto params = friction_model_params_buffer_.try_get();
    if (!params) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get friction model parameters from realtime buffer. Retrying in "
                                            "next update cycle.");
      return controller_interface::return_type::OK;
    }

    bool write_successful = true;
    for (size_t i = 0; i < 6; ++i) {
      write_successful &=
          command_interfaces_[friction_model::FRICTION_MODEL_VISCOUS_0 + i].set_value(params->viscous_scale[i]);
    }
    for (size_t i = 0; i < 6; ++i) {
      write_successful &=
          command_interfaces_[friction_model::FRICTION_MODEL_COULOMB_0 + i].set_value(params->coulomb_scale[i]);
    }

    write_successful &= command_interfaces_[friction_model::FRICTION_MODEL_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    async_state_ = ASYNC_WAITING;
    async_result_ = false;

    if (!write_successful) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not write to command interfaces.");
      return controller_interface::return_type::ERROR;
    }
    change_requested_ = false;
  }

  return controller_interface::return_type::OK;
}

bool FrictionModelController::setFrictionModelParameters(
    const ur_msgs::srv::SetFrictionModelParameters::Request::SharedPtr req,
    ur_msgs::srv::SetFrictionModelParameters::Response::SharedPtr resp)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new requests. Controller is not running.");
    resp->success = false;
    return false;
  }

  if (req->parameters.viscous_scale.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "viscous_scale must have exactly 6 elements, got %zu.",
                 req->parameters.viscous_scale.size());
    resp->success = false;
    return false;
  }

  if (req->parameters.coulomb_scale.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "coulomb_scale must have exactly 6 elements, got %zu.",
                 req->parameters.coulomb_scale.size());
    resp->success = false;
    return false;
  }

  FrictionModelParameters friction_model_parameters;
  std::copy(req->parameters.viscous_scale.begin(), req->parameters.viscous_scale.end(),
            friction_model_parameters.viscous_scale.begin());
  std::copy(req->parameters.coulomb_scale.begin(), req->parameters.coulomb_scale.end(),
            friction_model_parameters.coulomb_scale.begin());

  friction_model_params_buffer_.set(friction_model_parameters);

  change_requested_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for friction model parameters to be set.");
  if (!waitForAsyncCommand([&]() { return async_result_.load(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that friction model parameters were set. (This might "
                                          "happen when using the mocked interface)");
  }

  resp->success = async_result_.load();

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Friction model parameters have been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the friction model parameters.");
    return false;
  }

  return true;
}

bool FrictionModelController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = params_.check_io_successful_retries;
  int retries = 0;
  while (get_value() == false) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries)
      return false;
  }
  return true;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::FrictionModelController, controller_interface::ControllerInterface)
