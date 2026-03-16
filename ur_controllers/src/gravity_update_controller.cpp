// Copyright (c) 2026 PickNik LLC
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
 * \author  Adam Pettinger adam.l.pettinger@gmail.com
 * \date    2026-03-13
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/gravity_update_controller.hpp"

#include <string>

namespace ur_controllers
{
controller_interface::CallbackReturn GravityUpdateController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<gravity_update_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityUpdateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // Gravity stuff
  config.names.emplace_back(tf_prefix + "gravity/x");
  config.names.emplace_back(tf_prefix + "gravity/y");
  config.names.emplace_back(tf_prefix + "gravity/z");
  config.names.emplace_back(tf_prefix + "gravity/gravity_async_success");

  return config;
}

controller_interface::InterfaceConfiguration
ur_controllers::GravityUpdateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

controller_interface::return_type ur_controllers::GravityUpdateController::update(const rclcpp::Time& /*time*/,
                                                                                  const rclcpp::Duration& /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ur_controllers::GravityUpdateController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::GravityUpdateController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    set_gravity_srv_ = get_node()->create_service<ur_msgs::srv::SetGravity>(
        "~/set_gravity",
        std::bind(&GravityUpdateController::setGravity, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::GravityUpdateController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    set_gravity_srv_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool GravityUpdateController::setGravity(const ur_msgs::srv::SetGravity::Request::SharedPtr req,
                                         ur_msgs::srv::SetGravity::Response::SharedPtr resp)
{
  // Check transform
  const std::string base_frame_name = params_.tf_prefix + "base";
  geometry_msgs::msg::TransformStamped tf_to_base_link;
  try {
    tf_to_base_link = tf_buffer_->lookupTransform(base_frame_name, req->gravity.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    resp->success = false;
    resp->status = ex.what();
    return false;
  }

  // The passed gravity vector is the direction of gravity (towards the Earth)
  // But the UR Client Library call is expecting anti-gravity (away from Earth), so negate here
  tf2::Vector3 anti_gravity(-1 * req->gravity.vector.x, -1 * req->gravity.vector.y, -1 * req->gravity.vector.z);

  // Rotate the gravity vector
  tf2::Quaternion quat;
  tf2::fromMsg(tf_to_base_link.transform.rotation, quat);
  tf2::Vector3 transformed_gravity = tf2::quatRotate(quat, anti_gravity);

  // reset success flag
  std::ignore = command_interfaces_[CommandInterfaces::GRAVITY_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  std::ignore = command_interfaces_[CommandInterfaces::GRAVITY_X].set_value(transformed_gravity.x());
  std::ignore = command_interfaces_[CommandInterfaces::GRAVITY_Y].set_value(transformed_gravity.y());
  std::ignore = command_interfaces_[CommandInterfaces::GRAVITY_Z].set_value(transformed_gravity.z());

  if (!waitForAsyncCommand([&]() {
        return command_interfaces_[CommandInterfaces::GRAVITY_ASYNC_SUCCESS].get_optional().value_or(ASYNC_WAITING);
      })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that gravity was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(
      command_interfaces_[CommandInterfaces::GRAVITY_ASYNC_SUCCESS].get_optional().value_or(ASYNC_WAITING));

  if (resp->success) {
    resp->status = "Gravity has been set successfully";
  } else {
    resp->status = "Could not set the gravity";
  }

  return resp->success;
}

bool GravityUpdateController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = params_.check_io_successfull_retries;
  int retries = 0;
  while (get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries)
      return false;
  }
  return true;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::GravityUpdateController, controller_interface::ControllerInterface)
