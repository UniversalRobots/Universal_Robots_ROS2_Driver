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
 * \author  Felix Exner exner@fzi.de
 * \date    2023-06-29
 */
//----------------------------------------------------------------------

#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ur_controllers/force_mode_controller.hpp>
namespace ur_controllers
{
controller_interface::CallbackReturn ForceModeController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<force_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration ForceModeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  RCLCPP_DEBUG(get_node()->get_logger(), "Configure UR force_mode controller with tf_prefix: %s", tf_prefix.c_str());

  // Get all the command interfaces needed for force mode from the hardware interface
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_x");
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_y");
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_z");
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_rx");
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_ry");
  config.names.emplace_back(tf_prefix + "force_mode/task_frame_rz");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_x");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_y");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_z");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_rx");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_ry");
  config.names.emplace_back(tf_prefix + "force_mode/selection_vector_rz");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_x");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_y");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_z");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_rx");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_ry");
  config.names.emplace_back(tf_prefix + "force_mode/wrench_rz");
  config.names.emplace_back(tf_prefix + "force_mode/type");
  config.names.emplace_back(tf_prefix + "force_mode/limits_x");
  config.names.emplace_back(tf_prefix + "force_mode/limits_y");
  config.names.emplace_back(tf_prefix + "force_mode/limits_z");
  config.names.emplace_back(tf_prefix + "force_mode/limits_rx");
  config.names.emplace_back(tf_prefix + "force_mode/limits_ry");
  config.names.emplace_back(tf_prefix + "force_mode/limits_rz");
  config.names.emplace_back(tf_prefix + "force_mode/force_mode_async_success");
  config.names.emplace_back(tf_prefix + "force_mode/disable_cmd");
  config.names.emplace_back(tf_prefix + "force_mode/damping");
  config.names.emplace_back(tf_prefix + "force_mode/gain_scaling");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::ForceModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  // Get the state interface indicating whether the hardware interface has been initialized
  config.names.emplace_back(tf_prefix + "system_interface/initialized");

  return config;
}

controller_interface::return_type ur_controllers::ForceModeController::update(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  // Publish state of force_mode?

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_WARN(get_node()->get_logger(), "The ForceModeController is considered a beta feature. Its interface might "
                                        "change in the future.");
  while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Create the service server that will be used to start force mode
  try {
    set_force_mode_srv_ = get_node()->create_service<ur_msgs::srv::SetForceMode>(
        "~/start_force_mode",
        std::bind(&ForceModeController::setForceMode, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Stop force mode if this controller is deactivated.
  disableForceMode();
  try {
    set_force_mode_srv_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool ForceModeController::setForceMode(const ur_msgs::srv::SetForceMode::Request::SharedPtr req,
                                       ur_msgs::srv::SetForceMode::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  // transform task frame into base
  const std::string tf_prefix = params_.tf_prefix;
  try {
    auto task_frame_transformed = tf_buffer_->transform(req->task_frame, tf_prefix + "base");
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_X].set_value(task_frame_transformed.pose.position.x);
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_Y].set_value(task_frame_transformed.pose.position.y);
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_Z].set_value(task_frame_transformed.pose.position.z);

    tf2::Quaternion quat_tf;
    tf2::convert(task_frame_transformed.pose.orientation, quat_tf);
    tf2::Matrix3x3 rot_mat(quat_tf);
    std::vector<double> rot(3);
    rot_mat.getRPY(rot[0], rot[1], rot[2]);
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RX].set_value(rot[0]);
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RY].set_value(rot[1]);
    command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RZ].set_value(rot[2]);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not transform %s to robot base: %s",
                 req->task_frame.header.frame_id.c_str(), ex.what());
  }

  /* The selection vector dictates which axes the robot should be compliant along and around */
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_X].set_value(req->selection_vector_x);
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_Y].set_value(req->selection_vector_y);
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_Z].set_value(req->selection_vector_z);
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RX].set_value(req->selection_vector_rx);
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RY].set_value(req->selection_vector_ry);
  command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RZ].set_value(req->selection_vector_rz);

  /* The wrench parameters dictate the amount of force/torque the robot will apply to its environment. The robot will
   * move along/around compliant axes to match the specified force/torque. Has no effect for non-compliant axes. */
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_X].set_value(req->wrench.wrench.force.x);
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_Y].set_value(req->wrench.wrench.force.y);
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_Z].set_value(req->wrench.wrench.force.z);
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RX].set_value(req->wrench.wrench.torque.x);
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RY].set_value(req->wrench.wrench.torque.y);
  command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RZ].set_value(req->wrench.wrench.torque.z);

  /* The limits specifies the maximum allowed speed along/around compliant axes. For non-compliant axes this value is
   * the maximum allowed deviation between actual tcp position and the one that has been programmed. */
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_X].set_value(req->limits.twist.linear.x);
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_Y].set_value(req->limits.twist.linear.y);
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_Z].set_value(req->limits.twist.linear.z);
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RX].set_value(req->limits.twist.angular.x);
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RY].set_value(req->limits.twist.angular.y);
  command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RZ].set_value(req->limits.twist.angular.z);

  /* The type decides how the robot interprets the force frame (the one defined in task_frame). See ur_script manual for
   * explanation, under force_mode. */
  command_interfaces_[CommandInterfaces::FORCE_MODE_TYPE].set_value(unsigned(req->type));

  /* The damping factor decides how fast the robot decelarates if no force is present. 0 means no deceleration, 1 means
   * quick deceleration*/
  command_interfaces_[CommandInterfaces::FORCE_MODE_DAMPING].set_value(req->damping_factor);
  /*The gain scaling factor scales the force mode gain. A value larger than 1 may make force mode unstable. */
  command_interfaces_[CommandInterfaces::FORCE_MODE_GAIN_SCALING].set_value(req->gain_scaling);

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for force mode to be set.");
  const auto maximum_retries = params_.check_io_successful_retries;
  int retries = 0;
  while (command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries) {
      resp->success = false;
    }
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Force mode has been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the force mode.");
    return false;
  }

  return true;
}

bool ForceModeController::disableForceMode()
{
  command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::FORCE_MODE_DISABLE_CMD].set_value(1);

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for force mode to be disabled.");
  while (command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    // Asynchronous wait until the hardware interface has set the force mode
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  bool success = static_cast<bool>(command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].get_value());
  if (success) {
    RCLCPP_INFO(get_node()->get_logger(), "Force mode has been disabled successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not disable force mode.");
    return false;
  }
  return true;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ForceModeController, controller_interface::ControllerInterface)
