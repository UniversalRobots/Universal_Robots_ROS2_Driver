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

#include <limits>
#include <lifecycle_msgs/msg/state.hpp>
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

  // Create the service server that will be used to start force mode
  try {
    set_force_mode_srv_ = get_node()->create_service<ur_msgs::srv::SetForceMode>(
        "~/start_force_mode",
        std::bind(&ForceModeController::setForceMode, this, std::placeholders::_1, std::placeholders::_2));
    disable_force_mode_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/stop_force_mode",
        std::bind(&ForceModeController::disableForceMode, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  change_requested_ = false;
  force_mode_active_ = false;
  async_state_ = std::numeric_limits<double>::quiet_NaN();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Stop force mode if this controller is deactivated.
  command_interfaces_[CommandInterfaces::FORCE_MODE_DISABLE_CMD].set_value(1.0);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::ForceModeController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  set_force_mode_srv_.reset();
  disable_force_mode_srv_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ur_controllers::ForceModeController::update(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  async_state_ = command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].get_value();

  // Publish state of force_mode?
  if (change_requested_) {
    if (force_mode_active_) {
      const auto force_mode_parameters = force_mode_params_buffer_.readFromRT();
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_X].set_value(force_mode_parameters->task_frame[0]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_Y].set_value(force_mode_parameters->task_frame[1]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_Z].set_value(force_mode_parameters->task_frame[2]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RX].set_value(force_mode_parameters->task_frame[3]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RY].set_value(force_mode_parameters->task_frame[4]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_TASK_FRAME_RZ].set_value(force_mode_parameters->task_frame[5]);

      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_X].set_value(
          force_mode_parameters->selection_vec[0]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_Y].set_value(
          force_mode_parameters->selection_vec[1]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_Z].set_value(
          force_mode_parameters->selection_vec[2]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RX].set_value(
          force_mode_parameters->selection_vec[3]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RY].set_value(
          force_mode_parameters->selection_vec[4]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_SELECTION_VECTOR_RZ].set_value(
          force_mode_parameters->selection_vec[5]);

      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_X].set_value(force_mode_parameters->wrench.force.x);
      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_Y].set_value(force_mode_parameters->wrench.force.y);
      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_Z].set_value(force_mode_parameters->wrench.force.z);
      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RX].set_value(force_mode_parameters->wrench.torque.x);
      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RY].set_value(force_mode_parameters->wrench.torque.y);
      command_interfaces_[CommandInterfaces::FORCE_MODE_WRENCH_RZ].set_value(force_mode_parameters->wrench.torque.z);

      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_X].set_value(force_mode_parameters->limits[0]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_Y].set_value(force_mode_parameters->limits[1]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_Z].set_value(force_mode_parameters->limits[2]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RX].set_value(force_mode_parameters->limits[3]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RY].set_value(force_mode_parameters->limits[4]);
      command_interfaces_[CommandInterfaces::FORCE_MODE_LIMITS_RZ].set_value(force_mode_parameters->limits[5]);

      command_interfaces_[CommandInterfaces::FORCE_MODE_TYPE].set_value(force_mode_parameters->type);
      command_interfaces_[CommandInterfaces::FORCE_MODE_DAMPING].set_value(force_mode_parameters->damping_factor);
      command_interfaces_[CommandInterfaces::FORCE_MODE_GAIN_SCALING].set_value(force_mode_parameters->gain_scaling);

      // Signal that we are waiting for confirmation that force mode is activated
      command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
      async_state_ = ASYNC_WAITING;
    } else {
      command_interfaces_[CommandInterfaces::FORCE_MODE_DISABLE_CMD].set_value(1.0);
      command_interfaces_[CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
      async_state_ = ASYNC_WAITING;
    }
    change_requested_ = false;
  }

  return controller_interface::return_type::OK;
}

bool ForceModeController::setForceMode(const ur_msgs::srv::SetForceMode::Request::SharedPtr req,
                                       ur_msgs::srv::SetForceMode::Response::SharedPtr resp)
{
  // Reject if controller is not active
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new requests. Controller is not running.");
    resp->success = false;
    return false;
  }

  ForceModeParameters force_mode_parameters;

  // transform task frame into base
  const std::string tf_prefix = params_.tf_prefix;
  if (std::abs(req->task_frame.pose.orientation.x) < 1e-6 && std::abs(req->task_frame.pose.orientation.y) < 1e-6 &&
      std::abs(req->task_frame.pose.orientation.z) < 1e-6 && std::abs(req->task_frame.pose.orientation.w) < 1e-6) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received task frame with all-zeros quaternion. It should have at least one "
                                           "non-zero entry.");
    resp->success = false;
    return false;
  }

  try {
    auto task_frame_transformed = tf_buffer_->transform(req->task_frame, tf_prefix + "base");

    force_mode_parameters.task_frame[0] = task_frame_transformed.pose.position.x;
    force_mode_parameters.task_frame[1] = task_frame_transformed.pose.position.y;
    force_mode_parameters.task_frame[2] = task_frame_transformed.pose.position.z;

    tf2::Quaternion quat_tf;
    tf2::convert(task_frame_transformed.pose.orientation, quat_tf);
    const double angle = quat_tf.getAngle();
    const auto axis = quat_tf.getAxis();
    force_mode_parameters.task_frame[3] = axis.x() * angle;  // rx
    force_mode_parameters.task_frame[4] = axis.y() * angle;  // ry
    force_mode_parameters.task_frame[5] = axis.z() * angle;  // rz
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not transform %s to robot base: %s",
                 req->task_frame.header.frame_id.c_str(), ex.what());
    resp->success = false;
    return false;
  }

  // The selection vector dictates which axes the robot should be compliant along and around
  force_mode_parameters.selection_vec[0] = req->selection_vector_x;
  force_mode_parameters.selection_vec[1] = req->selection_vector_y;
  force_mode_parameters.selection_vec[2] = req->selection_vector_z;
  force_mode_parameters.selection_vec[3] = req->selection_vector_rx;
  force_mode_parameters.selection_vec[4] = req->selection_vector_ry;
  force_mode_parameters.selection_vec[5] = req->selection_vector_rz;

  // The wrench parameters dictate the amount of force/torque the robot will apply to its environment. The robot will
  // move along/around compliant axes to match the specified force/torque. Has no effect for non-compliant axes.
  force_mode_parameters.wrench = req->wrench;

  /* The limits specifies the maximum allowed speed along/around compliant axes. For non-compliant axes this value is
   * the maximum allowed deviation between actual tcp position and the one that has been programmed. */
  force_mode_parameters.limits[0] = req->selection_vector_x ? req->speed_limits.linear.x : req->deviation_limits[0];
  force_mode_parameters.limits[1] = req->selection_vector_y ? req->speed_limits.linear.y : req->deviation_limits[1];
  force_mode_parameters.limits[2] = req->selection_vector_z ? req->speed_limits.linear.z : req->deviation_limits[2];
  force_mode_parameters.limits[3] = req->selection_vector_rx ? req->speed_limits.angular.x : req->deviation_limits[3];
  force_mode_parameters.limits[4] = req->selection_vector_ry ? req->speed_limits.angular.y : req->deviation_limits[4];
  force_mode_parameters.limits[5] = req->selection_vector_rz ? req->speed_limits.angular.z : req->deviation_limits[5];

  if (req->type < 1 || req->type > 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "The force mode type has to be 1, 2, or 3. Received %u", req->type);
    resp->success = false;
    return false;
  }

  /* The type decides how the robot interprets the force frame (the one defined in task_frame). See ur_script manual
   * for explanation, under force_mode. */
  force_mode_parameters.type = static_cast<double>(req->type);

  /* The damping factor decides how fast the robot decelarates if no force is present. 0 means no deceleration, 1
   * means quick deceleration*/
  if (req->damping_factor < 0.0 || req->damping_factor > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "The damping factor has to be between 0 and 1. Received %f",
                 req->damping_factor);
    resp->success = false;
    return false;
  }
  force_mode_parameters.damping_factor = req->damping_factor;

  /*The gain scaling factor scales the force mode gain. A value larger than 1 may make force mode unstable. */
  if (req->gain_scaling < 0.0 || req->gain_scaling > 2.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "The gain scaling has to be between 0 and 2. Received %f",
                 req->gain_scaling);
    resp->success = false;
    return false;
  }
  if (req->gain_scaling > 1.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "A gain_scaling >1.0 can make force mode unstable, e.g. in case of collisions or pushing against "
                "hard surfaces. Received %f",
                req->gain_scaling);
  }
  force_mode_parameters.gain_scaling = req->gain_scaling;

  force_mode_params_buffer_.writeFromNonRT(force_mode_parameters);
  force_mode_active_ = true;
  change_requested_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for force mode to be set.");
  const auto maximum_retries = params_.check_io_successful_retries;
  int retries = 0;
  while (async_state_ == ASYNC_WAITING || change_requested_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    retries++;

    if (retries > maximum_retries) {
      resp->success = false;
    }
  }

  resp->success = async_state_ == 1.0;

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Force mode has been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the force mode.");
    return false;
  }

  return true;
}

bool ForceModeController::disableForceMode(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                           std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  force_mode_active_ = false;
  change_requested_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for force mode to be disabled.");
  while (async_state_ == ASYNC_WAITING || change_requested_) {
    // Asynchronous wait until the hardware interface has set the force mode
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  resp->success = async_state_ == 1.0;
  if (resp->success) {
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
