// Copyright 2023, FZI Forschungszentrum Informatik
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
 * \author  Felix Exner <exner@fzi.de>
 * \date    2023-09-11
 *
 */
//----------------------------------------------------------------------
//
#include "ur_controllers/pose_broadcaster.hpp"
#include <rclcpp/logging.hpp>
#include "tf2_msgs/msg/tf_message.hpp"

namespace ur_controllers
{

PoseBroadcaster::PoseBroadcaster() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration PoseBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PoseBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = pose_component_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::return_type PoseBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    pose_component_->get_values_as_message(pose_);
    realtime_publisher_->msg_.transforms[0].header.stamp = time;
    realtime_publisher_->msg_.transforms[0].transform.translation.x = pose_.position.x;
    realtime_publisher_->msg_.transforms[0].transform.translation.y = pose_.position.y;
    realtime_publisher_->msg_.transforms[0].transform.translation.z = pose_.position.z;
    realtime_publisher_->msg_.transforms[0].transform.rotation.w = pose_.orientation.w;
    realtime_publisher_->msg_.transforms[0].transform.rotation.x = pose_.orientation.x;
    realtime_publisher_->msg_.transforms[0].transform.rotation.y = pose_.orientation.y;
    realtime_publisher_->msg_.transforms[0].transform.rotation.z = pose_.orientation.z;
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn PoseBroadcaster::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  try {
    param_listener_ = std::make_shared<pose_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  pose_component_ = std::make_unique<ur_controllers::PoseComponent>(ur_controllers::PoseComponent(params_.sensor_name));

  try {
    // register sensor data publisher
    sensor_state_publisher_ =
        get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not create publisher: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.transforms.resize(1);
  realtime_publisher_->msg_.transforms[0].header.frame_id = params_.parent_frame_id;
  realtime_publisher_->msg_.transforms[0].child_frame_id = params_.child_frame_id;
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  pose_component_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  pose_component_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ur_controllers
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PoseBroadcaster, controller_interface::ControllerInterface)
