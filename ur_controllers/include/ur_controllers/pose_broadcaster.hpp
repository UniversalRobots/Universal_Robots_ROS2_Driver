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

#ifndef UR_CONTROLLERS__POSE_BROADCASTER_HPP
#define UR_CONTROLLERS__POSE_BROADCASTER_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "pose_broadcaster_parameters.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <ur_controllers/sc_pose.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace ur_controllers
{
class PoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  PoseBroadcaster();
  virtual ~PoseBroadcaster() = default;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_init() override;

private:
  std::unique_ptr<ur_controllers::PoseComponent> pose_component_;
  std::shared_ptr<pose_broadcaster::ParamListener> param_listener_;
  pose_broadcaster::Params params_;

  using StatePublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;

  geometry_msgs::msg::Pose pose_;
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS__POSE_BROADCASTER_HPP
