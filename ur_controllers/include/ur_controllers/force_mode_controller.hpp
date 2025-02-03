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

#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/srv/set_force_mode.hpp>

#include "ur_controllers/force_mode_controller_parameters.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  FORCE_MODE_TASK_FRAME_X = 0u,
  FORCE_MODE_TASK_FRAME_Y = 1,
  FORCE_MODE_TASK_FRAME_Z = 2,
  FORCE_MODE_TASK_FRAME_RX = 3,
  FORCE_MODE_TASK_FRAME_RY = 4,
  FORCE_MODE_TASK_FRAME_RZ = 5,
  FORCE_MODE_SELECTION_VECTOR_X = 6,
  FORCE_MODE_SELECTION_VECTOR_Y = 7,
  FORCE_MODE_SELECTION_VECTOR_Z = 8,
  FORCE_MODE_SELECTION_VECTOR_RX = 9,
  FORCE_MODE_SELECTION_VECTOR_RY = 10,
  FORCE_MODE_SELECTION_VECTOR_RZ = 11,
  FORCE_MODE_WRENCH_X = 12,
  FORCE_MODE_WRENCH_Y = 13,
  FORCE_MODE_WRENCH_Z = 14,
  FORCE_MODE_WRENCH_RX = 15,
  FORCE_MODE_WRENCH_RY = 16,
  FORCE_MODE_WRENCH_RZ = 17,
  FORCE_MODE_TYPE = 18,
  FORCE_MODE_LIMITS_X = 19,
  FORCE_MODE_LIMITS_Y = 20,
  FORCE_MODE_LIMITS_Z = 21,
  FORCE_MODE_LIMITS_RX = 22,
  FORCE_MODE_LIMITS_RY = 23,
  FORCE_MODE_LIMITS_RZ = 24,
  FORCE_MODE_ASYNC_SUCCESS = 25,
  FORCE_MODE_DISABLE_CMD = 26,
  FORCE_MODE_DAMPING = 27,
  FORCE_MODE_GAIN_SCALING = 28,
};
enum StateInterfaces
{
  INITIALIZED_FLAG = 0u,
};

struct ForceModeParameters
{
  std::array<double, 6> task_frame;
  std::array<double, 6> selection_vec;
  std::array<double, 6> limits;
  geometry_msgs::msg::Wrench wrench;
  double type;
  double damping_factor;
  double gain_scaling;
};

class ForceModeController : public controller_interface::ControllerInterface
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
  bool setForceMode(const ur_msgs::srv::SetForceMode::Request::SharedPtr req,
                    ur_msgs::srv::SetForceMode::Response::SharedPtr resp);
  bool disableForceMode(const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr resp);
  rclcpp::Service<ur_msgs::srv::SetForceMode>::SharedPtr set_force_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_force_mode_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<force_mode_controller::ParamListener> param_listener_;
  force_mode_controller::Params params_;

  realtime_tools::RealtimeBuffer<ForceModeParameters> force_mode_params_buffer_;
  std::atomic<bool> force_mode_active_;
  std::atomic<bool> change_requested_;
  std::atomic<double> async_state_;

  static constexpr double ASYNC_WAITING = 2.0;
  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace ur_controllers
