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

#ifndef UR_CONTROLLERS__GRAVITY_UPDATE_CONTROLLER_HPP_
#define UR_CONTROLLERS__GRAVITY_UPDATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// #include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
// #include "ur_msgs/msg/io_states.hpp"
// #include "ur_msgs/msg/tool_data_msg.hpp"
// #include "ur_dashboard_msgs/msg/robot_mode.hpp"
// #include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_msgs/srv/set_gravity.hpp"
// #include "ur_msgs/srv/set_io.hpp"
// #include "ur_msgs/srv/set_analog_output.hpp"
// #include "ur_msgs/srv/set_speed_slider_fraction.hpp"
// #include "ur_msgs/srv/set_payload.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
// #include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "ur_controllers/gravity_update_controller_parameters.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  GRAVITY_X,
  GRAVITY_Y,
  GRAVITY_Z,
  GRAVITY_ASYNC_SUCCESS,
};

enum StateInterfaces
{
  INITIALIZED_FLAG = 69,
};

class GravityUpdateController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  // bool setIO(ur_msgs::srv::SetIO::Request::SharedPtr req, ur_msgs::srv::SetIO::Response::SharedPtr resp);

  // bool setAnalogOutput(ur_msgs::srv::SetAnalogOutput::Request::SharedPtr req,
  //                      ur_msgs::srv::SetAnalogOutput::Response::SharedPtr resp);

  // bool setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
  //                     ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp);

  // bool resendRobotProgram(std_srvs::srv::Trigger::Request::SharedPtr req,
  //                         std_srvs::srv::Trigger::Response::SharedPtr resp);

  // bool handBackControl(std_srvs::srv::Trigger::Request::SharedPtr req,
  //                      std_srvs::srv::Trigger::Response::SharedPtr resp);

  // bool setPayload(const ur_msgs::srv::SetPayload::Request::SharedPtr req,
  //                 ur_msgs::srv::SetPayload::Response::SharedPtr resp);

  bool setGravity(const ur_msgs::srv::SetGravity::Request::SharedPtr req,
                  ur_msgs::srv::SetGravity::Response::SharedPtr resp);

  // bool zeroFTSensor(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

  // void publishIO();

  // void publishToolData();

  // void publishRobotMode();

  // void publishSafetyMode();

  // void publishProgramRunning();

protected:
  // void initMsgs();

  // bool first_pass_;

  // internal commands
  // std::array<double, 18> standard_digital_output_cmd_;
  // std::array<double, 18> standard_analog_output_cmd_;
  // double target_speed_fraction_cmd_;

  // services
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hand_back_control_srv_;
  // rclcpp::Service<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr set_speed_slider_srv_;
  // rclcpp::Service<ur_msgs::srv::SetIO>::SharedPtr set_io_srv_;
  // rclcpp::Service<ur_msgs::srv::SetAnalogOutput>::SharedPtr set_analog_output_srv_;
  // rclcpp::Service<ur_msgs::srv::SetPayload>::SharedPtr set_payload_srv_;
  rclcpp::Service<ur_msgs::srv::SetGravity>::SharedPtr set_gravity_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_sensor_srv_;

  // std::shared_ptr<rclcpp::Publisher<ur_msgs::msg::IOStates>> io_pub_;
  // std::shared_ptr<rclcpp::Publisher<ur_msgs::msg::ToolDataMsg>> tool_data_pub_;
  // std::shared_ptr<rclcpp::Publisher<ur_dashboard_msgs::msg::RobotMode>> robot_mode_pub_;
  // std::shared_ptr<rclcpp::Publisher<ur_dashboard_msgs::msg::SafetyMode>> safety_mode_pub_;
  // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> program_state_pub_;

  // ur_msgs::msg::IOStates io_msg_;
  // ur_msgs::msg::ToolDataMsg tool_data_msg_;
  // ur_dashboard_msgs::msg::RobotMode robot_mode_msg_;
  // ur_dashboard_msgs::msg::SafetyMode safety_mode_msg_;
  // std_msgs::msg::Bool program_running_msg_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters from ROS for gpio_controller
  std::shared_ptr<gravity_update_controller::ParamListener> param_listener_;
  gravity_update_controller::Params params_;

  static constexpr double ASYNC_WAITING = 2.0;
  // TODO(anyone) publishers to add: tcp_pose_pub_
  // TODO(anyone) subscribers to add: script_command_sub_
  // TODO(anyone) service servers to add: resend_robot_program_srv_, deactivate_srv_, set_payload_srv_, tare_sensor_srv_

  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__GRAVITY_UPDATE_CONTROLLER_HPP_
