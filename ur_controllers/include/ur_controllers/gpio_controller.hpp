// Copyright (c) 2021 PickNik LLC
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
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-20
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define UR_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "ur_msgs/msg/tool_data_t1_msg.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_analog_output.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"
#include "ur_msgs/srv/set_payload.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ur_controllers/gpio_controller_parameters.hpp"

namespace ur_controllers
{
// Flat digital-output index used by SetIO and standard_digital_output_cmd_*.
// 0-7 standard, 8-15 configurable, 16-17 T0_DO, 18-21 T1_DO0-T1_DO3.
// T1 outputs are tool digital pins 2-5 (RTDEWriter::sendToolDigitalOutput).
inline constexpr std::size_t kStandardDigitalIoCount = 18;
inline constexpr std::size_t kT1DigitalOutputCount = 4;
inline constexpr std::size_t kDigitalOutputCount = kStandardDigitalIoCount + kT1DigitalOutputCount;
inline constexpr std::size_t kT1ConfigurableDigitalInputCount = 2;
inline constexpr std::size_t kT1SmartPanelDigitalInputCount = 3;

enum CommandInterfaces
{
  DIGITAL_OUTPUTS_CMD = 0u,
  ANALOG_OUTPUTS_CMD = 22,
  TOOL_VOLTAGE_CMD = 24,
  IO_ASYNC_SUCCESS = 25,
  TARGET_SPEED_FRACTION_CMD = 26,
  TARGET_SPEED_FRACTION_ASYNC_SUCCESS = 27,
  RESEND_ROBOT_PROGRAM_CMD = 28,
  RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS = 29,
  PAYLOAD_MASS = 30,
  PAYLOAD_COG_X = 31,
  PAYLOAD_COG_Y = 32,
  PAYLOAD_COG_Z = 33,
  PAYLOAD_ASYNC_SUCCESS = 34,
  ZERO_FTSENSOR_CMD = 35,
  ZERO_FTSENSOR_ASYNC_SUCCESS = 36,
  HAND_BACK_CONTROL_CMD = 37,
  HAND_BACK_CONTROL_ASYNC_SUCCESS = 38,
  ANALOG_OUTPUTS_DOMAIN = 39,
  PAYLOAD_INERTIA_IXX = 40,
  PAYLOAD_INERTIA_IYY = 41,
  PAYLOAD_INERTIA_IZZ = 42,
  PAYLOAD_INERTIA_IXY = 43,
  PAYLOAD_INERTIA_IXZ = 44,
  PAYLOAD_INERTIA_IYZ = 45,
  PAYLOAD_TRANSITION_TIME = 46,
};

enum StateInterfaces
{
  DIGITAL_OUTPUTS = 0u,
  DIGITAL_INPUTS = 22,
  ANALOG_OUTPUTS = 40,
  ANALOG_INPUTS = 42,
  ANALOG_IO_TYPES = 44,
  TOOL_MODE = 48,
  TOOL_OUTPUT_VOLTAGE = 49,
  TOOL_OUTPUT_CURRENT = 50,
  TOOL_TEMPERATURE = 51,
  TOOL_ANALOG_INPUTS = 52,
  TOOL_ANALOG_IO_TYPES = 54,
  ROBOT_MODE = 56,
  ROBOT_STATUS_BITS = 57,
  SAFETY_MODE = 61,
  SAFETY_STATUS_BITS = 62,
  INITIALIZED_FLAG = 73,
  PROGRAM_RUNNING = 74,
  PAYLOAD_STATE_MASS = 75,
  PAYLOAD_STATE_COG_X = 76,
  PAYLOAD_STATE_COG_Y = 77,
  PAYLOAD_STATE_COG_Z = 78,
  PAYLOAD_STATE_INERTIA_IXX = 79,
  PAYLOAD_STATE_INERTIA_IYY = 80,
  PAYLOAD_STATE_INERTIA_IZZ = 81,
  PAYLOAD_STATE_INERTIA_IXY = 82,
  PAYLOAD_STATE_INERTIA_IXZ = 83,
  PAYLOAD_STATE_INERTIA_IYZ = 84,
  TOOL_T1_CONFIGURABLE_DIGITAL_INPUTS = 85,
  TOOL_T1_SMART_PANEL_DIGITAL_INPUTS = 87,
  TOOL_T1_CONNECTED = 90,
};

class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  // Rejects a service request when the controller is not active.
  // Returns true if the controller is active and the request may proceed.
  template <typename ResponseT>
  bool ensureActive(const ResponseT& resp);

  bool setIO(ur_msgs::srv::SetIO::Request::SharedPtr req, ur_msgs::srv::SetIO::Response::SharedPtr resp);

  bool setAnalogOutput(ur_msgs::srv::SetAnalogOutput::Request::SharedPtr req,
                       ur_msgs::srv::SetAnalogOutput::Response::SharedPtr resp);

  bool setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                      ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp);

  bool resendRobotProgram(std_srvs::srv::Trigger::Request::SharedPtr req,
                          std_srvs::srv::Trigger::Response::SharedPtr resp);

  bool handBackControl(std_srvs::srv::Trigger::Request::SharedPtr req,
                       std_srvs::srv::Trigger::Response::SharedPtr resp);

  bool setPayload(const ur_msgs::srv::SetPayload::Request::SharedPtr req,
                  ur_msgs::srv::SetPayload::Response::SharedPtr resp);

  bool zeroFTSensor(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

  void publishIO();

  void publishToolData();

  void publishToolDataT1();

  void publishRobotMode();

  void publishSafetyMode();

  void publishProgramRunning();

protected:
  void initMsgs();

  bool first_pass_;

  // internal commands
  double target_speed_fraction_cmd_;

  // services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hand_back_control_srv_;
  rclcpp::Service<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr set_speed_slider_srv_;
  rclcpp::Service<ur_msgs::srv::SetIO>::SharedPtr set_io_srv_;
  rclcpp::Service<ur_msgs::srv::SetAnalogOutput>::SharedPtr set_analog_output_srv_;
  rclcpp::Service<ur_msgs::srv::SetPayload>::SharedPtr set_payload_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_sensor_srv_;

  // Publishing in realtime ros2_control loop, so these are wrapped in non-blocking tries to prevent controller overrun
  std::shared_ptr<realtime_tools::RealtimePublisher<ur_msgs::msg::IOStates>> io_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ur_msgs::msg::ToolDataMsg>> tool_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ur_msgs::msg::ToolDataT1Msg>> tool_data_t1_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ur_dashboard_msgs::msg::RobotMode>> robot_mode_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ur_dashboard_msgs::msg::SafetyMode>> safety_mode_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> program_state_pub_;

  ur_msgs::msg::IOStates io_msg_;
  ur_msgs::msg::ToolDataMsg tool_data_msg_;
  ur_msgs::msg::ToolDataT1Msg tool_data_t1_msg_;
  ur_dashboard_msgs::msg::RobotMode robot_mode_msg_;
  ur_dashboard_msgs::msg::SafetyMode safety_mode_msg_;
  std_msgs::msg::Bool program_running_msg_;

  // Parameters from ROS for gpio_controller
  std::shared_ptr<gpio_controller::ParamListener> param_listener_;
  gpio_controller::Params params_;

  static constexpr double ASYNC_WAITING = 2.0;
  // TODO(anyone) publishers to add: tcp_pose_pub_
  // TODO(anyone) subscribers to add: script_command_sub_
  // TODO(anyone) service servers to add: resend_robot_program_srv_, deactivate_srv_, set_payload_srv_, tare_sensor_srv_

  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);

  bool waitForPayloadRtdeMatch(double mass, double cx, double cy, double cz, double ixx, double iyy, double izz,
                               double ixy, double ixz, double iyz, double transition_time);
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__GPIO_CONTROLLER_HPP_
