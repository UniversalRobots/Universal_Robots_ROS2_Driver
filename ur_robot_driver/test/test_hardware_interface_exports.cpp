// Copyright 2026, FZI Forschungszentrum Informatik
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

// Regression test for the deprecated-Handle-API migration: the exported state/command
// interface *names* must exactly match what the old raw-pointer export_state_interfaces()/
// export_command_interfaces() used to export, one-for-one. Nothing here exercises hardware -
// only on_init() + export via the hardware_interface::System wrapper (which runs the same
// old-export-empty -> on_export_state_interfaces() fallback the real resource_manager uses).
//
// The HardwareInfo below transcribes every joint/sensor/gpio declared in
// ur.ros2_control.xacro (plus ur_description's ur_joint_control.xacro/ur_sensors.xacro), since
// every interface this driver exports turned out to already be URDF-declared - there is no
// export_unlisted_*_interface_descriptions() override to test separately.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ur_robot_driver/hardware_interface.hpp"

namespace
{
constexpr size_t kJointCount = 6;
constexpr const char* kTfPrefix = "";

hardware_interface::HardwareInfo BuildTestHardwareInfo()
{
  hardware_interface::HardwareInfo info;
  info.type = "system";
  info.hardware_parameters["tf_prefix"] = kTfPrefix;

  auto make_interface = [](const std::string& name) {
    hardware_interface::InterfaceInfo iface{};
    iface.name = name;
    return iface;
  };

  for (size_t i = 1; i <= kJointCount; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    joint.command_interfaces = { make_interface(hardware_interface::HW_IF_POSITION),
                                  make_interface(hardware_interface::HW_IF_VELOCITY),
                                  make_interface(hardware_interface::HW_IF_EFFORT) };
    joint.state_interfaces = { make_interface(hardware_interface::HW_IF_POSITION),
                                make_interface(hardware_interface::HW_IF_VELOCITY),
                                make_interface(hardware_interface::HW_IF_EFFORT) };
    info.joints.push_back(joint);
  }

  auto make_sensor = [&](const std::string& name, const std::vector<std::string>& state_names) {
    hardware_interface::ComponentInfo sensor;
    sensor.name = std::string(kTfPrefix) + name;
    for (const auto& state_name : state_names)
    {
      sensor.state_interfaces.push_back(make_interface(state_name));
    }
    info.sensors.push_back(sensor);
  };
  make_sensor("tcp_fts_sensor", { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" });
  make_sensor("tcp_pose", { "position.x", "position.y", "position.z", "orientation.x", "orientation.y",
                            "orientation.z", "orientation.w" });

  auto make_gpio = [&](const std::string& name, const std::vector<std::string>& command_names,
                       const std::vector<std::string>& state_names) {
    hardware_interface::ComponentInfo gpio;
    gpio.name = std::string(kTfPrefix) + name;
    for (const auto& command_name : command_names)
    {
      gpio.command_interfaces.push_back(make_interface(command_name));
    }
    for (const auto& state_name : state_names)
    {
      gpio.state_interfaces.push_back(make_interface(state_name));
    }
    info.gpios.push_back(gpio);
  };

  make_gpio("speed_scaling", { "target_speed_fraction_cmd", "target_speed_fraction_async_success" },
            { "speed_scaling_factor" });

  std::vector<std::string> gpio_cmds;
  for (size_t i = 0; i < 18; ++i)
  {
    gpio_cmds.push_back("standard_digital_output_cmd_" + std::to_string(i));
  }
  gpio_cmds.push_back("standard_analog_output_cmd_0");
  gpio_cmds.push_back("standard_analog_output_cmd_1");
  gpio_cmds.push_back("analog_output_domain_cmd");
  gpio_cmds.push_back("tool_voltage_cmd");
  gpio_cmds.push_back("io_async_success");
  std::vector<std::string> gpio_states;
  for (size_t i = 0; i < 18; ++i)
  {
    gpio_states.push_back("digital_output_" + std::to_string(i));
  }
  for (size_t i = 0; i < 18; ++i)
  {
    gpio_states.push_back("digital_input_" + std::to_string(i));
  }
  gpio_states.push_back("standard_analog_output_0");
  gpio_states.push_back("standard_analog_output_1");
  gpio_states.push_back("standard_analog_input_0");
  gpio_states.push_back("standard_analog_input_1");
  for (size_t i = 0; i < 4; ++i)
  {
    gpio_states.push_back("analog_io_type_" + std::to_string(i));
  }
  gpio_states.push_back("tool_mode");
  gpio_states.push_back("tool_output_voltage");
  gpio_states.push_back("tool_output_current");
  gpio_states.push_back("tool_temperature");
  gpio_states.push_back("tool_analog_input_0");
  gpio_states.push_back("tool_analog_input_1");
  gpio_states.push_back("tool_analog_input_type_0");
  gpio_states.push_back("tool_analog_input_type_1");
  gpio_states.push_back("robot_mode");
  for (size_t i = 0; i < 4; ++i)
  {
    gpio_states.push_back("robot_status_bit_" + std::to_string(i));
  }
  gpio_states.push_back("safety_mode");
  for (size_t i = 0; i < 11; ++i)
  {
    gpio_states.push_back("safety_status_bit_" + std::to_string(i));
  }
  gpio_states.push_back("program_running");
  make_gpio("gpio", gpio_cmds, gpio_states);

  make_gpio("payload",
            { "mass", "cog.x", "cog.y", "cog.z", "inertia.ixx", "inertia.iyy", "inertia.izz", "inertia.ixy",
              "inertia.ixz", "inertia.iyz", "transition_time", "payload_async_success" },
            { "mass", "cog.x", "cog.y", "cog.z", "inertia.ixx", "inertia.iyy", "inertia.izz", "inertia.ixy",
              "inertia.ixz", "inertia.iyz" });

  make_gpio("gravity", { "x", "y", "z", "gravity_async_success" }, {});

  std::vector<std::string> friction_cmds;
  for (size_t i = 0; i < 6; ++i)
  {
    friction_cmds.push_back("viscous_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    friction_cmds.push_back("coulomb_" + std::to_string(i));
  }
  friction_cmds.push_back("async_success");
  make_gpio("friction_model", friction_cmds, {});

  make_gpio("resend_robot_program", { "resend_robot_program_cmd", "resend_robot_program_async_success" }, {});
  make_gpio("hand_back_control", { "hand_back_control_cmd", "hand_back_control_async_success" }, {});
  make_gpio("zero_ftsensor", { "zero_ftsensor_cmd", "zero_ftsensor_async_success" }, {});
  make_gpio("freedrive_mode", { "async_success", "enable", "abort" }, {});
  make_gpio("tool_contact", { "tool_contact_set_state" }, { "tool_contact_result", "tool_contact_state" });
  make_gpio("system_interface", {}, { "initialized" });

  std::vector<std::string> force_mode_cmds = {
    "task_frame_x",         "task_frame_y",         "task_frame_z",        "task_frame_rx",
    "task_frame_ry",        "task_frame_rz",        "selection_vector_x",  "selection_vector_y",
    "selection_vector_z",   "selection_vector_rx",  "selection_vector_ry", "selection_vector_rz",
    "wrench_x",             "wrench_y",             "wrench_z",            "wrench_rx",
    "wrench_ry",            "wrench_rz",            "limits_x",            "limits_y",
    "limits_z",             "limits_rx",            "limits_ry",           "limits_rz",
    "type",                 "damping",              "gain_scaling",        "disable_cmd",
    "force_mode_async_success"
  };
  make_gpio("force_mode", force_mode_cmds, {});

  std::vector<std::string> passthrough_cmds;
  for (size_t i = 0; i < 6; ++i)
  {
    passthrough_cmds.push_back("setpoint_positions_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    passthrough_cmds.push_back("setpoint_velocities_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    passthrough_cmds.push_back("setpoint_accelerations_" + std::to_string(i));
  }
  passthrough_cmds.push_back("transfer_state");
  passthrough_cmds.push_back("time_from_start");
  passthrough_cmds.push_back("abort");
  passthrough_cmds.push_back("trajectory_size");
  make_gpio("trajectory_passthrough", passthrough_cmds, {});

  make_gpio("twist",
            { "linear_velocity_x", "linear_velocity_y", "linear_velocity_z", "angular_velocity_x",
              "angular_velocity_y", "angular_velocity_z" },
            {});

  make_gpio("get_robot_software_version", {},
            { "get_version_major", "get_version_minor", "get_version_build", "get_version_bugfix" });

  std::vector<std::string> moprim_cmds = { "motion_type", "q1",  "q2",  "q3",  "q4",       "q5",
                                           "q6",          "pos_x", "pos_y", "pos_z", "pos_qx",   "pos_qy",
                                           "pos_qz",      "pos_qw", "pos_via_x", "pos_via_y", "pos_via_z",
                                           "pos_via_qx",  "pos_via_qy", "pos_via_qz", "pos_via_qw",
                                           "blend_radius", "velocity", "acceleration", "move_time" };
  make_gpio("motion_primitive", moprim_cmds, { "execution_status", "ready_for_new_primitive" });

  return info;
}

std::vector<std::string> GetNames(const std::vector<hardware_interface::StateInterface::ConstSharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

std::vector<std::string> GetNames(const std::vector<hardware_interface::CommandInterface::SharedPtr>& interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto& interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

class URHardwareInterfaceExportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = BuildTestHardwareInfo();
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_ur_hardware_interface_exports");

    hw_ = std::make_unique<hardware_interface::System>(
        std::make_unique<ur_robot_driver::URPositionHardwareInterface>());
    const auto state = hw_->initialize(params);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "on_init() did not succeed - check the test HardwareInfo matches what "
           "URPositionHardwareInterface::on_init() validates.";
  }

  std::unique_ptr<hardware_interface::System> hw_;
};

// Interfaces exported by the removed raw-pointer export_state_interfaces(), for comparison.
TEST_F(URHardwareInterfaceExportTest, ExportsExpectedStateInterfaces)
{
  const auto exported_names = GetNames(hw_->export_state_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  expected_names.push_back(std::string(kTfPrefix) + "speed_scaling/speed_scaling_factor");
  for (const char* name : { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "tcp_fts_sensor/" + name);
  }
  for (size_t i = 0; i < 18; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/digital_output_" + std::to_string(i));
    expected_names.push_back(std::string(kTfPrefix) + "gpio/digital_input_" + std::to_string(i));
  }
  for (size_t i = 0; i < 11; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/safety_status_bit_" + std::to_string(i));
  }
  for (size_t i = 0; i < 4; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/analog_io_type_" + std::to_string(i));
    expected_names.push_back(std::string(kTfPrefix) + "gpio/robot_status_bit_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/tool_analog_input_type_" + std::to_string(i));
    expected_names.push_back(std::string(kTfPrefix) + "gpio/tool_analog_input_" + std::to_string(i));
    expected_names.push_back(std::string(kTfPrefix) + "gpio/standard_analog_input_" + std::to_string(i));
    expected_names.push_back(std::string(kTfPrefix) + "gpio/standard_analog_output_" + std::to_string(i));
  }
  for (const char* name : { "tool_output_voltage", "robot_mode", "safety_mode", "tool_mode", "tool_output_current",
                           "tool_temperature", "program_running" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/" + name);
  }
  expected_names.push_back(std::string(kTfPrefix) + "system_interface/initialized");
  for (const char* name :
       { "position.x", "position.y", "position.z", "orientation.x", "orientation.y", "orientation.z",
         "orientation.w" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "tcp_pose/" + name);
  }
  for (const char* name :
       { "get_version_major", "get_version_minor", "get_version_bugfix", "get_version_build" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "get_robot_software_version/" + name);
  }
  expected_names.push_back(std::string(kTfPrefix) + "tool_contact/tool_contact_result");
  expected_names.push_back(std::string(kTfPrefix) + "tool_contact/tool_contact_state");
  for (const char* name : { "mass", "cog.x", "cog.y", "cog.z", "inertia.ixx", "inertia.iyy", "inertia.izz",
                           "inertia.ixy", "inertia.ixz", "inertia.iyz" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "payload/" + name);
  }
  expected_names.push_back(std::string(kTfPrefix) + "motion_primitive/execution_status");
  expected_names.push_back(std::string(kTfPrefix) + "motion_primitive/ready_for_new_primitive");

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}

// Interfaces exported by the removed raw-pointer export_command_interfaces(), for comparison.
TEST_F(URHardwareInterfaceExportTest, ExportsExpectedCommandInterfaces)
{
  const auto exported_names = GetNames(hw_->export_command_interfaces());

  std::vector<std::string> expected_names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    expected_names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  expected_names.push_back(std::string(kTfPrefix) + "gpio/io_async_success");
  expected_names.push_back(std::string(kTfPrefix) + "speed_scaling/target_speed_fraction_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "speed_scaling/target_speed_fraction_async_success");
  expected_names.push_back(std::string(kTfPrefix) + "resend_robot_program/resend_robot_program_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "resend_robot_program/resend_robot_program_async_success");
  expected_names.push_back(std::string(kTfPrefix) + "hand_back_control/hand_back_control_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "hand_back_control/hand_back_control_async_success");
  for (const char* name : { "mass", "cog.x", "cog.y", "cog.z", "inertia.ixx", "inertia.iyy", "inertia.izz",
                           "inertia.ixy", "inertia.ixz", "inertia.iyz", "transition_time", "payload_async_success" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "payload/" + name);
  }
  for (size_t i = 0; i < 6; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "friction_model/viscous_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "friction_model/coulomb_" + std::to_string(i));
  }
  expected_names.push_back(std::string(kTfPrefix) + "friction_model/async_success");
  for (const char* name :
       { "task_frame_x",        "task_frame_y",        "task_frame_z",        "task_frame_rx",
         "task_frame_ry",       "task_frame_rz",       "selection_vector_x",  "selection_vector_y",
         "selection_vector_z",  "selection_vector_rx", "selection_vector_ry", "selection_vector_rz",
         "wrench_x",            "wrench_y",            "wrench_z",            "wrench_rx",
         "wrench_ry",           "wrench_rz",           "type",                "limits_x",
         "limits_y",            "limits_z",            "limits_rx",           "limits_ry",
         "limits_rz",           "force_mode_async_success", "disable_cmd",    "damping",
         "gain_scaling" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "force_mode/" + name);
  }
  for (const char* name : { "x", "y", "z", "gravity_async_success" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "gravity/" + name);
  }
  for (size_t i = 0; i < 18; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/standard_digital_output_cmd_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "gpio/standard_analog_output_cmd_" + std::to_string(i));
  }
  expected_names.push_back(std::string(kTfPrefix) + "gpio/analog_output_domain_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "gpio/tool_voltage_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "zero_ftsensor/zero_ftsensor_cmd");
  expected_names.push_back(std::string(kTfPrefix) + "zero_ftsensor/zero_ftsensor_async_success");
  expected_names.push_back(std::string(kTfPrefix) + "freedrive_mode/async_success");
  expected_names.push_back(std::string(kTfPrefix) + "freedrive_mode/enable");
  expected_names.push_back(std::string(kTfPrefix) + "freedrive_mode/abort");
  expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/transfer_state");
  expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/time_from_start");
  expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/abort");
  expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/trajectory_size");
  for (size_t i = 0; i < 6; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/setpoint_positions_" +
                             std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/setpoint_velocities_" +
                             std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i)
  {
    expected_names.push_back(std::string(kTfPrefix) + "trajectory_passthrough/setpoint_accelerations_" +
                             std::to_string(i));
  }
  for (const char* name :
       { "linear_velocity_x", "linear_velocity_y", "linear_velocity_z", "angular_velocity_x",
         "angular_velocity_y", "angular_velocity_z" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "twist/" + name);
  }
  expected_names.push_back(std::string(kTfPrefix) + "tool_contact/tool_contact_set_state");
  for (const char* name : { "motion_type", "q1",  "q2",  "q3",  "q4",  "q5",  "q6",  "pos_x", "pos_y", "pos_z",
                           "pos_qx", "pos_qy", "pos_qz", "pos_qw", "pos_via_x", "pos_via_y", "pos_via_z",
                           "pos_via_qx", "pos_via_qy", "pos_via_qz", "pos_via_qw", "blend_radius", "velocity",
                           "acceleration", "move_time" })
  {
    expected_names.push_back(std::string(kTfPrefix) + "motion_primitive/" + name);
  }

  EXPECT_THAT(exported_names, ::testing::UnorderedElementsAreArray(expected_names));
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
