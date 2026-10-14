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

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace
{
// Interfaces the GPIO controller always claims. Names match gpio_controller.cpp. Connector T2 is
// added separately so a description without it can be tested.
const std::map<std::string, std::vector<std::string>> kCommandInterfaces = {
  { "gpio",
    [] {
      std::vector<std::string> names;
      for (int i = 0; i < 18; ++i) {
        names.push_back("standard_digital_output_cmd_" + std::to_string(i));
      }
      names.push_back("standard_analog_output_cmd_0");
      names.push_back("standard_analog_output_cmd_1");
      names.push_back("tool_voltage_cmd");
      names.push_back("io_async_success");
      names.push_back("analog_output_domain_cmd");
      return names;
    }() },
  { "speed_scaling", { "target_speed_fraction_cmd", "target_speed_fraction_async_success" } },
  { "resend_robot_program", { "resend_robot_program_cmd", "resend_robot_program_async_success" } },
  { "payload",
    { "mass", "cog.x", "cog.y", "cog.z", "payload_async_success", "inertia.ixx", "inertia.iyy", "inertia.izz",
      "inertia.ixy", "inertia.ixz", "inertia.iyz", "transition_time" } },
  { "zero_ftsensor", { "zero_ftsensor_cmd", "zero_ftsensor_async_success" } },
  { "hand_back_control", { "hand_back_control_cmd", "hand_back_control_async_success" } },
};

const std::map<std::string, std::vector<std::string>> kStateInterfaces = {
  { "gpio",
    [] {
      std::vector<std::string> names;
      for (int i = 0; i < 18; ++i) {
        names.push_back("digital_output_" + std::to_string(i));
      }
      for (int i = 0; i < 18; ++i) {
        names.push_back("digital_input_" + std::to_string(i));
      }
      names.insert(names.end(), { "standard_analog_output_0", "standard_analog_output_1", "standard_analog_input_0",
                                  "standard_analog_input_1" });
      for (int i = 0; i < 4; ++i) {
        names.push_back("analog_io_type_" + std::to_string(i));
      }
      names.insert(names.end(), { "tool_mode", "tool_output_voltage", "tool_output_current", "tool_temperature",
                                  "tool_analog_input_0", "tool_analog_input_1", "tool_analog_input_type_0",
                                  "tool_analog_input_type_1", "robot_mode" });
      for (int i = 0; i < 4; ++i) {
        names.push_back("robot_status_bit_" + std::to_string(i));
      }
      names.push_back("safety_mode");
      for (int i = 0; i < 11; ++i) {
        names.push_back("safety_status_bit_" + std::to_string(i));
      }
      names.push_back("program_running");
      return names;
    }() },
  { "system_interface", { "initialized" } },
  { "payload",
    { "mass", "cog.x", "cog.y", "cog.z", "inertia.ixx", "inertia.iyy", "inertia.izz", "inertia.ixy", "inertia.ixz",
      "inertia.iyz" } },
};

std::string makeUrdf(bool with_tool_t2)
{
  auto interfaces = [](const std::string& tag, const std::vector<std::string>& names, const std::string& initial) {
    std::string xml;
    for (const auto& name : names) {
      xml += "<" + tag + " name=\"" + name + "\">";
      if (!initial.empty()) {
        xml += "<param name=\"initial_value\">" + initial + "</param>";
      }
      xml += "</" + tag + ">";
    }
    return xml;
  };

  std::string gpios;
  for (const auto& [name, commands] : kCommandInterfaces) {
    const auto states = kStateInterfaces.count(name) ? kStateInterfaces.at(name) : std::vector<std::string>{};
    std::string extra_commands;
    std::string extra_states;
    if (with_tool_t2 && name == "gpio") {
      extra_commands = interfaces("command_interface", { "tool_voltage_t2_cmd" }, "0");
      extra_states = interfaces(
          "state_interface", { "tool_t2_available", "tool_output_voltage_2", "tool_output_current_2" }, "0");
    }
    const std::string initial = name == "system_interface" ? "1" : "0";
    gpios += "<gpio name=\"" + name + "\">" + interfaces("command_interface", commands, "") + extra_commands +
             interfaces("state_interface", states, initial) + extra_states + "</gpio>";
  }
  // system_interface has no command interfaces, so the loop above skipped it.
  gpios +=
      "<gpio name=\"system_interface\"><state_interface name=\"initialized\">"
      "<param name=\"initial_value\">1</param></state_interface></gpio>";

  return "<robot name=\"gpio_test\"><link name=\"base_link\"/>"
         "<ros2_control name=\"gpio_test\" type=\"system\"><hardware>"
         "<plugin>mock_components/GenericSystem</plugin></hardware>" +
         gpios + "</ros2_control></robot>";
}

controller_interface::return_type activate(const std::string& urdf, bool tool_t2_interfaces)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager cm{ executor, urdf, true, "test_controller_manager" };
  std::thread spinner([&executor]() { executor->spin(); });

  // This controller_manager does not run its own update thread. switch_controller only schedules
  // the change; read/update/write have to perform it.
  std::atomic<bool> run_update{ true };
  std::thread updater([&cm, &run_update]() {
    auto last = cm.now();
    while (run_update.load()) {
      const auto now = cm.now();
      const auto period = now - last;
      cm.read(now, period);
      cm.update(now, period);
      cm.write(now, period);
      last = now;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  const std::string name = "test_gpio_controller";
  if (tool_t2_interfaces) {
    const std::string params = std::string{ TEST_FILES_DIRECTORY } + "/gpio_controller_tool_t2_params.yaml";
    cm.set_parameter({ name + ".params_file", params });
  }
  cm.set_parameter({ name + ".type", "ur_controllers/GPIOController" });

  controller_interface::return_type result = controller_interface::return_type::ERROR;
  if (cm.load_controller(name) != nullptr &&
      cm.configure_controller(name) == controller_interface::return_type::OK) {
    result = cm.switch_controller({ name }, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT, false,
                                  rclcpp::Duration::from_seconds(5.0));
  }

  run_update.store(false);
  updater.join();
  executor->cancel();
  spinner.join();
  return result;
}
}  // namespace

TEST(TestGPIOControllerToolT2, activates_without_t2_interfaces_when_disabled)
{
  EXPECT_EQ(activate(makeUrdf(false), false), controller_interface::return_type::OK);
}

TEST(TestGPIOControllerToolT2, refuses_activation_when_t2_interfaces_are_missing)
{
  EXPECT_EQ(activate(makeUrdf(false), true), controller_interface::return_type::ERROR);
}

TEST(TestGPIOControllerToolT2, activates_with_t2_interfaces_when_enabled)
{
  EXPECT_EQ(activate(makeUrdf(true), true), controller_interface::return_type::OK);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
