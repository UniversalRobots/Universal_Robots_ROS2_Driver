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

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ur_controllers/gpio_controller.hpp"
#include "ur_msgs/srv/set_io.hpp"

namespace
{
using ur_controllers::kDigitalOutputCount;
using ur_controllers::kStandardDigitalIoCount;
using ur_controllers::kT1ConfigurableDigitalInputCount;
using ur_controllers::kT1SmartPanelDigitalInputCount;

// Interfaces the GPIO controller claims. Names match gpio_controller.cpp. Connector T2 is
// added separately so a description without it can be tested.
const std::map<std::string, std::vector<std::string>> kCommandInterfaces = {
  { "gpio",
    [] {
      std::vector<std::string> names;
      for (size_t i = 0; i < kDigitalOutputCount; ++i) {
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
      for (size_t i = 0; i < kDigitalOutputCount; ++i) {
        names.push_back("digital_output_" + std::to_string(i));
      }
      for (size_t i = 0; i < kStandardDigitalIoCount; ++i) {
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
      for (size_t i = 0; i < kT1ConfigurableDigitalInputCount; ++i) {
        names.push_back("tool_t1_configurable_digital_input_" + std::to_string(i));
      }
      for (size_t i = 0; i < kT1SmartPanelDigitalInputCount; ++i) {
        names.push_back("tool_t1_digital_input_" + std::to_string(i));
      }
      names.push_back("tool_t1_connected");
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
      extra_states = interfaces("state_interface", { "tool_flange_type" }, "2") +
                     interfaces("state_interface", { "tool_output_voltage_2", "tool_output_current_2" }, "0");
    }
    const std::string initial = name == "system_interface" ? "1" : "0";
    gpios += "<gpio name=\"" + name + "\">" + interfaces("command_interface", commands, "") + extra_commands +
             interfaces("state_interface", states, initial) + extra_states + "</gpio>";
  }
  // system_interface has no command interfaces, so the loop above skipped it.
  gpios += "<gpio name=\"system_interface\"><state_interface name=\"initialized\">"
           "<param name=\"initial_value\">1</param></state_interface></gpio>";

  return "<robot name=\"gpio_test\"><link name=\"base_link\"/>"
         "<ros2_control name=\"gpio_test\" type=\"system\"><hardware>"
         "<plugin>mock_components/GenericSystem</plugin></hardware>" +
         gpios + "</ros2_control></robot>";
}

controller_interface::return_type activate(const std::string& urdf,
                                           const std::function<void(rclcpp::Node&)>& while_active = {})
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager cm{ executor, urdf, true, "test_controller_manager" };
  auto client_node = std::make_shared<rclcpp::Node>("test_gpio_controller_client");
  executor->add_node(client_node);
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
  cm.set_parameter({ name + ".type", "ur_controllers/GPIOController" });

  controller_interface::return_type result = controller_interface::return_type::ERROR;
  if (cm.load_controller(name) != nullptr && cm.configure_controller(name) == controller_interface::return_type::OK) {
    result = cm.switch_controller({ name }, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT, false,
                                  rclcpp::Duration::from_seconds(5.0));
  }
  if (result == controller_interface::return_type::OK && while_active) {
    while_active(*client_node);
  }

  run_update.store(false);
  updater.join();
  executor->cancel();
  spinner.join();
  return result;
}
}  // namespace

TEST(TestGPIOControllerToolT2, activates_with_t2_interfaces)
{
  EXPECT_EQ(activate(makeUrdf(true)), controller_interface::return_type::OK);
}

TEST(TestGPIOControllerToolT2, refuses_activation_without_t2_interfaces)
{
  EXPECT_EQ(activate(makeUrdf(false)), controller_interface::return_type::ERROR);
}

// Mock hardware never acknowledges io_async_success, so the request times out with the pending value.
TEST(TestGPIOControllerToolT2, unacknowledged_t2_voltage_request_fails)
{
  std::optional<bool> success;
  const auto result = activate(makeUrdf(true), [&success](rclcpp::Node& node) {
    auto client = node.create_client<ur_msgs::srv::SetIO>("/test_gpio_controller/set_io");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = ur_msgs::srv::SetIO::Request::FUN_SET_TOOL_VOLTAGE_T2;
    request->state = ur_msgs::srv::SetIO::Request::STATE_TOOL_VOLTAGE_0V;
    auto future = client->async_send_request(request);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    success = future.get()->success;
  });
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_TRUE(success.has_value());
  EXPECT_FALSE(*success);
}

// Tool flange type is the wire value latched from gpio/tool_flange_type at activation.
// 2 or greater is a V2 (or later) flange, which is the only flange with connectors T1 and T2.
class GPIOControllerFlangeServiceTest : public ::testing::TestWithParam<double>
{
protected:
  void SetUp() override
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "gpio_flange_test";
    params.robot_description = "";
    params.update_rate = 500;
    params.controller_manager_update_rate = 500;
    params.node_namespace = "";
    params.node_options = controller_.define_custom_node_options();
    ASSERT_EQ(controller_.init(params), controller_interface::return_type::OK);

    const auto command_names = controller_.command_interface_configuration().names;
    const auto state_names = controller_.state_interface_configuration().names;
    commands_.reserve(command_names.size());
    states_.reserve(state_names.size());
    for (const auto& full_name : command_names) {
      const auto slash = full_name.find('/');
      commands_.push_back(std::make_shared<hardware_interface::CommandInterface>(full_name.substr(0, slash),
                                                                                 full_name.substr(slash + 1)));
    }
    for (const auto& full_name : state_names) {
      const auto slash = full_name.find('/');
      auto state = std::make_shared<hardware_interface::StateInterface>(full_name.substr(0, slash),
                                                                        full_name.substr(slash + 1), "double", "0");
      if (full_name == "system_interface/initialized") {
        ASSERT_TRUE(state->set_value(1.0));
      }
      if (full_name == "gpio/tool_flange_type") {
        ASSERT_TRUE(state->set_value(GetParam()));
      }
      states_.push_back(state);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(commands_.size());
    loaned_states.reserve(states_.size());
    for (const auto& command : commands_) {
      loaned_commands.emplace_back(command, []() {});
    }
    for (const auto& state : states_) {
      loaned_states.emplace_back(state);
    }
    controller_.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));

    const rclcpp_lifecycle::State previous_state;
    ASSERT_EQ(controller_.on_configure(previous_state), controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(controller_.on_activate(previous_state), controller_interface::CallbackReturn::SUCCESS);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(controller_.get_node()->get_node_base_interface());
    spinner_ = std::thread([this]() { executor_->spin(); });

    acknowledge_ = true;
    acker_ = std::thread([this]() {
      auto io_async = find_command("gpio/io_async_success");
      while (acknowledge_.load()) {
        const auto value = io_async->get_optional<double>();
        if (value.has_value() && *value == 2.0) {
          std::ignore = io_async->set_value(1.0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    client_node_ = std::make_shared<rclcpp::Node>("gpio_flange_set_io_client");
    const std::string service_name = std::string(controller_.get_node()->get_fully_qualified_name()) + "/set_io";
    client_ = client_node_->create_client<ur_msgs::srv::SetIO>(service_name);
    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override
  {
    acknowledge_.store(false);
    if (acker_.joinable()) {
      acker_.join();
    }
    if (executor_) {
      executor_->cancel();
    }
    if (spinner_.joinable()) {
      spinner_.join();
    }
    controller_.release_interfaces();
    client_.reset();
    client_node_.reset();
    if (controller_.get_node()) {
      controller_.get_node()->shutdown();
    }
  }

  hardware_interface::CommandInterface::SharedPtr find_command(const std::string& name) const
  {
    for (const auto& command : commands_) {
      if (command->get_name() == name) {
        return command;
      }
    }
    return nullptr;
  }

  ur_msgs::srv::SetIO::Response::SharedPtr call_set_io(int8_t fun, int8_t pin, float state)
  {
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = fun;
    request->pin = pin;
    request->state = state;
    auto future = client_->async_send_request(request);
    EXPECT_EQ(rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  bool connectors_present() const
  {
    return GetParam() >= 2.0;
  }

  ur_controllers::GPIOController controller_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
  std::vector<hardware_interface::StateInterface::SharedPtr> states_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  std::thread acker_;
  std::atomic<bool> acknowledge_{ false };
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client_;
};

TEST_P(GPIOControllerFlangeServiceTest, set_io_answers_from_tool_flange_type)
{
  using Request = ur_msgs::srv::SetIO::Request;

  const auto t1 = call_set_io(Request::FUN_SET_DIGITAL_OUT, Request::PIN_TOOL_T1_DOUT0, Request::STATE_ON);
  ASSERT_NE(t1, nullptr);
  EXPECT_EQ(t1->success, connectors_present());
  const auto t1_command = find_command("gpio/standard_digital_output_cmd_18");
  ASSERT_NE(t1_command, nullptr);
  const auto t1_value = t1_command->get_optional<double>();
  ASSERT_TRUE(t1_value.has_value());
  if (connectors_present()) {
    EXPECT_DOUBLE_EQ(*t1_value, Request::STATE_ON);
  } else {
    EXPECT_TRUE(std::isnan(*t1_value));
  }

  const auto t2 = call_set_io(Request::FUN_SET_TOOL_VOLTAGE_T2, 0, Request::STATE_TOOL_VOLTAGE_24V);
  ASSERT_NE(t2, nullptr);
  EXPECT_EQ(t2->success, connectors_present());
  const auto t2_command = find_command("gpio/tool_voltage_t2_cmd");
  ASSERT_NE(t2_command, nullptr);
  const auto t2_value = t2_command->get_optional<double>();
  ASSERT_TRUE(t2_value.has_value());
  if (connectors_present()) {
    EXPECT_DOUBLE_EQ(*t2_value, Request::STATE_TOOL_VOLTAGE_24V);
  } else {
    EXPECT_TRUE(std::isnan(*t2_value));
  }

  // 12 V is not a T2 voltage, including on a V2 flange.
  const auto rejected_voltage = call_set_io(Request::FUN_SET_TOOL_VOLTAGE_T2, 0, Request::STATE_TOOL_VOLTAGE_12V);
  ASSERT_NE(rejected_voltage, nullptr);
  EXPECT_FALSE(rejected_voltage->success);

  // Standard and T0 outputs do not depend on the tool flange.
  const auto standard = call_set_io(Request::FUN_SET_DIGITAL_OUT, Request::PIN_DOUT0, Request::STATE_ON);
  ASSERT_NE(standard, nullptr);
  EXPECT_TRUE(standard->success);
  const auto tool_t0 = call_set_io(Request::FUN_SET_DIGITAL_OUT, Request::PIN_TOOL_DOUT0, Request::STATE_ON);
  ASSERT_NE(tool_t0, nullptr);
  EXPECT_TRUE(tool_t0->success);
}

INSTANTIATE_TEST_SUITE_P(ToolFlangeTypes, GPIOControllerFlangeServiceTest, ::testing::Values(0.0, 1.0, 2.0),
                         [](const ::testing::TestParamInfo<double>& info) {
                           if (info.param == 2.0) {
                             return std::string("v2");
                           }
                           if (info.param == 1.0) {
                             return std::string("v1");
                           }
                           return std::string("unknown");
                         });

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
