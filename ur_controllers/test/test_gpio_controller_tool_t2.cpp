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

// cppcheck-suppress-file syntaxError

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ur_controllers/gpio_controller.hpp"
#include "ur_msgs/srv/set_io.hpp"

namespace
{
constexpr double kAsyncWaiting = 2.0;

// Tool flange type is the wire value latched from gpio/tool_flange_type at activation.
// 2 or greater is a V2 (or later) flange, which is the only flange with connectors T1 and T2.
class GPIOControllerFlangeServiceTest : public ::testing::TestWithParam<double>
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    ASSERT_EQ(controller_.init("gpio_flange_test", "", options), controller_interface::return_type::OK);

    const auto command_names = controller_.command_interface_configuration().names;
    const auto state_names = controller_.state_interface_configuration().names;
    command_values_.assign(command_names.size(), std::numeric_limits<double>::quiet_NaN());
    state_values_.assign(state_names.size(), 0.0);
    commands_.reserve(command_names.size());
    states_.reserve(state_names.size());
    for (size_t i = 0; i < command_names.size(); ++i) {
      const auto slash = command_names[i].find('/');
      commands_.emplace_back(command_names[i].substr(0, slash), command_names[i].substr(slash + 1),
                             &command_values_[i]);
    }
    for (size_t i = 0; i < state_names.size(); ++i) {
      const auto slash = state_names[i].find('/');
      if (state_names[i] == "system_interface/initialized") {
        state_values_[i] = 1.0;
      }
      if (state_names[i] == "gpio/tool_flange_type") {
        state_values_[i] = GetParam();
      }
      states_.emplace_back(state_names[i].substr(0, slash), state_names[i].substr(slash + 1), &state_values_[i]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    loaned_commands.reserve(commands_.size());
    loaned_states.reserve(states_.size());
    for (auto& command : commands_) {
      loaned_commands.emplace_back(command);
    }
    for (auto& state : states_) {
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
      auto* io_async = find_command("gpio/io_async_success");
      while (acknowledge_.load()) {
        if (io_async != nullptr && io_async->get_value() == kAsyncWaiting) {
          io_async->set_value(1.0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    client_node_ = std::make_shared<rclcpp::Node>("gpio_flange_set_io_client");
    const std::string service_name =
        std::string(controller_.get_node()->get_node_base_interface()->get_fully_qualified_name()) + "/set_io";
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

  hardware_interface::CommandInterface* find_command(const std::string& name)
  {
    for (auto& command : commands_) {
      if (command.get_name() == name) {
        return &command;
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
    auto future_return = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    EXPECT_EQ(future_return, rclcpp::FutureReturnCode::SUCCESS);
    if (future_return != rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  bool connectors_present() const
  {
    return GetParam() >= 2.0;
  }

  ur_controllers::GPIOController controller_;
  std::vector<double> command_values_;
  std::vector<double> state_values_;
  std::vector<hardware_interface::CommandInterface> commands_;
  std::vector<hardware_interface::StateInterface> states_;
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
  const auto* t1_command = find_command("gpio/standard_digital_output_cmd_18");
  ASSERT_NE(t1_command, nullptr);
  if (connectors_present()) {
    EXPECT_DOUBLE_EQ(t1_command->get_value(), Request::STATE_ON);
  } else {
    EXPECT_TRUE(std::isnan(t1_command->get_value()));
  }

  const auto t2 = call_set_io(Request::FUN_SET_TOOL_VOLTAGE_T2, 0, Request::STATE_TOOL_VOLTAGE_24V);
  ASSERT_NE(t2, nullptr);
  EXPECT_EQ(t2->success, connectors_present());
  const auto* t2_command = find_command("gpio/tool_voltage_t2_cmd");
  ASSERT_NE(t2_command, nullptr);
  if (connectors_present()) {
    EXPECT_DOUBLE_EQ(t2_command->get_value(), Request::STATE_TOOL_VOLTAGE_24V);
  } else {
    EXPECT_TRUE(std::isnan(t2_command->get_value()));
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

// Mock hardware never acknowledges io_async_success, so the request times out with the pending value.
TEST(TestGPIOControllerToolT2, unacknowledged_t2_voltage_request_fails)
{
  ur_controllers::GPIOController controller;
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  ASSERT_EQ(controller.init("gpio_t2_timeout_test", "", options), controller_interface::return_type::OK);

  const auto command_names = controller.command_interface_configuration().names;
  const auto state_names = controller.state_interface_configuration().names;
  std::vector<double> command_values(command_names.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<double> state_values(state_names.size(), 0.0);
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  commands.reserve(command_names.size());
  states.reserve(state_names.size());
  for (size_t i = 0; i < command_names.size(); ++i) {
    const auto slash = command_names[i].find('/');
    commands.emplace_back(command_names[i].substr(0, slash), command_names[i].substr(slash + 1), &command_values[i]);
  }
  for (size_t i = 0; i < state_names.size(); ++i) {
    const auto slash = state_names[i].find('/');
    if (state_names[i] == "system_interface/initialized") {
      state_values[i] = 1.0;
    }
    if (state_names[i] == "gpio/tool_flange_type") {
      state_values[i] = 2.0;
    }
    states.emplace_back(state_names[i].substr(0, slash), state_names[i].substr(slash + 1), &state_values[i]);
  }

  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_commands.reserve(commands.size());
  loaned_states.reserve(states.size());
  for (auto& command : commands) {
    loaned_commands.emplace_back(command);
  }
  for (auto& state : states) {
    loaned_states.emplace_back(state);
  }
  controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));

  const rclcpp_lifecycle::State previous_state;
  ASSERT_EQ(controller.on_configure(previous_state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.on_activate(previous_state), controller_interface::CallbackReturn::SUCCESS);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(controller.get_node()->get_node_base_interface());
  std::thread spinner([&executor]() { executor->spin(); });

  auto client_node = std::make_shared<rclcpp::Node>("gpio_t2_timeout_client");
  const std::string service_name =
      std::string(controller.get_node()->get_node_base_interface()->get_fully_qualified_name()) + "/set_io";
  auto client = client_node->create_client<ur_msgs::srv::SetIO>(service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun = ur_msgs::srv::SetIO::Request::FUN_SET_TOOL_VOLTAGE_T2;
  request->state = ur_msgs::srv::SetIO::Request::STATE_TOOL_VOLTAGE_0V;
  auto future = client->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(future.get()->success);

  executor->cancel();
  spinner.join();
  controller.release_interfaces();
  if (controller.get_node()) {
    controller.get_node()->shutdown();
  }
}
}  // namespace

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
