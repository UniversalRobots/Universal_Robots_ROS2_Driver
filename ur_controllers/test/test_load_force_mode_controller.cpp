// Copyright 2024, Universal Robots A/S
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
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <tuple>
#include <vector>
#include "controller_interface/controller_interface_params.hpp"
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "ur_controllers/force_mode_controller.hpp"

TEST(TestLoadForceModeController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm{ executor, ros2_control_test_assets::minimal_robot_urdf, true,
                                            "test_controller_manager" };

  const std::string test_file_path = std::string{ TEST_FILES_DIRECTORY } + "/force_mode_controller_params.yaml";
  cm.set_parameter({ "test_force_mode_controller.params_file", test_file_path });

  cm.set_parameter({ "test_force_mode_controller.type", "ur_controllers/ForceModeController" });

  ASSERT_NE(cm.load_controller("test_force_mode_controller"), nullptr);
}

class ForceModeCancellationTest : public ::testing::TestWithParam<std::tuple<bool, bool>>
{
protected:
  void SetUp() override
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "force_mode_cancellation_test";
    params.update_rate = 500;
    params.controller_manager_update_rate = 500;
    params.node_options = controller_.define_custom_node_options();
    params.node_options.parameter_overrides({ rclcpp::Parameter("check_io_successful_retries", 1) });
    ASSERT_EQ(controller_.init(params), controller_interface::return_type::OK);
    ASSERT_EQ(controller_.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    const auto names = controller_.command_interface_configuration().names;
    values_.resize(names.size(), std::numeric_limits<double>::quiet_NaN());
    std::vector<hardware_interface::LoanedCommandInterface> loaned_interfaces;
    for (size_t index = 0; index < names.size(); ++index) {
      const auto separator = names[index].find('/');
      auto interface = std::make_shared<hardware_interface::CommandInterface>(
          names[index].substr(0, separator), names[index].substr(separator + 1), &values_[index]);
      loaned_interfaces.emplace_back(interface);
      interfaces_.push_back(std::move(interface));
    }
    controller_.assign_interfaces(std::move(loaned_interfaces), {});
    ASSERT_EQ(controller_.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = std::make_shared<rclcpp::Node>("force_mode_cancellation_client");
    start_client_ = client_node_->create_client<ur_msgs::srv::SetForceMode>("/force_mode_cancellation_test/"
                                                                            "start_force_mode");
    stop_client_ = client_node_->create_client<std_srvs::srv::Trigger>("/force_mode_cancellation_test/stop_force_mode");
    client_executor_.add_node(client_node_);
    server_executor_.add_node(controller_.get_node()->get_node_base_interface());
    server_thread_ = std::thread([this]() { server_executor_.spin(); });
    ASSERT_TRUE(start_client_->wait_for_service(std::chrono::seconds(1)));
    ASSERT_TRUE(stop_client_->wait_for_service(std::chrono::seconds(1)));
  }

  void TearDown() override
  {
    update();
    server_executor_.cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  std::shared_ptr<ur_msgs::srv::SetForceMode::Request> startRequest()
  {
    auto request = std::make_shared<ur_msgs::srv::SetForceMode::Request>();
    request->task_frame.header.frame_id = "base";
    request->task_frame.pose.orientation.w = 1.0;
    request->type = 2;
    request->damping_factor = 0.025;
    request->gain_scaling = 0.5;
    return request;
  }

  void update()
  {
    EXPECT_EQ(controller_.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002)),
              controller_interface::return_type::OK);
  }

  template <typename Future>
  void checkCancellation(Future& pending)
  {
    if (std::get<1>(GetParam())) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (values_[ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS] != 2.0 &&
             std::chrono::steady_clock::now() < deadline) {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      ASSERT_DOUBLE_EQ(values_[ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS], 2.0);
    }

    const auto result = client_executor_.spin_until_future_complete(pending, std::chrono::seconds(1));
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      update();
      result = client_executor_.spin_until_future_complete(pending, std::chrono::seconds(1));
    }
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(pending.get()->success);
    EXPECT_EQ(controller_.get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto rejected_start = start_client_->async_send_request(startRequest());
    ASSERT_EQ(client_executor_.spin_until_future_complete(rejected_start, std::chrono::milliseconds(100)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(rejected_start.get()->success);
    auto rejected_stop = stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(client_executor_.spin_until_future_complete(rejected_stop, std::chrono::milliseconds(100)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(rejected_stop.get()->success);

    update();
    for (size_t index = 0; index < values_.size(); ++index) {
      if (index == ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS) {
        EXPECT_DOUBLE_EQ(values_[index], 0.0);
      } else {
        EXPECT_TRUE(std::isnan(values_[index]));
      }
    }

    auto accepted_stop = stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (client_executor_.spin_until_future_complete(accepted_stop, std::chrono::milliseconds(1)) !=
               rclcpp::FutureReturnCode::SUCCESS &&
           std::chrono::steady_clock::now() < deadline) {
      update();
      if (values_[ur_controllers::CommandInterfaces::FORCE_MODE_DISABLE_CMD] == 1.0) {
        values_[ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS] = 1.0;
      }
    }
    ASSERT_EQ(client_executor_.spin_until_future_complete(accepted_stop, std::chrono::milliseconds(1)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(accepted_stop.get()->success);
  }

  std::vector<double> values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> interfaces_;
  ur_controllers::ForceModeController controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<ur_msgs::srv::SetForceMode>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;
  rclcpp::executors::SingleThreadedExecutor server_executor_;
  std::thread server_thread_;
};

TEST_P(ForceModeCancellationTest, StalledUpdatesBoundCancellationAndBlockRequestsUntilWithdrawal)
{
  if (std::get<0>(GetParam())) {
    auto pending = start_client_->async_send_request(startRequest());
    checkCancellation(pending);
  } else {
    auto pending = stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    checkCancellation(pending);
  }
}

TEST_P(ForceModeCancellationTest, DeactivationWithdrawsOldCommandAndStagesHardwareDisable)
{
  if (std::get<0>(GetParam())) {
    for (size_t index = 0; index < values_.size(); ++index) {
      if (index != ur_controllers::CommandInterfaces::FORCE_MODE_DISABLE_CMD) {
        values_[index] = 1.0;
      }
    }
  } else {
    values_[ur_controllers::CommandInterfaces::FORCE_MODE_DISABLE_CMD] = 1.0;
  }
  values_[ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS] = std::get<1>(GetParam()) ? 2.0 : 1.0;

  ASSERT_EQ(controller_.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  for (size_t index = 0; index < values_.size(); ++index) {
    if (index == ur_controllers::CommandInterfaces::FORCE_MODE_DISABLE_CMD) {
      EXPECT_DOUBLE_EQ(values_[index], 1.0);
    } else if (index == ur_controllers::CommandInterfaces::FORCE_MODE_ASYNC_SUCCESS) {
      EXPECT_DOUBLE_EQ(values_[index], 2.0);
    } else {
      EXPECT_TRUE(std::isnan(values_[index]));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(StartAndStop, ForceModeCancellationTest,
                         ::testing::Combine(::testing::Bool(), ::testing::Bool()));

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
