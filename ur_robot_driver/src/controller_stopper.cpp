// Copyright 2019 FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-12
 *
 * \author  Mads Holm Peters
 * \date    2022-02-25
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <string>
#include <vector>

#include "ur_robot_driver/controller_stopper.hpp"

ControllerStopper::ControllerStopper(const rclcpp::Node::SharedPtr& node, bool stop_controllers_on_startup)
  : node_(node), stop_controllers_on_startup_(stop_controllers_on_startup), robot_running_(true)
{
  // Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes
  // on changes. However, this node only reacts on state changes, so a state published each cycle
  // would also be fine.
  robot_running_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "io_and_status_controller/robot_program_running", 1,
      std::bind(&ControllerStopper::robotRunningCallback, this, std::placeholders::_1));

  // Controller manager service to switch controllers
  controller_manager_srv_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/"
                                                                                                 "switch_controller");
  // Controller manager service to list controllers
  controller_list_srv_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("controller_manager/"
                                                                                             "list_controllers");

  RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Waiting for switch controller service to come up on "
                                                        "controller_manager/switch_controller");
  controller_manager_srv_->wait_for_service();
  RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");
  RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Waiting for list controllers service to come up on "
                                                        "controller_manager/list_controllers");
  controller_list_srv_->wait_for_service();
  RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

  consistent_controllers_ =
      node_->declare_parameter<std::vector<std::string>>("consistent_controllers", std::vector<std::string>(0));

  if (stop_controllers_on_startup_ == true) {
    while (stopped_controllers_.empty()) {
      auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
      auto future = controller_list_srv_->async_send_request(request);
      rclcpp::spin_until_future_complete(node_, future);
      auto result = future.get();
      for (auto& controller : result->controller) {
        // Check if in consistent_controllers
        // Else:
        //   Add to stopped_controllers
        if (controller.state == "active") {
          auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
          if (it == consistent_controllers_.end()) {
            stopped_controllers_.push_back(controller.name);
          }
        }
      }
    }
    auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request_switch_controller->strictness = request_switch_controller->STRICT;
    request_switch_controller->stop_controllers = stopped_controllers_;
    auto future = controller_manager_srv_->async_send_request(request_switch_controller);
    rclcpp::spin_until_future_complete(node_, future);
    if (future.get()->ok == false) {
      RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate requested controllers");
    }
    robot_running_ = false;
  }
}

void ControllerStopper::findAndStopControllers()
{
  stopped_controllers_.clear();
  auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  auto request_list_controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

  // Callback to switch controllers
  auto callback_switch_controller =
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
        auto result = future_response.get();
        if (result->ok == false) {
          RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate requested controllers");
        }
      };

  // Callback to list controllers
  auto callback_list_controller =
      [this, request_switch_controller, callback_switch_controller](
          rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future_response) {
        auto result = future_response.get();
        for (auto& controller : result->controller) {
          // Check if in consistent_controllers
          // Else:
          //   Add to stopped_controllers
          if (controller.state == "active") {
            auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
            if (it == consistent_controllers_.end()) {
              stopped_controllers_.push_back(controller.name);
            }
          }
        }
        request_switch_controller->strictness = request_switch_controller->STRICT;
        if (!stopped_controllers_.empty()) {
          request_switch_controller->stop_controllers = stopped_controllers_;
          auto future =
              controller_manager_srv_->async_send_request(request_switch_controller, callback_switch_controller);
        }
      };

  auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::startControllers()
{
  // Callback to switch controllers
  auto callback = [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
    auto result = future_response.get();
    if (result->ok == false) {
      RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not activate requested controllers");
    }
  };
  if (!stopped_controllers_.empty()) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = request->STRICT;
    request->start_controllers = stopped_controllers_;
    auto future = controller_manager_srv_->async_send_request(request, callback);
  }
}

void ControllerStopper::robotRunningCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "robotRunningCallback with data %d", msg->data);

  if (msg->data && !robot_running_) {
    RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "Starting controllers");
    startControllers();
  } else if (!msg->data && robot_running_) {
    RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "Stopping controllers");
    // stop all controllers except the once in consistent_controllers_
    findAndStopControllers();
  }
  robot_running_ = msg->data;
}
