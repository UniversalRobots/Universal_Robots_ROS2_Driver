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
 * \date    2023-06-20
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/primary/primary_client.h>

#include <chrono>
#include <thread>

#include <ur_msgs/action/send_script.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>

using SendScript = ur_msgs::action::SendScript;

class URScriptInterface : public rclcpp::Node
{
public:
  URScriptInterface() : Node("urscript_interface")
  {
    this->declare_parameter("robot_ip", rclcpp::PARAMETER_STRING);
    this->declare_parameter("retry_on_readonly_interface", true);

    primary_client_ =
        std::make_unique<urcl::primary_interface::PrimaryClient>(this->get_parameter("robot_ip").as_string(), notif_);

    primary_client_->start(10, std::chrono::seconds(10));

    script_sub_ = create_subscription<std_msgs::msg::String>(
        "~/script_command", 1, std::bind(&URScriptInterface::script_callback, this, std::placeholders::_1));

    send_script_server_ = rclcpp_action::create_server<SendScript>(
        this, "~/execute_script",
        std::bind(&URScriptInterface::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&URScriptInterface::handle_cancel, this, std::placeholders::_1),
        std::bind(&URScriptInterface::handle_accepted, this, std::placeholders::_1));
  }

  ~URScriptInterface() override
  {
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
  }

private:
  // Use non-blocking interface
  bool script_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (primary_client_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "Primary client not initialized yet");
      return false;
    }
    if (busy) {
      RCLCPP_ERROR(get_logger(), "Script interface is executing action, ignoring topic");
      return false;
    }
    if (primary_client_->sendScript(msg->data)) {
      URCL_LOG_INFO("Sent program to robot:\n%s", msg->data.c_str());
      return true;
    } else {
      URCL_LOG_ERROR("Could not send program to robot");
      return false;
    }
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const SendScript::Goal> goal)
  {
    if (primary_client_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "Primary client not initialized yet");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (busy) {
      RCLCPP_ERROR(get_logger(), "Action server is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy = true;

    if (execute_thread_.joinable()) {
      // Should return immediately, as we are not busy
      execute_thread_.join();
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SendScript>> goal_handle)
  {
    if (goal_handle->is_executing()) {
      try {
        primary_client_->commandStop();
      } catch (const urcl::UrException& exc) {
        RCLCPP_ERROR(get_logger(), "Caught UR exception while trying to cancel goal:");
        RCLCPP_ERROR(get_logger(), exc.what());
      } catch (const std::exception& exc) {
        RCLCPP_ERROR(get_logger(), "Caught unexpected exception while trying to cancel goal:");
        RCLCPP_ERROR(get_logger(), exc.what());
      }
      return rclcpp_action::CancelResponse::ACCEPT;
    } else {
      RCLCPP_ERROR(get_logger(), "Received cancel request for inactive goal, rejecting");
      return rclcpp_action::CancelResponse::REJECT;
    }
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SendScript>> goal_handle)
  {
    execute_thread_ = std::thread([this, goal_handle]() { this->execute(goal_handle); });
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SendScript>> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto timeout = goal->start_timeout;
    auto chrono_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::seconds(timeout.sec) + std::chrono::nanoseconds(timeout.nanosec));
    // A ROS duration can be negative, would not make sense here
    if (chrono_timeout < std::chrono::milliseconds(0)) {
      chrono_timeout = std::chrono::milliseconds(0);
    }

    try {
      primary_client_->sendScriptBlocking(goal->program, goal->script_name, chrono_timeout, goal->fail_on_warnings,
                                          this->get_parameter("retry_on_readonly_interface").as_bool());
    }

    catch (const urcl::UrException& exc) {
      RCLCPP_ERROR(get_logger(), "Script did not execute successfully. Error message:");
      RCLCPP_ERROR(get_logger(), exc.what());
      auto res = std::make_shared<SendScript::Result>();
      res->success = false;
      res->message = exc.what();
      goal_handle->abort(res);
      busy = false;
      return;
    }

    catch (const std::exception& exc) {
      RCLCPP_ERROR(get_logger(), "Unexpected exception caught during script execution:");
      RCLCPP_ERROR(get_logger(), exc.what());
      auto res = std::make_shared<SendScript::Result>();
      res->success = false;
      res->message = exc.what();
      goal_handle->abort(res);
      busy = false;
      return;
    }

    auto res = std::make_shared<SendScript::Result>();
    if (goal_handle->is_canceling()) {
      res->success = false;
      res->message = "Script execution canceled";
      goal_handle->canceled(res);
    } else {
      res->success = true;
      res->message = "Script executed successfully";
      goal_handle->succeed(res);
    }
    busy = false;
    return;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr script_sub_;
  rclcpp_action::Server<SendScript>::SharedPtr send_script_server_;
  std::unique_ptr<urcl::primary_interface::PrimaryClient> primary_client_;
  urcl::comm::INotifier notif_;
  std::atomic<bool> busy = false;
  std::thread execute_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<URScriptInterface>());
  rclcpp::shutdown();
  return 0;
}
