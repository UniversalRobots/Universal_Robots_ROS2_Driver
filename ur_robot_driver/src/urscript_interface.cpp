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

#include <ur_client_library/comm/stream.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/primary/primary_package.h>

#include <memory>

#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class ErrorCodeConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
{
public:
  virtual bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> pkg)
  {
    auto error_msg = std::dynamic_pointer_cast<urcl::primary_interface::ErrorCodeMessage>(pkg);
    if (error_msg != nullptr) {
      if (error_msg->message_code_ == 210)  // Socket is read-only
      {
        RCLCPP_WARN(rclcpp::get_logger("urscript_interface"), "Received error code 210. The script code that caused "
                                                              "this error code to be sent to the client was not "
                                                              "executed on the robot. This can be the robot not being "
                                                              "in remote control mode or the client has not been "
                                                              "reconnected when switching from remote to local control "
                                                              "mode. Reconnection is required to be able to send "
                                                              "script code to the robot again.");
        reconnect_required_ = true;
      }
    }
    return true;
  }

  bool isReconnectRequired() const
  {
    return reconnect_required_;
  }

  void clearReconnectRequired()
  {
    reconnect_required_ = false;
  }

private:
  std::atomic<bool> reconnect_required_{ false };
};

class URScriptInterface : public rclcpp::Node
{
public:
  URScriptInterface() : Node("urscript_interface")
  {
    this->declare_parameter("robot_ip", rclcpp::PARAMETER_STRING);
    this->declare_parameter("reconnect_automatically", true);
    script_sub_ = this->create_subscription<std_msgs::msg::String>(
        "~/script_command", 1, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (primary_client_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Primary client not initialized yet");
            return false;
          }
          if (primary_client_->sendScript(msg->data)) {
            URCL_LOG_INFO("Sent program to robot:\n%s", msg->data.c_str());
            return true;
          }

          URCL_LOG_ERROR("Could not send program to robot");
          return false;
        });
    error_code_consumer_ = std::make_shared<ErrorCodeConsumer>();
    reconnect();

    reconnect_srv_ =
        create_service<std_srvs::srv::Trigger>("~/reconnect", [this](std_srvs::srv::Trigger::Request::SharedPtr req,
                                                                     std_srvs::srv::Trigger::Response::SharedPtr resp) {
          return this->reconnectSrvCallback(req, resp);
        });
    reconnect_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (get_parameter("reconnect_automatically").as_bool() && error_code_consumer_->isReconnectRequired()) {
        RCLCPP_INFO(this->get_logger(), "Reconnect required. Reconnecting primary interface client...");
        reconnect();
        error_code_consumer_->clearReconnectRequired();
      }
    });
  }

private:
  bool reconnectSrvCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                            std_srvs::srv::Trigger::Response::SharedPtr resp)
  {
    reconnect();
    resp->success = true;
    return true;
  }

  void reconnect()
  {
    primary_client_ = std::make_unique<urcl::primary_interface::PrimaryClient>(
        this->get_parameter("robot_ip").as_string(), notifier_);
    primary_client_->addPrimaryConsumer(error_code_consumer_);
    primary_client_->start();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr script_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_srv_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  std::unique_ptr<urcl::primary_interface::PrimaryClient> primary_client_;
  urcl::comm::INotifier notifier_;
  std::shared_ptr<ErrorCodeConsumer> error_code_consumer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<URScriptInterface>());
  rclcpp::shutdown();
  return 0;
}
