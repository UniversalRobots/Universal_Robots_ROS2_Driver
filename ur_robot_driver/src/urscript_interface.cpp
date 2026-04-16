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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class URScriptInterface : public rclcpp::Node
{
public:
  URScriptInterface() : Node("urscript_interface")
  {
    this->declare_parameter("robot_ip", rclcpp::PARAMETER_STRING);
    m_script_sub = this->create_subscription<std_msgs::msg::String>(
        "~/script_command", 1, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (m_primary_client == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Primary client not initialized yet");
            return false;
          }
          if (m_primary_client->sendScript(msg->data)) {
            URCL_LOG_INFO("Sent program to robot:\n%s", msg->data.c_str());
            return true;
          }

          URCL_LOG_ERROR("Could not send program to robot");
          return false;
        });
    m_primary_client = std::make_unique<urcl::primary_interface::PrimaryClient>(
        this->get_parameter("robot_ip").as_string(), m_notifier);
    m_primary_client->start();

    auto program_with_newline = std::string("sec "
                                            "urscript_interface_initialization():\ntextmsg(\"urscript_interface "
                                            "connected\")\nend");
    m_primary_client->sendScript(program_with_newline);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_script_sub;
  std::unique_ptr<urcl::primary_interface::PrimaryClient> m_primary_client;
  urcl::comm::INotifier m_notifier;
  // std::unique_ptr<urcl::comm::URStream<urcl::primary_interface::PrimaryPackage>> m_secondary_stream;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<URScriptInterface>());
  rclcpp::shutdown();
  return 0;
}
