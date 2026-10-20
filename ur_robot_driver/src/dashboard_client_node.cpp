// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <string>

#include <ur_robot_driver/dashboard_client_ros.hpp>
#include <ur_robot_driver/urcl_log_handler.hpp>
#include <rclcpp/logging.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ur_dashboard_client");

  // The IP address under which the robot is reachable.
  std::string robot_ip = node->declare_parameter<std::string>("robot_ip", "192.168.56.101");
  node->get_parameter<std::string>("robot_ip", robot_ip);

  ur_robot_driver::registerUrclLogHandler("");  // Set empty tf_prefix at the moment

  std::shared_ptr<ur_robot_driver::DashboardClientROS> client;
  try {
    client = std::make_shared<ur_robot_driver::DashboardClientROS>(node, robot_ip);
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"),
                 "Error raised during Dashboard Client startup: %s. Exiting dashboard client now.", e.what());
    return 1;
  }

  // Runs before spin() exists, so Ctrl-C can abort a blocking connect().
  rclcpp::on_shutdown([weak_client = std::weak_ptr<ur_robot_driver::DashboardClientROS>(client)]() {
    if (auto locked_client = weak_client.lock()) {
      locked_client->stop();
    }
  });

  if (node->get_parameter("autoconnect").as_bool()) {
    while (rclcpp::ok()) {
      try {
        if (client->connect()) {
          break;
        }
      } catch (const urcl::UrException& e) {
        if (!rclcpp::ok()) {
          break;
        }
        RCLCPP_ERROR(node->get_logger(), "Could not determine robot version: %s", e.what());
        client->stop();
        rclcpp::shutdown();
        return 1;
      }
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to connect to Dashboard Server at %s. Please check the IP address and ensure the robot is "
                   "powered on and has the dashboard server enabled. Retrying in 5 seconds.",
                   robot_ip.c_str());
      if (!rclcpp::ok()) {
        break;
      }
      rclcpp::sleep_for(std::chrono::seconds(5));
    }
  } else {
    RCLCPP_INFO(node->get_logger(),
                "Dashboard client started with autoconnect disabled. Call the ~/connect service to connect to %s.",
                robot_ip.c_str());
  }

  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }

  client->stop();
  rclcpp::shutdown();
  return 0;
}
