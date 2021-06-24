// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/dashboard_client_ros.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ur_robot_driver/urcl_log_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ur_dashboard_client");

  // The IP address under which the robot is reachable.
  std::string robot_ip = node->declare_parameter<std::string>("robot_ip", "192.168.56.101");
  node->get_parameter<std::string>("robot_ip", robot_ip);

  ur_robot_driver::registerUrclLogHandler();

  ur_robot_driver::DashboardClientROS client(node, robot_ip);

  rclcpp::spin(node);

  return 0;
}
