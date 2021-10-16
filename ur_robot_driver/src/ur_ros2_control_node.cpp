// Copyright 2020 ROS2-Control Development Team
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
 * \date    2019-04-11
 * \author  Lovro Ivanov
 * \date    2021-10-15
 *
 */
//----------------------------------------------------------------------

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  std::thread cm_thread([cm]() {
    RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

    rclcpp::Time timestamp = cm->now();

    // Use nanoseconds to avoid chrono's rounding
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000 / cm->get_update_rate()));
    auto t1 = std::chrono::steady_clock::now();
    auto t2 = std::chrono::steady_clock::now();
    auto expected_period = static_cast<int64_t>(1000000000.0 / (cm->get_update_rate()));

    while (rclcpp::ok()) {
      cm->read();
      timestamp = cm->now();
      t1 = std::chrono::steady_clock::now();
      rclcpp::Duration period = rclcpp::Duration::from_nanoseconds((t1 - t2).count());
      t2 = t1;
      cm->update(timestamp, period);
      cm->write();

      if (period.nanoseconds() > expected_period) {
        //            RCLCPP_WARN(cm->get_logger(), " Could not keep cycle rate of '%ld' ns", expected_period);
        //            RCLCPP_WARN(cm->get_logger(), " Actual cycle time: '%ld' ns ", period.nanoseconds());
      }
    }
  });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
