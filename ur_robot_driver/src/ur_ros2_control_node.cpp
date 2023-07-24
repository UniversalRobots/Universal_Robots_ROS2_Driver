// Copyright 2022 Universal Robots A/S
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
//    * Neither the name of the Universal Robots A/S nor the names of its
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
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-1-7
 *
 */
//----------------------------------------------------------------------

#include <thread>
#include <memory>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

// code is inspired by
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create executor
  std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // create controller manager instance
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");

  // control loop thread
  std::thread control_loop([controller_manager]() {
    if (!realtime_tools::configure_sched_fifo(50)) {
      RCLCPP_WARN(controller_manager->get_logger(), "Could not enable FIFO RT scheduling policy");
    }

    // for calculating sleep time
    auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());
    auto const cm_now = std::chrono::nanoseconds(controller_manager->now().nanoseconds());
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{ cm_now };

    // for calculating the measured period of the loop
    rclcpp::Time previous_time = controller_manager->now();

    while (rclcpp::ok()) {
      // calculate measured period
      auto const current_time = controller_manager->now();
      auto const measured_period = current_time - previous_time;
      previous_time = current_time;

      // execute update loop
      controller_manager->read(controller_manager->now(), measured_period);
      controller_manager->update(controller_manager->now(), measured_period);
      controller_manager->write(controller_manager->now(), measured_period);

      // wait until we hit the end of the period
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }
  });

  // spin the executor with controller manager node
  e->add_node(controller_manager);
  e->spin();

  // wait for control loop to finish
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
