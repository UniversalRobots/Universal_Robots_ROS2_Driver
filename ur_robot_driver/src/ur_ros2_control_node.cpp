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

#include <pthread.h>
#include <thread>
#include <memory>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

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
    // use fixed time step
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

    while (rclcpp::ok()) {
      // ur client library is blocking and is the one that is controlling time step
      controller_manager->read();
      controller_manager->update(controller_manager->now(), dt);
      controller_manager->write();
    }
  });

  auto control_loop_handle = control_loop.native_handle();
  sched_param params;
  int policy;
  bool sched_success = true;
  sched_success &= (pthread_getschedparam(control_loop_handle, &policy, &params) == 0);
  if (sched_success) {
    RCLCPP_INFO(controller_manager->get_logger(),
                "Control loop default sched parameters: \n    * sched priority: '%d \n    * policy: '%d' ",
                params.sched_priority, policy);
  } else {
    RCLCPP_ERROR(controller_manager->get_logger(), "Unable to fetch control loop sched parameters!");
  }

  // priority to 49
  params.sched_priority = (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2 - 1;
  sched_success &= (pthread_setschedparam(control_loop_handle, SCHED_FIFO, &params) == 0);
  if (sched_success) {
    RCLCPP_INFO(controller_manager->get_logger(), "Control loop thread priority set successfully. Verifying "
                                                  "priority...");
    sched_success &= (pthread_getschedparam(control_loop_handle, &policy, &params) == 0);
    if (!sched_success) {
      RCLCPP_ERROR(controller_manager->get_logger(), "Unable to retrieve scheduling parameters to verify changes...");
    }
    if (policy != SCHED_FIFO) {
      RCLCPP_ERROR(controller_manager->get_logger(), "Scheduling policy is not SCHED_FIFO.");
    } else {
      RCLCPP_INFO(controller_manager->get_logger(), "SCHED_FIFO set as scheduling policy.");
    }
    RCLCPP_INFO(controller_manager->get_logger(), "Control loop thread priority is '%d'. ", params.sched_priority);
    pthread_setname_np(control_loop_handle, "ctrl_loop_ur");
  } else {
    RCLCPP_ERROR(controller_manager->get_logger(), "Unable to set control loop thread priority!");
  }

  // spin the executor with controller manager node
  e->add_node(controller_manager);
  e->spin();

  // wait for control loop to finish
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
