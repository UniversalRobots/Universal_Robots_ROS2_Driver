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

// code is inspired by
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp








#include <pthread.h>

bool setRealtimePriority(int priority, int cpu_core){
  //Taken from https://github.com/leggedrobotics/ethercat_sdk_master/blob/6b420bc1785cf26324aab62c79347b2a6e07924d/src/ethercat_sdk_master/EthercatMaster.cpp#L121
  bool success = true;
  //Handle to our thread
  pthread_t thread = pthread_self();
  //First set the priority of the thread in the scheduler to the passed priority (99 is max)
  sched_param param;
  param.sched_priority = priority;
  int errorFlag = pthread_setschedparam(thread, SCHED_FIFO, &param);
  if(errorFlag != 0){
    std::cerr << "[ur_ros2_control_node::setRealtimePriority]"
                      << " Could not set thread priority. Check limits.conf or"
                      << " execute as root";
    success &= false;
  }

  //Allow attaching the thread to a certain cpu core
  //Create an empty cpu set for the scheduler
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  //Obtain amount of cpus
  int number_of_cpus = sysconf(_SC_NPROCESSORS_ONLN);

  //In case the user passed a core value > 0
  if(cpu_core > 0 )
  {
      //check if the core is < than the number of available cpus
      if(cpu_core >= number_of_cpus)
      {
          std::cerr <<"[ur_ros2_control_node::setRealtimePriority]" <<
                            "Tried to attach thread to core: " << cpu_core << " even though we only have: " << number_of_cpus  << " core!";
          return false;
      }
      //Set the core
      CPU_SET(cpu_core, &cpuset);
      //Tell the scheduler our preferences
      errorFlag = pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset);
      if(errorFlag != 0)
      {
          std::cerr <<"[ur_ros2_control_node::setRealtimePriority]" <<
                          "Could not assign control thread to single cpu core: "
                            << errorFlag;
          success &= false;
      }

  }
  return success;
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // create executor
  std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // create controller manager instance
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");

  // control loop thread
  std::thread control_loop([controller_manager]() {
    //Increase the scheduler priority of the current thread to max and tie the thread to core 1
    //Note core 0 is usually also used in the kernel for kernal internal tasks. Therefore it makes sense to use another one!
    setRealtimePriority(99,1);
    // use fixed time step
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

    while (rclcpp::ok()) {
      // ur client library is blocking and is the one that is controlling time step
      controller_manager->read(controller_manager->now(), dt);
      controller_manager->update(controller_manager->now(), dt);
      controller_manager->write(controller_manager->now(), dt);
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
