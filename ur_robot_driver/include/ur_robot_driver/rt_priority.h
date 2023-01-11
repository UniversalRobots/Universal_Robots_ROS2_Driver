#pragma once

#include <thread>
#include <pthread.h>
#include <unistd.h>

/**
 * @brief set the priority of the current thread to the passed priority and tie the thread to a certain core
 * @param priority - the desired priority between [0,99] with 99 beeing the highest priority
 * @param cpu_core - the number of the core you want to tie the thread to (0,your amount of cores]. You are not allowed to tie the thread to core 0 as core 0 is often used for kernel tasks!
 * 
 * @note In order for this method to work without root permissions you need to add the following lines to /etc/security/limits.conf (without the ticks "")
 *    "* soft rtprio 99"
 *    "* hard rtprio 99"
 * You need to logout and log back into your session in order for these changes to be applied
*/
static bool setRealtimePriority(int priority, int cpu_core){
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
