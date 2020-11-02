// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-12
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/rtde/rtde_client.h>

#include <iostream>
#include <memory>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string ROBOT_IP = "192.168.56.101";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::chrono::milliseconds READ_TIMEOUT{ 100 };

int main(int argc, char* argv[])
{
  // TODO: Write good docstring for notifier
  comm::INotifier notifier;
  rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE, INPUT_RECIPE);
  my_client.init();

  // We will use the speed_slider_fraction as an example how to write to RTDE
  double speed_slider_fraction = 1.0;
  double speed_slider_increment = 0.01;

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  my_client.start();
  while (true)
  {
    // Read latest RTDE package. This will block for READ_TIMEOUT, so the
    // robot will effectively be in charge of setting the frequency of this loop unless RTDE
    // communication doesn't work in which case the user will be notified.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = my_client.getDataPackage(READ_TIMEOUT);
    if (data_pkg)
    {
      std::cout << data_pkg->toString() << std::endl;
    }
    else
    {
      std::cout << "Could not get fresh data package from robot" << std::endl;
    }

    if (!my_client.getWriter().sendSpeedSlider(speed_slider_fraction))
    {
      // This will happen for example, when the required keys are not configured inside the input
      // recipe.
      std::cout << "\033[1;31mSending RTDE data failed."
                << "\033[0m\n"
                << std::endl;
    }

    // Change the speed slider so that it will move between 0 and 1 all the time. This is for
    // demonstration purposes only and gains no real value.
    if (speed_slider_increment > 0)
    {
      if (speed_slider_fraction + speed_slider_increment > 1.0)
      {
        speed_slider_increment *= -1;
      }
    }
    else if (speed_slider_fraction + speed_slider_increment < 0.0)
    {
      speed_slider_increment *= -1;
    }
    speed_slider_fraction += speed_slider_increment;
  }

  return 0;
}
