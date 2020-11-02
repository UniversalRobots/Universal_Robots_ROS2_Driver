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
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-06
 *
 * Make sure to run this program from its source directory in order to find the respective files.
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <iostream>
#include <memory>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/scriptfile.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

std::unique_ptr<UrDriver> g_my_driver;
vector6d_t g_joint_positions;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

int main(int argc, char* argv[])
{
  std::unique_ptr<ToolCommSetup> tool_comm_setup;
  g_my_driver.reset(new UrDriver(ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, false,
                                 std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_driver->startRTDECommunication();
  double increment = 0.01;
  while (true)
  {
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
    if (data_pkg)
    {
      // Read current joint positions from robot data
      if (!data_pkg->getData("actual_q", g_joint_positions))
      {
        // This throwing should never happen unless misconfigured
        std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
        throw std::runtime_error(error_msg);
      }

      // Simple motion command of last joint
      if (g_joint_positions[5] > 3)
      {
        increment = -3;  // this large jump will activate speed scaling
      }
      else if (g_joint_positions[5] < -3)
      {
        increment = 0.02;
      }
      g_joint_positions[5] += increment;
      g_my_driver->writeJointCommand(g_joint_positions, comm::ControlMode::MODE_SERVOJ);
      std::cout << data_pkg->toString() << std::endl;
    }
    else
    {
      std::cout << "Could not get fresh data package from robot" << std::endl;
    }
  }
  return 0;
}
