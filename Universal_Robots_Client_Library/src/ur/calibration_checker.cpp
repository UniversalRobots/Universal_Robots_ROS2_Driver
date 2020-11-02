// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-14
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/ur/calibration_checker.h>

namespace urcl
{
CalibrationChecker::CalibrationChecker(const std::string& expected_hash)
  : expected_hash_(expected_hash), checked_(false)
{
}
bool CalibrationChecker::consume(std::shared_ptr<primary_interface::PrimaryPackage> product)
{
  auto kin_info = std::dynamic_pointer_cast<primary_interface::KinematicsInfo>(product);
  if (kin_info != nullptr)
  {
    // LOG_INFO("%s", product->toString().c_str());
    //
    if (kin_info->toHash() != expected_hash_)
    {
      LOG_ERROR("The calibration parameters of the connected robot don't match the ones from the given kinematics "
                "config file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use the "
                "ur_calibration tool to extract the correct calibration from the robot and pass that into the "
                "description. See "
                "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] for "
                "details.");
    }
    else
    {
      LOG_INFO("Calibration checked successfully.");
    }

    checked_ = true;
  }

  return true;
}
}  // namespace urcl
