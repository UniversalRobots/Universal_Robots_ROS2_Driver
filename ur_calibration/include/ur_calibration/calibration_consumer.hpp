// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Created on behalf of Universal Robots A/S
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
 * \date    2019-05-28
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-09-22
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CALIBRATION__CALIBRATION_CONSUMER_HPP_
#define UR_CALIBRATION__CALIBRATION_CONSUMER_HPP_

#include <memory>

#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/primary/robot_state/kinematics_info.h"

#include "ur_calibration/calibration.hpp"

namespace ur_calibration
{
class CalibrationConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
{
public:
  CalibrationConsumer();
  virtual ~CalibrationConsumer() = default;

  virtual bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product);

  bool isCalibrated() const
  {
    return calibrated_;
  }

  YAML::Node getCalibrationParameters() const;

private:
  bool calibrated_;
  YAML::Node calibration_parameters_;
};
}  // namespace ur_calibration
#endif  // UR_CALIBRATION__CALIBRATION_CONSUMER_HPP_
