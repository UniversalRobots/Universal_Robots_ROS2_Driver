// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Created on behalf of Universal Robots A/S
// Copyright 2019 FZI Forschungszentrum Informatik
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
//    * Neither the name of the {copyright_holder} nor the names of its
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
