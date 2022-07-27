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

#include "ur_calibration/calibration_consumer.hpp"

#include <memory>

namespace ur_calibration
{
CalibrationConsumer::CalibrationConsumer() : calibrated_(false)
{
}

bool CalibrationConsumer::consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product)
{
  auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
  if (kin_info != nullptr) {
    RCLCPP_INFO(rclcpp::get_logger("ur_calibration"), "%s", product->toString().c_str());
    DHRobot my_robot;
    for (size_t i = 0; i < kin_info->dh_a_.size(); ++i) {
      my_robot.segments_.push_back(
          DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
    }
    Calibration calibration(my_robot);
    calibration.correctChain();

    calibration_parameters_ = calibration.toYaml();
    calibration_parameters_["kinematics"]["hash"] = kin_info->toHash();
    calibrated_ = true;
  }
  return true;
}

YAML::Node CalibrationConsumer::getCalibrationParameters() const
{
  if (!calibrated_) {
    throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
  }
  return calibration_parameters_;
}
}  // namespace ur_calibration
