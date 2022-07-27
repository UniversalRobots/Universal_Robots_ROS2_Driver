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
 * \date    2019-01-10
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-09-22
 *
 */
//----------------------------------------------------------------------

#include "ur_calibration/calibration_consumer.hpp"

#include <filesystem>
#include <memory>
#include <string>

#include "ur_client_library/comm/parser.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/producer.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/primary/package_header.h"
#include "ur_client_library/primary/primary_parser.h"

#include "ur_robot_driver/urcl_log_handler.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;

using urcl::UrException;
using urcl::comm::INotifier;
using urcl::comm::Pipeline;
using urcl::comm::URProducer;
using urcl::comm::URStream;
using urcl::primary_interface::PrimaryPackage;
using urcl::primary_interface::PrimaryParser;
using urcl::primary_interface::UR_PRIMARY_PORT;

using ur_calibration::CalibrationConsumer;

class CalibrationCorrection : public rclcpp::Node
{
public:
  CalibrationCorrection()
    : Node(
          "ur_calibration",
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    std::string output_package_name;

    try {
      // The robot's IP address
      robot_ip_ = this->get_parameter("robot_ip").as_string();
      // The target file where the calibration data is written to
      output_filename_ = this->get_parameter("output_filename").as_string();
    } catch (rclcpp::exceptions::ParameterNotDeclaredException& e) {
      RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
      exit(1);
    }
  }

  virtual ~CalibrationCorrection() = default;

  void run()
  {
    URStream<PrimaryPackage> stream(robot_ip_, UR_PRIMARY_PORT);
    PrimaryParser parser;
    URProducer<PrimaryPackage> prod(stream, parser);
    CalibrationConsumer consumer;

    INotifier notifier;

    Pipeline<PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
    pipeline.run();
    while (!consumer.isCalibrated()) {
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.1).to_chrono<std::chrono::nanoseconds>());
    }
    calibration_data_.reset(new YAML::Node);
    *calibration_data_ = consumer.getCalibrationParameters();
  }

  bool writeCalibrationData()
  {
    if (calibration_data_ == nullptr) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Calibration data not yet set.");
      return false;
    }

    fs::path out_path = fs::absolute(output_filename_);

    fs::path dst_path = out_path.parent_path();
    if (!fs::exists(dst_path)) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Parent folder " << dst_path << " does not exist.");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Writing calibration data to " << out_path);
    if (fs::exists(output_filename_)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Output file " << output_filename_ << " already exists. Overwriting.");
    }
    std::ofstream file(output_filename_);
    if (file.is_open()) {
      file << *calibration_data_;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed writing the file. Do you have the correct rights?");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Wrote output.");

    return true;
  }

private:
  std::string robot_ip_;
  std::string output_filename_;
  std::shared_ptr<YAML::Node> calibration_data_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  ur_robot_driver::registerUrclLogHandler();

  try {
    auto calib_node = std::make_shared<CalibrationCorrection>();
    calib_node->run();
    if (!calib_node->writeCalibrationData()) {
      RCLCPP_ERROR_STREAM(calib_node->get_logger(), "Failed writing calibration data. See errors above for details.");
      return -1;
    }
    RCLCPP_INFO_STREAM(calib_node->get_logger(), "Calibration correction done");
  } catch (const UrException& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ur_calibration"), e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ur_calibration"), e.what());
  }

  rclcpp::shutdown();

  return 0;
}
