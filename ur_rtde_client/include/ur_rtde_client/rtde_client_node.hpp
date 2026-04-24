// Copyright 2026, Universal Robots A/S
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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ur_rtde_client/rtde_manager.hpp"
#include "ur_rtde_client/publisher.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ur_rtde_client/publisher.hpp"

namespace ur_rtde_client
{
/**
 * @brief ROS 2 node that bridges UR RTDE data to ROS topics.
 */
class RtdeClientNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the node, declares parameters, loads config, creates publishers,
   *        and starts RTDE communication (which runs in a background thread).
   */
  RtdeClientNode();

private:
  /// @brief Helper that loads YAML mapping and creates ROS publishers per RTDE variable/group.
  std::unique_ptr<Publisher> publisher_;

  /// @brief Manager that owns the URCL RTDEClient, initializes and streams data.
  std::unique_ptr<RtdeManager> rtde_manager_;

  /// @brief Recipe (list of RTDE output keys) requested to the robot and published by this node.
  std::vector<std::string> output_recipe_;

  /// @brief Robot controller IP address (declared/read as parameter).
  std::string robot_ip_;

  /// @brief Keys actually created and active after validating with the YAML mapping.
  std::vector<std::string> effective_keys_;
};

}  // namespace ur_rtde_client