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

#include "ur_rtde_client/rtde_client_node.hpp"

using namespace std::chrono_literals;

namespace ur_rtde_client
{

RtdeClientNode::RtdeClientNode() : rclcpp::Node("rtde_client_node")
{
  // Declare parameters
  output_recipe_ = declare_parameter<std::vector<std::string>>("output_recipe");
  robot_ip_ = declare_parameter<std::string>("robot_ip");

  if (output_recipe_[0] == "none" || robot_ip_ == "none") {
    std::ostringstream oss;

    oss << "\033[1;31m"
        << "\n"
        << "ERROR: Missing required parameter(s): robot_ip and/or output_recipe.\n"
        << "Provide them like this:\n"
        << "  ros2 launch ur_rtde_client rtde_client.launch.xml \\\n"
        << "    robot_ip:=\"172.17.0.3\" \\\n"
        << "    output_recipe:=\"[\"payload\",\"timestamp\",\"safety_status\"]\" \033[0m \n";

    throw std::runtime_error(oss.str());
  }

  // Load publisher config
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("ur_rtde_client");
  std::string config_path = package_share_dir + "/config/rtde_map.yaml";

  // Initialize publisher
  publisher_ = std::make_unique<Publisher>(*this, config_path);

  if (output_recipe_.empty()) {
    RCLCPP_WARN(this->get_logger(), "output_recipe is empty, no publishers will be created");
    return;
  }

  // Create publishers based on output_recipe and config
  publisher_->createPublishersForRecipe(output_recipe_);
  effective_keys_ = publisher_->effective_keys();

  RCLCPP_INFO(this->get_logger(), "Created %zu publishers from recipe", effective_keys_.size());

  try {
    // Initialize RTDE communication
    // The RtdeManager will run data reading in a background thread
    rtde_manager_ = start_communication(*this, robot_ip_, effective_keys_, *publisher_);

    if (!rtde_manager_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize RTDE communication");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "RTDE client node initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
  }
}

}  // namespace ur_rtde_client

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_rtde_client::RtdeClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
