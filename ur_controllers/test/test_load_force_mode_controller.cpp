// Copyright 2024, Universal Robots A/S
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

#include <gmock/gmock.h>

#include <atomic>
#include <thread>
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

TEST(TestLoadForceModeController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm{ executor, ros2_control_test_assets::minimal_robot_urdf, true,
                                            "test_controller_manager" };

  const std::string test_file_path = std::string{ TEST_FILES_DIRECTORY } + "/force_mode_controller_params.yaml";
  cm.set_parameter({ "test_force_mode_controller.params_file", test_file_path });

  cm.set_parameter({ "test_force_mode_controller.type", "ur_controllers/ForceModeController" });

  ASSERT_NE(cm.load_controller("test_force_mode_controller"), nullptr);
}

TEST(TestLoadForceModeController, reconfigure_controller_with_tf_traffic)
{
  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm{ executor, ros2_control_test_assets::minimal_robot_urdf, true,
                                            "test_controller_manager" };

  const std::string test_file_path = std::string{ TEST_FILES_DIRECTORY } + "/force_mode_controller_params.yaml";
  cm.set_parameter({ "test_force_mode_controller.params_file", test_file_path });
  cm.set_parameter({ "test_force_mode_controller.type", "ur_controllers/ForceModeController" });
  ASSERT_NE(cm.load_controller("test_force_mode_controller"), nullptr);

  // Publish TF data during the test. The TF listener of the controller writes it into its buffer.
  auto tf_node = std::make_shared<rclcpp::Node>("test_tf_publisher");
  tf2_ros::TransformBroadcaster broadcaster(tf_node);
  std::atomic<bool> publish{ true };
  std::thread publisher_thread([&]() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base";
    transform.child_frame_id = "tool0";
    transform.transform.rotation.w = 1.0;
    while (publish) {
      transform.header.stamp = tf_node->now();
      broadcaster.sendTransform(transform);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  // Configure an inactive controller again: the controller manager cleans it up and configures it.
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(cm.configure_controller("test_force_mode_controller"), controller_interface::return_type::OK);
  }

  publish = false;
  publisher_thread.join();
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
