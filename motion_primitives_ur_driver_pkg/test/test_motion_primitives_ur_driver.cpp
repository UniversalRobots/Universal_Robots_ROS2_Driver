// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

// #include <gmock/gmock.h>

// #include <string>

// #include "hardware_interface/resource_manager.hpp"
// #include "ros2_control_test_assets/components_urdfs.hpp"
// #include "ros2_control_test_assets/descriptions.hpp"

// class TestMotionPrimitivesUrDriver : public ::testing::Test
// {
// protected:
//   void SetUp() override
//   {
//     // TODO(anyone): Extend this description to your robot
//     motion_primitives_ur_driver_2dof_ =
//       R"(
//         <ros2_control name="MotionPrimitivesUrDriver2dof" type="system">
//           <hardware>
//             <plugin>motion_primitives_ur_driver_pkg/MotionPrimitivesUrDriver</plugin>
//           </hardware>
//           <joint name="joint1">
//             <command_interface name="position"/>
//             <state_interface name="position"/>
//             <param name="initial_position">1.57</param>
//           </joint>
//           <joint name="joint2">
//             <command_interface name="position"/>
//             <state_interface name="position"/>
//             <param name="initial_position">0.7854</param>
//           </joint>
//         </ros2_control>
//     )";
//   }

//   std::string motion_primitives_ur_driver_2dof_;
// };

// TEST_F(TestMotionPrimitivesUrDriver, load_motion_primitives_ur_driver_2dof)
// {
//   auto urdf = ros2_control_test_assets::urdf_head + motion_primitives_ur_driver_2dof_ +
//               ros2_control_test_assets::urdf_tail;
//   ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
// }
