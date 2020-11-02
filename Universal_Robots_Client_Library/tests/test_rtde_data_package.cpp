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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-09-11
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_client_library/rtde/data_package.h>

using namespace urcl;

TEST(rtde_data_package, serialize_pkg)
{
  std::vector<std::string> recipe{ "speed_slider_mask" };
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

  uint32_t value = 1;
  package.setData("speed_slider_mask", value);

  uint8_t buffer[4096];
  package.setRecipeID(1);
  size_t size = package.serializePackage(buffer);

  EXPECT_EQ(size, 8);

  uint8_t expected[] = { 0x0, 0x08, 0x55, 0x01, 0x00, 0x00, 0x00, 0x01 };
  std::cout << "Serialized buffer: " << std::endl;
  for (size_t i = 0; i < size; ++i)
  {
    EXPECT_EQ(buffer[i], expected[i]);
  }
  std::cout << std::endl;
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
