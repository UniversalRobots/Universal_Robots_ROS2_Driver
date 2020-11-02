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
 * \date    2020-07-09
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/rtde/rtde_parser.h>
#include <ur_client_library/rtde/request_protocol_version.h>

using namespace urcl;

TEST(rtde_parser, request_protocol_version)
{
  // Accepted request protocol version
  unsigned char raw_data[] = { 0x00, 0x04, 0x56, 0x01 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::RequestProtocolVersion* data =
          dynamic_cast<rtde_interface::RequestProtocolVersion*>(products[0].get()))
  {
    EXPECT_EQ(data->accepted_, true);
  }
}

TEST(rtde_parser, get_urcontrol_version)
{
  // URControl version 5.8.0-0
  unsigned char raw_data[] = { 0x00, 0x13, 0x76, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                               0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::GetUrcontrolVersion* data = dynamic_cast<rtde_interface::GetUrcontrolVersion*>(products[0].get()))
  {
    EXPECT_EQ(data->version_information_.major, 5);
    EXPECT_EQ(data->version_information_.minor, 8);
    EXPECT_EQ(data->version_information_.bugfix, 0);
    EXPECT_EQ(data->version_information_.build, 0);
  }
}

TEST(rtde_parser, test_to_string)
{
  // Non-existent type
  unsigned char raw_data[] = { 0x00, 0x05, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  std::stringstream expected;
  expected << "Type: 2" << std::endl;
  expected << "Raw byte stream: 0 0 " << std::endl;

  EXPECT_EQ(products[0]->toString(), expected.str());
}

TEST(rtde_parser, test_illegal_length)
{
  // Non-existent type with false size information
  unsigned char raw_data[] = { 0x00, 0x06, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  EXPECT_FALSE(parser.parse(bp, products));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
