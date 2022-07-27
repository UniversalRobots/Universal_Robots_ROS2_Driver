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
 * \date    2019-03-11
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-09-27
 *
 */
//----------------------------------------------------------------------

#include <gmock/gmock.h>
#include <ur_calibration/calibration.hpp>

using ur_calibration::Calibration;
using ur_calibration::DHRobot;
using ur_calibration::DHSegment;

namespace
{
/*
bool isApproximately(const double val1, const double val2, const double precision)
{
  return std::abs(val1 - val2) < precision;
}
*/
template <class Scalar_, int dim_>
void doubleEqVec(const Eigen::Matrix<Scalar_, dim_, 1> vec1, const Eigen::Matrix<Scalar_, dim_, 1> vec2,
                 const double precision)
{
  for (size_t i = 0; i < dim_; ++i) {
    EXPECT_NEAR(vec1[i], vec2[i], precision);
  }
}

}  // namespace

class CalibrationTest : public ::testing::Test
{
public:
  void SetUp()
  {
    Test::SetUp();
  }
  void TearDown()
  {
    Test::TearDown();
  }

protected:
  const double pi = std::atan(1) * 4;
  const double pi_2 = 1.570796327;  // This is what the simulated robot reports as pi/2
  DHRobot my_robot_;
  DHRobot my_robot_calibration_;
};

TEST_F(CalibrationTest, ur10_fw_kinematics_ideal)
{
  my_robot_.segments_.clear();
  // This is an ideal UR10
  // clang-format off
    my_robot_.segments_.emplace_back(DHSegment(0.1273  , 0      , 0    , pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.612 , 0    , 0));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.5723, 0    , 0.0));
    my_robot_.segments_.emplace_back(DHSegment(0.163941, 0      , 0    , pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0.1157  , 0      , 0    , -pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on

  Calibration calibration(my_robot_);

  Eigen::Matrix<double, 6, 1> jointvalues;
  Eigen::Vector3d expected_position;
  Eigen::Quaterniond expected_orientation;
  {
    jointvalues << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_DOUBLE_EQ(fk(0, 3), my_robot_.segments_[1].a_ + my_robot_.segments_[2].a_);
    EXPECT_DOUBLE_EQ(fk(1, 3), -1 * (my_robot_.segments_[3].d_ + my_robot_.segments_[5].d_));
    EXPECT_DOUBLE_EQ(fk(2, 3), my_robot_.segments_[0].d_ - my_robot_.segments_[4].d_);
  }

  {
    jointvalues << M_PI_2, -M_PI_4, M_PI_2, -M_PI_4, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    expected_position << my_robot_.segments_[3].d_ + my_robot_.segments_[5].d_,
        my_robot_.segments_[1].a_ / std::sqrt(2) + my_robot_.segments_[2].a_ / std::sqrt(2),
        my_robot_.segments_[0].d_ - my_robot_.segments_[1].a_ / std::sqrt(2) +
            my_robot_.segments_[2].a_ / std::sqrt(2) - my_robot_.segments_[4].d_;
    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-16);
  }
}

TEST_F(CalibrationTest, ur10_fw_kinematics_real)
{
  // This test compares a corrected ideal model against positions taken from a simulated robot.
  my_robot_.segments_.clear();
  // This is an ideal UR10
  // clang-format off
    my_robot_.segments_.emplace_back(DHSegment(0.1273  , 0      , 0    , pi_2));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.612 , 0    , 0));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.5723, 0    , 0.0));
    my_robot_.segments_.emplace_back(DHSegment(0.163941, 0      , 0    , pi_2));
    my_robot_.segments_.emplace_back(DHSegment(0.1157  , 0      , 0    , -pi_2));
    my_robot_.segments_.emplace_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on

  Calibration calibration(my_robot_);

  Eigen::Matrix<double, 6, 1> jointvalues;
  Eigen::Vector3d expected_position;
  Eigen::Quaterniond expected_orientation;
  {
    jointvalues << -1.6007002035724084976209269370884, -1.7271001974688928726209269370884,
        -2.2029998938189905288709269370884, -0.80799991289247685699592693708837, 1.59510004520416259765625,
        -0.03099996248354131012092693708837;
    expected_position << -0.179925914147547, -0.606869755162764, 0.230789102067257;

    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-15);
  }
  {
    jointvalues << 1.32645022869110107421875, 2.426007747650146484375, 5.951572895050048828125,
        1.27409040927886962890625, -0.54105216661562138824592693708837, 0.122173048555850982666015625;
    expected_position << 0.39922988003280424074148413637886, 0.59688365069340565405298093537567,
        -0.6677819040276375961440180617501;

    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);

    doubleEqVec<double, 3>(expected_position, fk.topRightCorner(3, 1), 1e-15);
  }

  TearDown();
}

TEST_F(CalibrationTest, calibration)
{
  // This test compares the forward kinematics of the model constructed from uncorrected
  // parameters with the one from the corrected parameters. They are tested against random
  // joint values and should be equal (in a numeric sense).

  my_robot_.segments_.clear();

  // This is an ideal UR10
  // clang-format off
  //                                     d,        a,       theta, alpha
    my_robot_.segments_.emplace_back(DHSegment(0.1273  , 0      , 0    , pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.612 , 0    , 0));
    my_robot_.segments_.emplace_back(DHSegment(0       , -0.5723, 0    , 0.0));
    my_robot_.segments_.emplace_back(DHSegment(0.163941, 0      , 0    , pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0.1157  , 0      , 0    , -pi / 2));
    my_robot_.segments_.emplace_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on
  my_robot_calibration_.segments_.clear();
  // clang-format off
  //                                                 d,        a,       theta, alpha
    my_robot_calibration_.segments_.emplace_back(DHSegment( 0.00065609212979853    ,  4.6311376834935676e-05 ,
          -7.290070070824746e-05  ,  0.000211987863869334  ));
    my_robot_calibration_.segments_.emplace_back(DHSegment( 1.4442162376284788     , -0.00012568315331862312 ,
          -0.01713897289704999    , -0.0072553625957652995));
    my_robot_calibration_.segments_.emplace_back(DHSegment( 0.854147723854608      ,  0.00186216581161458    ,
          -0.03707159413492756    , -0.013483226769541364 ));
    my_robot_calibration_.segments_.emplace_back(DHSegment(-2.2989425877563705     ,  9.918593870679266e-05  ,
          0.054279462160583214   ,  0.0013495820227329425 ));
    my_robot_calibration_.segments_.emplace_back(DHSegment(-1.573498686836816e-05  ,  4.215462720453189e-06  ,
          1.488984257025741e-07  , -0.001263136163679901 ));
    my_robot_calibration_.segments_.emplace_back(DHSegment( 1.9072435590711256e-05 ,  0                      ,
          1.551499479707493e-05  ,  0 ));
  // clang-format on

  Eigen::Matrix<double, 6, 1> jointvalues;
  jointvalues << 0, 0, 0, 0, 0, 0;

  for (size_t i = 0; i < 1000; ++i) {
    Calibration calibration(my_robot_ + my_robot_calibration_);
    jointvalues = 2 * pi * Eigen::Matrix<double, 6, 1>::Random();
    Eigen::Matrix4d fk_orig = calibration.calcForwardKinematics(jointvalues);
    Eigen::Matrix3d rot_mat_orig = fk_orig.topLeftCorner(3, 3);
    Eigen::Quaterniond rot_orig(rot_mat_orig);
    calibration.correctChain();
    Eigen::Matrix4d fk_corrected = calibration.calcForwardKinematics(jointvalues);
    Eigen::Matrix3d rot_mat_corrected = fk_corrected.topLeftCorner(3, 3);
    Eigen::Quaterniond rot_corrected(rot_mat_corrected);
    double angle_error = std::abs(rot_orig.angularDistance(rot_corrected));
    EXPECT_NEAR(fk_orig(0, 3), fk_corrected(0, 3), 1e-12);
    EXPECT_NEAR(fk_orig(1, 3), fk_corrected(1, 3), 1e-12);
    EXPECT_NEAR(fk_orig(2, 3), fk_corrected(2, 3), 1e-12);
    EXPECT_NEAR(angle_error, 0.0, 1e-12);
  }
}
