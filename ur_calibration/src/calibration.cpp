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

#include "ur_calibration/calibration.hpp"

#include <vector>

namespace ur_calibration
{
Calibration::Calibration(const DHRobot& robot_parameters) : robot_parameters_(robot_parameters)
{
  buildChain();
}

Calibration::~Calibration()
{
}

void Calibration::correctChain()
{
  correctAxis(1);
  correctAxis(2);
}

void Calibration::correctAxis(const size_t link_index)
{
  // Each DH-Segment is split into two chain segments. One representing the d and theta parameters and
  // one with the a and alpha parameters. If we start from the first segment (which represents d and
  // theta), there follows a passive segment (with a and alpha) and the next d/theta-segment after
  // that.
  //
  // In principle, the d parameter of the first segment gets set to zero, first. With this change,
  // the kinematic structure gets destroyed, which has to be corrected:
  //   - With setting d to 0, both the start and end points of the passive segment move along the
  //   rotational axis of the start segment. Instead, the end point of the passive segment has to
  //   move along the rotational axis of the next segment. This creates a change in a and theta, if
  //   the two rotational axes are not parallel.
  //
  //   - The length of moving along the next segment's rotational axis is calculated by intersecting
  //   the rotational axis with the XY-plane of the first segment.
  //
  auto& d_theta_segment = chain_[2 * link_index];
  auto& a_alpha_segment = chain_[2 * link_index + 1];

  auto& d = d_theta_segment(2, 3);
  auto& a = a_alpha_segment(0, 3);

  if (d == 0.0) {
    // Nothing to do here.
    return;
  }

  // Start of the next joint's d_theta segment relative to the joint before the current one
  Eigen::Matrix4d next_joint_root = Eigen::Matrix4d::Identity();
  next_joint_root *= d_theta_segment * a_alpha_segment;

  Eigen::Vector3d next_root_position = next_joint_root.topRightCorner(3, 1);

  const auto& next_d_theta_segment = chain_[(link_index + 1) * 2];
  Eigen::Vector3d next_d_theta_end = (next_joint_root * next_d_theta_segment).topRightCorner(3, 1);

  // Construct a representation of the next segment's rotational axis
  Eigen::ParametrizedLine<double, 3> next_rotation_axis;
  next_rotation_axis = Eigen::ParametrizedLine<double, 3>::Through(next_root_position, next_d_theta_end);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_calibration"), "next rotation axis:" << std::endl
                                                                                  << "Base:" << std::endl
                                                                                  << next_rotation_axis.origin()
                                                                                  << std::endl
                                                                                  << "Direction:" << std::endl
                                                                                  << next_rotation_axis.direction());

  // XY-Plane of first segment's start
  Eigen::Hyperplane<double, 3> plane(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d::Zero());

  // Intersect the rotation axis of the next joint with the XY-Plane.
  // * The intersection_param is the length moving along the rotation axis until intersecting the plane.
  // * The intersection point will be used for calculating the new angle theta.
  double intersection_param = next_rotation_axis.intersectionParameter(plane);
  Eigen::Vector3d intersection_point = next_rotation_axis.intersectionPoint(plane);

  // A non-zero a parameter will result in an intersection point at (a, 0) even without any
  // additional rotations. This effect has to be subtracted from the resulting theta value.
  double subtraction_angle = 0.0;
  if (std::abs(a) > 0) {
    // This is pi
    subtraction_angle = atan(1) * 4;
  }
  double new_theta = std::atan2(intersection_point.y(), intersection_point.x()) - subtraction_angle;
  // Upper and lower arm segments on URs all have negative length due to dh params
  double new_link_length = -1 * intersection_point.norm();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_calibration"), "Next joint's rotation axis intersecting at "
                                                                << std::endl
                                                                << intersection_point);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_calibration"), "Angle is " << new_theta);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_calibration"), "Length is " << new_link_length);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_calibration"), "Intersection param is " << intersection_param);

  // as we move the passive segment towards the first segment, we have to move away the next segment
  // again, to keep the same kinematic structure.
  double sign_dir = next_rotation_axis.direction().z() > 0 ? 1.0 : -1.0;
  double distance_correction = intersection_param * sign_dir;

  // Set d parameter of the first segment to 0 and theta to the calculated new angle
  // Correct arm segment length and angle
  d = 0.0;
  d_theta_segment.topLeftCorner(3, 3) = Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Correct arm segment length and angle
  a = new_link_length;
  a_alpha_segment.topLeftCorner(3, 3) =
      Eigen::AngleAxisd(robot_parameters_.segments_[link_index].theta_ - new_theta, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(robot_parameters_.segments_[link_index].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();

  // Correct next joint's d parameter
  chain_[2 * link_index + 2](2, 3) -= distance_correction;
}

Eigen::Matrix4d Calibration::calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values,
                                                   const size_t link_nr)
{
  // Currently ignore input and calculate for zero vector input
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();

  std::vector<Eigen::Matrix4d> simplified_chain = getSimplified();
  for (size_t i = 0; i < link_nr; ++i) {
    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation.topLeftCorner(3, 3) = Eigen::AngleAxisd(joint_values(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    output *= simplified_chain[i] * rotation;
  }

  return output;
}

void Calibration::buildChain()
{
  chain_.clear();
  for (size_t i = 0; i < robot_parameters_.segments_.size(); ++i) {
    Eigen::Matrix4d seg1_mat = Eigen::Matrix4d::Identity();
    seg1_mat.topLeftCorner(3, 3) =
        Eigen::AngleAxisd(robot_parameters_.segments_[i].theta_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    seg1_mat(2, 3) = robot_parameters_.segments_[i].d_;

    chain_.push_back(seg1_mat);

    Eigen::Matrix4d seg2_mat = Eigen::Matrix4d::Identity();
    seg2_mat.topLeftCorner(3, 3) =
        Eigen::AngleAxisd(robot_parameters_.segments_[i].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();
    seg2_mat(0, 3) = robot_parameters_.segments_[i].a_;

    chain_.push_back(seg2_mat);
  }
}

std::vector<Eigen::Matrix4d> Calibration::getSimplified() const
{
  std::vector<Eigen::Matrix4d> simplified_chain;
  simplified_chain.push_back(chain_[0]);
  for (size_t i = 1; i < chain_.size() - 1; i += 2) {
    simplified_chain.emplace_back(chain_[i] * chain_[i + 1]);
    /*
    Eigen::Matrix3d rot_a = chain_[i].topLeftCorner(3, 3);
    Eigen::Vector3d rpy_a = rot_a.eulerAngles(0, 1, 2);

    Eigen::Matrix3d rot_b = chain_[i + 1].topLeftCorner(3, 3);
    Eigen::Vector3d rpy_b = rot_b.eulerAngles(0, 1, 2);

    Eigen::Matrix3d rot = simplified_chain.back().topLeftCorner(3, 3);
    Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    Eigen::Quaterniond quat(rot);
     */
  }
  simplified_chain.push_back(chain_.back());
  return simplified_chain;
}

YAML::Node Calibration::toYaml() const
{
  YAML::Node node;

  std::vector<Eigen::Matrix4d> chain = getSimplified();

  for (std::size_t i = 0; i < link_names_.size(); ++i) {
    YAML::Node link;
    link["x"] = chain[i](0, 3);
    link["y"] = chain[i](1, 3);
    link["z"] = chain[i](2, 3);
    Eigen::Matrix3d rot = chain[i].topLeftCorner(3, 3);
    Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    link["roll"] = rpy[0];
    link["pitch"] = rpy[1];
    link["yaw"] = rpy[2];
    node["kinematics"][link_names_[i]] = link;
  }

  return node;
}
}  // namespace ur_calibration
