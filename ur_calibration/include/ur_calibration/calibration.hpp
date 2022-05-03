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

#ifndef UR_CALIBRATION__CALIBRATION_HPP_
#define UR_CALIBRATION__CALIBRATION_HPP_

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace ur_calibration
{
/*!
 * \brief An internal representation of a DH-parametrized link.
 *
 * Each segment consists of parameters a, d, alpha and theta.
 */
struct DHSegment
{
  double d_;
  double a_;
  double theta_;
  double alpha_;

  /*!
   * \brief Creates an element with defined elements.
   */
  DHSegment(const double d, const double a, const double theta, const double alpha)
    : d_(d), a_(a), theta_(theta), alpha_(alpha)
  {
  }

  /*!
   * \brief Creates a Segment with all elements equal to zero.
   */
  DHSegment() : d_(0), a_(0), theta_(0), alpha_(0)
  {
  }

  /*!
   * \brief Adds another segment element-wise (a+a, d+d, alpha+alpha, theta+theta)
   *
   * \param other Other segment to add
   *
   * \returns Segment consisting of element-wise addition of \p this and \p other
   */
  DHSegment operator+(const DHSegment& other)
  {
    return DHSegment(this->d_ + other.d_, this->a_ + other.a_, this->theta_ + other.theta_,
                     this->alpha_ + other.alpha_);
  }
};

/*!
 * \brief Internal representation of a robot based on DH parameters.
 *
 * Note that this representation doesn't contain a real DH parameter representation, but is used for
 * a corrected model of calibrated UR robots. Shoulder and elbow axes are artificially shortened in
 * the final representation, requiring a correction parameter in \p theta_2 and \p theta_3.
 */
struct DHRobot
{
  std::vector<DHSegment> segments_;
  double delta_theta_correction2_;
  double delta_theta_correction3_;

  /*!
   * \brief Create a new robot representation giving a set of \ref DHSegment objects
   */
  explicit DHRobot(const std::vector<DHSegment>& segments)
  {
    delta_theta_correction2_ = 0;
    delta_theta_correction3_ = 0;
  }

  DHRobot() = default;

  /*!
   * \brief Adds another robot representation, by adding their segments element-wise. See \ref
   * DHSegment::operator+ for details.
   */
  DHRobot operator+(const DHRobot& other)
  {
    assert(this->segments_.size() == other.segments_.size());
    DHRobot ret;
    for (size_t i = 0; i < this->segments_.size(); ++i) {
      ret.segments_.push_back(this->segments_[i] + other.segments_[i]);
    }
    return ret;
  }
};

/*!
 * \brief Class that handles the calibration correction for Universal Robots
 *
 * Universal robots provide a factory calibration of their DH parameters to exactly estimate their
 * TCP pose using forward kinematics. However, those DH parameters construct a kinematic chain that
 * can be very long, as upper arm and lower arm segments can be drawn out of their physical position
 * by multiple meters (even above 100m can occur).
 *
 * This class helps creating a kinematic chain, that is as close as possible to the physical model,
 * by dragging the upper and lower arm segments back to their zero position.
 */
class Calibration
{
public:
  explicit Calibration(const DHRobot& robot);
  virtual ~Calibration();

  /*!
   * \brief Corrects a UR kinematic chain in such a way that shoulder and elbow offsets are 0.
   */
  void correctChain();

  /*!
   * \brief Get the transformation matrix representation of the chain as constructed by the
   * DH parameters.
   *
   * This will contain twice as many transformation matrices as joints, as for each set of DH
   * parameters two matrices are generated. If you'd like to receive one matrix per joint instead,
   * use the getSimplified() function instead.
   *
   * \returns A vector of 4x4 transformation matrices, two for each joint going from the base to the
   * tcp.
   */
  std::vector<Eigen::Matrix4d> getChain()
  {
    return chain_;
  }

  /*!
   * \brief Get the transformation matrix representation of the chain, where each joint is
   * represented by one matrix.
   *
   * \returns Vector of 4x4 transformation matrices, one for each joint going from the base to the
   * tcp.
   */
  std::vector<Eigen::Matrix4d> getSimplified() const;

  /*!
   * \brief Generates a yaml representation of all transformation matrices as returned by
   * getSimplified()
   *
   * \returns A YAML tree representing all transformation matrices.
   */
  YAML::Node toYaml() const;

  /*!
   * \brief Calculates the forwart kinematics given a joint configuration with respect to the base
   * link.
   *
   * \param joint_values Joint values for which the forward kinematics should be calculated.
   * \param link_nr If given, the cartesian position for this joint (starting at 1) is returned. By
   * default the 6th joint is used.
   *
   * \returns Transformation matrix giving the full pose of the requested link in base coordinates.
   */
  Eigen::Matrix4d calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values, const size_t link_nr = 6);

private:
  // Corrects a single axis
  void correctAxis(const size_t correction_index);

  // Builds the chain from robot_parameters_
  void buildChain();

  DHRobot robot_parameters_;
  std::vector<std::string> link_names_ = { "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3" };

  std::vector<Eigen::Matrix4d> chain_;
};
}  // namespace ur_calibration
#endif  // UR_CALIBRATION__CALIBRATION_HPP_
