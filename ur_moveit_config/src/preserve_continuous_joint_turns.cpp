// Copyright 2026, Universal Robots ROS 2 Driver contributors
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
//    * Neither the name of the copyright holder nor the names of its
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

#include "preserve_continuous_joint_turns.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include <moveit/robot_model/revolute_joint_model.hpp>

namespace ur_moveit_config
{
void preserveContinuousJointTurns(const moveit::core::RobotState& measured, robot_trajectory::RobotTrajectory& path)
{
  if (path.empty() || !path.getGroup() || path.getRobotModel() != measured.getRobotModel()) {
    return;
  }

  // Only use the live state when the path starts at an equivalent configuration.
  // A plan from a different start configuration must retain its original coordinates.
  constexpr double equivalence_tolerance = 1e-5;
  constexpr double full_turn = 2.0 * std::numbers::pi;
  const auto& first = path.getFirstWayPoint();
  std::vector<std::pair<int, double>> offsets;
  for (const auto* joint : path.getGroup()->getActiveJointModels()) {
    if (joint->getVariableCount() != 1) {
      return;
    }
    const int index = joint->getFirstVariableIndex();
    const double actual = measured.getVariablePosition(index);
    const double planned = first.getVariablePosition(index);
    if (!std::isfinite(actual) || !std::isfinite(planned)) {
      return;
    }
    const bool continuous = joint->getType() == moveit::core::JointModel::REVOLUTE &&
                            static_cast<const moveit::core::RevoluteJointModel*>(joint)->isContinuous();
    const double delta = actual - planned;
    const double offset = continuous ? std::round(delta / full_turn) * full_turn : 0.0;
    if (std::abs(delta - offset) > equivalence_tolerance) {
      return;
    }
    if (offset != 0.0) {
      offsets.emplace_back(index, offset);
    }
  }

  if (offsets.empty()) {
    return;
  }
  for (std::size_t i = 0; i < path.getWayPointCount(); ++i) {
    for (const auto& [index, offset] : offsets) {
      if (!std::isfinite(path.getWayPoint(i).getVariablePosition(index) + offset)) {
        return;
      }
    }
  }
  for (std::size_t i = 0; i < path.getWayPointCount(); ++i) {
    auto point = std::make_shared<moveit::core::RobotState>(path.getWayPoint(i));
    for (const auto& [index, offset] : offsets) {
      point->setVariablePosition(index, point->getVariablePosition(index) + offset);
    }
    path.getWayPointPtr(i) = std::move(point);
  }
}
}  // namespace ur_moveit_config
