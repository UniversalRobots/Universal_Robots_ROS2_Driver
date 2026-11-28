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

#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ur_moveit_config
{
class PreserveContinuousJointTurns : public planning_interface::PlanningResponseAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    const auto parameter = parameter_namespace + ".preserve_continuous_joint_turns";
    enabled_ = node->has_parameter(parameter) ? node->get_parameter(parameter).as_bool() :
                                                node->declare_parameter<bool>(parameter, false);
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return "PreserveContinuousJointTurns";
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest&,
             planning_interface::MotionPlanResponse& response) const override
  {
    if (enabled_ && scene && response.trajectory) {
      preserveContinuousJointTurns(scene->getCurrentState(), *response.trajectory);
    }
  }

private:
  bool enabled_{ false };
};
}  // namespace ur_moveit_config

PLUGINLIB_EXPORT_CLASS(ur_moveit_config::PreserveContinuousJointTurns, planning_interface::PlanningResponseAdapter)
