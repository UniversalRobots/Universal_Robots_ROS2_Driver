// Copyright 2019, FZI Forschungszentrum Informatik
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <optional>
#include <memory>
#include <vector>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "ur_controllers/scaled_joint_trajectory_controller_parameters.hpp"

namespace ur_controllers
{
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

private:
  std::atomic<double> scaling_factor_{ 1.0 };

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> scaling_state_interface_ =
      std::nullopt;

  std::shared_ptr<scaled_joint_trajectory_controller::ParamListener> scaled_param_listener_;
  scaled_joint_trajectory_controller::Params scaled_params_;

  // Private methods copied from Upstream JTC
  void update_pids();

  /**
   * @brief Assigns the values from a trajectory point interface to a joint interface.
   *
   * @tparam T The type of the joint interface.
   * @param[out] joint_interface The reference_wrapper to assign the values to
   * @param[in] trajectory_point_interface Containing the values to assign.
   * @todo: Use auto in parameter declaration with c++20
   */
  template <typename T>
  bool assign_interface_from_point(const T& joint_interface, const std::vector<double>& trajectory_point_interface)
  {
    bool success = true;
    for (size_t index = 0; index < num_cmd_joints_; ++index) {
      success &= joint_interface[index].get().set_value(trajectory_point_interface[map_cmd_to_joints_[index]]);
    }
    return success;
  }
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
