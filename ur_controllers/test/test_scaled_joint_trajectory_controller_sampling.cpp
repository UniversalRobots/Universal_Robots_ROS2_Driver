// Copyright 2026, Universal Robots A/S
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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "ur_controllers/scaled_joint_trajectory_controller.hpp"

namespace ur_controllers_test
{
constexpr char kScalingJoint[] = "speed_scaling";
constexpr char kScalingIface[] = "speed_scaling_factor";
constexpr char kJoint[] = "joint1";

class TestableScaledJointTrajectoryController : public ur_controllers::ScaledJointTrajectoryController
{
public:
  using ur_controllers::ScaledJointTrajectoryController::ScaledJointTrajectoryController;

  trajectory_msgs::msg::JointTrajectoryPoint get_state_desired() const
  {
    return state_desired_;
  }

  void wait_for_trajectory(rclcpp::Executor& executor,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(50))
  {
    const auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }
};

struct HwBuffers
{
  double command_position{ 0.0 };
  double state_position{ 0.0 };
  double state_velocity{ 0.0 };
  double scaling{ 1.0 };
  // Loaned{Command,State}Interface reference these; they must outlive assign_interfaces().
  std::vector<hardware_interface::CommandInterface> command_storage_;
  std::vector<hardware_interface::StateInterface> state_storage_;
};

class ScaledJointTrajectorySamplingTest : public ::testing::Test
{
protected:
  std::shared_ptr<TestableScaledJointTrajectoryController>
  create_controller(const std::string& controller_name, const std::vector<rclcpp::Parameter>& extra_params,
                    const std::string& scaling_iface_name)
  {
    auto controller = std::make_shared<TestableScaledJointTrajectoryController>();

    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("joints", std::vector<std::string>{ kJoint }),
      rclcpp::Parameter("command_interfaces", std::vector<std::string>{ "position" }),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{ "position", "velocity" }),
      rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", true),
      rclcpp::Parameter("speed_scaling_interface_name", scaling_iface_name),
      rclcpp::Parameter("constraints.joint1.trajectory", 10.0),
      rclcpp::Parameter("constraints.joint1.goal", 0.1),
      rclcpp::Parameter("constraints.stopped_velocity_tolerance", 1.0),
      rclcpp::Parameter("constraints.goal_time", 0.0),
    };
    overrides.insert(overrides.end(), extra_params.begin(), extra_params.end());

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(overrides);

    const auto init_ret = controller->init(controller_name, "", node_options);
    if (init_ret != controller_interface::return_type::OK) {
      return nullptr;
    }
    return controller;
  }

  void assign_interfaces(TestableScaledJointTrajectoryController& ctrl, HwBuffers& hw, bool with_scaling_state)
  {
    hw.command_storage_.clear();
    hw.state_storage_.clear();

    hw.command_storage_.emplace_back(kJoint, hardware_interface::HW_IF_POSITION, &hw.command_position);
    std::vector<hardware_interface::LoanedCommandInterface> cmd_ifs;
    cmd_ifs.emplace_back(hw.command_storage_.back());

    hw.state_storage_.emplace_back(kJoint, hardware_interface::HW_IF_POSITION, &hw.state_position);
    hw.state_storage_.emplace_back(kJoint, hardware_interface::HW_IF_VELOCITY, &hw.state_velocity);
    if (with_scaling_state) {
      hw.state_storage_.emplace_back(kScalingJoint, kScalingIface, &hw.scaling);
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_ifs.reserve(hw.state_storage_.size());
    for (auto& si : hw.state_storage_) {
      state_ifs.emplace_back(si);
    }

    ctrl.assign_interfaces(std::move(cmd_ifs), std::move(state_ifs));
  }

  trajectory_msgs::msg::JointTrajectory make_linear_trajectory_0_to_1_in_1s()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = { kJoint };
    traj.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    traj.points.resize(2);
    traj.points[0].positions = { 0.0 };
    traj.points[0].time_from_start = rclcpp::Duration(0, 0);
    traj.points[1].positions = { 1.0 };
    traj.points[1].time_from_start = rclcpp::Duration(1, 0);
    return traj;
  }

  double run_until(const std::string& controller_name, const std::string& scaling_iface_name, HwBuffers& hw,
                   bool with_scaling_state, int cycles, const rclcpp::Duration& period)
  {
    rclcpp::executors::MultiThreadedExecutor executor;

    auto ctrl = create_controller(controller_name, {}, scaling_iface_name);
    if (!ctrl) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    auto node = std::make_shared<rclcpp::Node>(controller_name + std::string("_traj_pub"));
    auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_name + "/joint_trajectory",
                                                                             rclcpp::SystemDefaultsQoS());

    executor.add_node(ctrl->get_node()->get_node_base_interface());
    executor.add_node(node->get_node_base_interface());

    if (ctrl->get_node()->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      executor.cancel();
      return std::numeric_limits<double>::quiet_NaN();
    }

    hw.command_position = 0.0;
    hw.state_position = 0.0;
    hw.state_velocity = 0.0;
    assign_interfaces(*ctrl, hw, with_scaling_state);

    if (ctrl->get_node()->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      executor.cancel();
      return std::numeric_limits<double>::quiet_NaN();
    }

    pub->publish(make_linear_trajectory_0_to_1_in_1s());
    ctrl->wait_for_trajectory(executor);

    rclcpp::Time t(0, 0, RCL_STEADY_TIME);
    for (int i = 0; i < cycles; ++i) {
      ctrl->update(t, period);
      t = t + period;
    }

    const double desired = ctrl->get_state_desired().positions[0];
    executor.cancel();
    return desired;
  }
};

/**
 * With identical trajectories and update schedules, halving the speed-scaling state halves how far
 * along the trajectory the controller samples (ratio of desired positions ≈ ratio of scaling).
 */
TEST_F(ScaledJointTrajectorySamplingTest, half_speed_scaling_samples_half_as_far)
{
  const std::string scaling_iface = std::string(kScalingJoint) + "/" + kScalingIface;
  HwBuffers hw_slow{};
  hw_slow.scaling = 0.5;
  HwBuffers hw_fast{};
  hw_fast.scaling = 1.0;

  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
  const int cycles = 100;

  const double p_slow = run_until("sjtc_half", scaling_iface, hw_slow, true, cycles, period);
  const double p_fast = run_until("sjtc_full", scaling_iface, hw_fast, true, cycles, period);

  ASSERT_FALSE(std::isnan(p_slow));
  ASSERT_FALSE(std::isnan(p_fast));
  ASSERT_GT(p_fast, 1e-6);

  const double ratio = p_slow / p_fast;
  EXPECT_NEAR(ratio, 0.5, 0.03);
}

/** No speed-scaling state interface leaves scaling at 1.0; same sampling as explicit 1.0 factor. */
TEST_F(ScaledJointTrajectorySamplingTest, no_scaling_interface_matches_explicit_unit_scaling)
{
  const std::string scaling_iface = std::string(kScalingJoint) + "/" + kScalingIface;
  HwBuffers hw_none{};
  HwBuffers hw_unit{};
  hw_unit.scaling = 1.0;

  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
  const int cycles = 80;

  const double p_no_iface = run_until("sjtc_no_iface", "", hw_none, false, cycles, period);
  const double p_unit_iface = run_until("sjtc_unit_iface", scaling_iface, hw_unit, true, cycles, period);

  ASSERT_FALSE(std::isnan(p_no_iface));
  ASSERT_FALSE(std::isnan(p_unit_iface));
  EXPECT_NEAR(p_no_iface, p_unit_iface, 1e-4);
}

}  // namespace ur_controllers_test

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
