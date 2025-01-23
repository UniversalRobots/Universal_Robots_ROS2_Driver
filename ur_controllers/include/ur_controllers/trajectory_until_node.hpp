// Copyright 2025, Universal Robots A/S
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
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2025-01-17
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__TRAJECTORY_UNTIL_NODE_HPP_
#define UR_CONTROLLERS__TRAJECTORY_UNTIL_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/duration.hpp>
#include <ur_msgs/action/tool_contact.hpp>
#include <ur_msgs/action/trajectory_until.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ur_controllers
{

// template <typename action_type>
class TrajectoryUntilNode : public rclcpp::Node
{
private:
  rclcpp_action::Server<ur_msgs::action::TrajectoryUntil>::SharedPtr action_server_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp_action::Client<ur_msgs::action::ToolContact>::SharedPtr until_action_client_;

  rclcpp::CallbackGroup::SharedPtr server_callback_group;
  rclcpp::CallbackGroup::SharedPtr clients_callback_group;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::TrajectoryUntil>> server_goal_handle_;

  void
  until_response_callback(const rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr& goal_handle);
  void
  until_result_callback(const rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::WrappedResult& result);

  void trajectory_response_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle);
  void trajectory_feedback_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);
  void trajectory_result_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result);

  void assign_until_action_client(std::shared_ptr<const ur_msgs::action::TrajectoryUntil::Goal> goal);

  rclcpp_action::GoalResponse goal_received_callback(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ur_msgs::action::TrajectoryUntil::Goal> goal);
  void goal_accepted_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::TrajectoryUntil>> goal_handle);
  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::TrajectoryUntil>> goal_handle);

  void send_trajectory_goal(std::shared_ptr<const ur_msgs::action::TrajectoryUntil::Goal> goal);

  void send_until_goal(std::shared_ptr<const ur_msgs::action::TrajectoryUntil::Goal> goal);

  void report_goal(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult result);

  template <typename UntilResult>
  void report_goal(UntilResult result);

  void reset_node();

  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
      current_trajectory_goal_handle_;
  rclcpp_action::ClientGoalHandle<ur_msgs::action::ToolContact>::SharedPtr current_until_goal_handle_;

  std::shared_ptr<ur_msgs::action::TrajectoryUntil::Result> prealloc_res;

  std::shared_ptr<ur_msgs::action::TrajectoryUntil::Feedback> prealloc_fb;

  std::atomic<bool> trajectory_accepted_;
  std::atomic<bool> until_accepted_;

  std::condition_variable cv_until_;
  std::condition_variable cv_trajectory_;
  std::mutex mutex_until;
  std::mutex mutex_trajectory;

public:
  explicit TrajectoryUntilNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TrajectoryUntilNode();
};

}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__TRAJECTORY_UNTIL_NODE_HPP_
