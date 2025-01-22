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

#include <ur_controllers/trajectory_until_node.hpp>
#include <functional>
#include <thread>

namespace ur_controllers
{

using ToolContact = ur_msgs::action::ToolContact;
using TrajectoryUntil = ur_msgs::action::TrajectoryUntil;
using GoalHandleTrajectoryUntil = rclcpp_action::ServerGoalHandle<TrajectoryUntil>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

using namespace std::chrono_literals;

TrajectoryUntilNode::TrajectoryUntilNode(const rclcpp::NodeOptions& options)
  : Node("TrajectoryUntilNode", options)
  , trajectory_accepted_(false)
  , until_accepted_(false)
  , current_trajectory_goal_handle_(nullptr)
  , current_until_goal_handle_(nullptr)
  , server_goal_handle_(nullptr)
  , until_action_client_(nullptr)
{
  trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                                                  "/motion_controller/"
                                                                                  "follow_joint_trajectory",
                                                                                  clients_callback_group);

  action_server_ = rclcpp_action::create_server<TrajectoryUntil>(
      this, "trajectory_until/execute",
      std::bind(&TrajectoryUntilNode::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryUntilNode::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&TrajectoryUntilNode::goal_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), server_callback_group);
}

TrajectoryUntilNode::~TrajectoryUntilNode()
{
}

void TrajectoryUntilNode::assign_until_action_client(std::shared_ptr<const TrajectoryUntil::Goal> goal)
{
  int type = goal->until_type;
  switch (type) {
    case TrajectoryUntil::Goal::TOOL_CONTACT:
      until_action_client_ = rclcpp_action::create_client<ToolContact>(this,
                                                                       "/tool_contact_controller/"
                                                                       "enable_tool_contact",
                                                                       clients_callback_group);
      break;

    default:
      break;
  }
}

rclcpp_action::GoalResponse TrajectoryUntilNode::goal_received_callback(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TrajectoryUntil::Goal> goal)
{
  assign_until_action_client(goal);
  if (!until_action_client_) {
    RCLCPP_ERROR(this->get_logger(), "Until action server not defined, double check the types in the action "
                                     "definition.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!until_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Until action server not available.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory action server not available.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (server_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Node is currently busy, rejecting action goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  send_until_goal(goal);
  std::unique_lock<std::mutex> until_lock(mutex_until);
  cv_until_.wait_for(until_lock, 1s);
  if (!until_accepted_) {
    RCLCPP_ERROR(this->get_logger(), "Until condition not accepted.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  send_trajectory_goal(goal);
  std::unique_lock<std::mutex> traj_lock(mutex_trajectory);
  cv_until_.wait_for(traj_lock, 1s);
  if (!trajectory_accepted_) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory not accepted.");
    until_action_client_->async_cancel_goal(current_until_goal_handle_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TrajectoryUntilNode::goal_accepted_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> goal_handle)
{
  server_goal_handle_ = goal_handle;

  return;
}

rclcpp_action::CancelResponse
TrajectoryUntilNode::goal_cancelled_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> goal_handle)
{
  until_action_client_->async_cancel_goal(current_until_goal_handle_);
  trajectory_action_client_->async_cancel_goal(current_trajectory_goal_handle_);
  server_goal_handle_ = nullptr;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryUntilNode::send_trajectory_goal(std::shared_ptr<const TrajectoryUntil::Goal> goal)
{
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = goal->trajectory;
  goal_msg.goal_time_tolerance = goal->goal_time_tolerance;
  goal_msg.goal_tolerance = goal->goal_tolerance;
  goal_msg.path_tolerance = goal->path_tolerance;

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TrajectoryUntilNode::trajectory_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&TrajectoryUntilNode::trajectory_result_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&TrajectoryUntilNode::trajectory_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  RCLCPP_INFO(this->get_logger(), "Sending trajectory goal");
  trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void TrajectoryUntilNode::send_until_goal(std::shared_ptr<const TrajectoryUntil::Goal> goal)
{
  auto goal_msg = ur_msgs::action::ToolContact::Goal();
  auto send_goal_options = rclcpp_action::Client<ToolContact>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TrajectoryUntilNode::until_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&TrajectoryUntilNode::until_result_callback, this, std::placeholders::_1);
  RCLCPP_INFO(this->get_logger(), "Sending until goal");
  until_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void TrajectoryUntilNode::trajectory_response_callback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle)
{
  {
    std::lock_guard<std::mutex> guard(mutex_trajectory);
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Trajectory accepted.");
      current_trajectory_goal_handle_ = goal_handle;
      trajectory_accepted_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory rejected.");
    }
  }
  cv_trajectory_.notify_one();
}

/* Just forward feedback from trajectory controller */
void TrajectoryUntilNode::trajectory_feedback_callback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr& goal_handle,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  if (server_goal_handle_) {
    prealloc_fb->actual = feedback->actual;
    prealloc_fb->desired = feedback->desired;
    prealloc_fb->error = feedback->error;
    prealloc_fb->header = feedback->header;
    prealloc_fb->joint_names = feedback->joint_names;
    server_goal_handle_->publish_feedback(prealloc_fb);
  }
}

void TrajectoryUntilNode::trajectory_result_callback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
{
  RCLCPP_INFO(this->get_logger(), "Result received.");
  current_trajectory_goal_handle_ = nullptr;
  if (current_until_goal_handle_) {
    until_action_client_->async_cancel_goal(current_until_goal_handle_);
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    report_goal(result, false);
  }
}

void TrajectoryUntilNode::until_response_callback(
    const rclcpp_action::ClientGoalHandle<ToolContact>::SharedPtr& goal_handle)
{
  {
    std::lock_guard<std::mutex> guard(mutex_until);
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Until condition accepted by server, waiting for result");
      current_until_goal_handle_ = goal_handle;
      until_accepted_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Until condition was rejected by server");
    }
  }
  cv_until_.notify_one();
}

void TrajectoryUntilNode::until_result_callback(
    const rclcpp_action::ClientGoalHandle<ToolContact>::WrappedResult& result)
{
  current_until_goal_handle_ = nullptr;
  if (current_trajectory_goal_handle_) {
    trajectory_action_client_->async_cancel_goal(current_trajectory_goal_handle_);
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    report_goal(result, false);
  }
}

void TrajectoryUntilNode::report_goal(auto result, bool until_triggered)
{
  if (until_triggered) {
    prealloc_res->until_condition_result = TrajectoryUntil::Result::TRIGGERED;
  } else {
    prealloc_res->until_condition_result = TrajectoryUntil::Result::NOT_TRIGGERED;
  }
  server_goal_handle_->succeed(prealloc_res);

  reset_vars();
}

void TrajectoryUntilNode::reset_vars()
{
  server_goal_handle_ = nullptr;
  current_trajectory_goal_handle_ = nullptr;
  current_until_goal_handle_ = nullptr;
  trajectory_accepted_ = false;
  until_accepted_ = false;
  prealloc_res = std::make_shared<TrajectoryUntil::Result>(TrajectoryUntil::Result());
  prealloc_fb = std::make_shared<TrajectoryUntil::Feedback>(TrajectoryUntil::Feedback());
  until_action_client_ = nullptr;
}

}  // namespace ur_controllers

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_controllers::TrajectoryUntilNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
