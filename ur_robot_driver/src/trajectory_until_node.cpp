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

#include <ur_robot_driver/trajectory_until_node.hpp>
#include <functional>

namespace ur_robot_driver
{

using TCAction = ur_msgs::action::ToolContact;
using GoalHandleTrajectoryUntil = rclcpp_action::ServerGoalHandle<ur_msgs::action::FollowJointTrajectoryUntil>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using TrajectoryResult = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult;

using namespace std::chrono_literals;

TrajectoryUntilNode::TrajectoryUntilNode(const rclcpp::NodeOptions& options)
  : Node("TrajectoryUntilNode", options)
  , server_goal_handle_(nullptr)
  , current_trajectory_goal_handle_(nullptr)
  , current_until_goal_handle_(nullptr)
  , trajectory_accepted_(false)
  , until_accepted_(false)
{
  prealloc_res_ = std::make_shared<TrajectoryUntilAction::Result>(TrajectoryUntilAction::Result());
  prealloc_fb_ = std::make_shared<TrajectoryUntilAction::Feedback>(TrajectoryUntilAction::Feedback());

  // Different callback groups for the server and the clients, so their callbacks can run concurrently.
  server_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clients_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  declare_parameter("motion_controller_uri", "scaled_joint_trajectory_controller/follow_joint_trajectory");
  get_parameter("motion_controller_uri", trajectory_action_uri_);
  // Initialize a trajectory action client, to a generic action that does not exist. This is remapped via ros-args when
  // launching the node
  trajectory_action_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(this, trajectory_action_uri_, clients_callback_group);

  declare_parameter("until_action_uri", "tool_contact_controller/detect_tool_contact");
  get_parameter("until_action_uri", until_action_uri_);

  // Create action server to advertise the "/trajectory_until_node/execute"
  action_server_ = rclcpp_action::create_server<TrajectoryUntilAction>(
      this, "~/execute",
      std::bind(&TrajectoryUntilNode::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryUntilNode::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&TrajectoryUntilNode::goal_accepted_callback, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), server_callback_group);
}

TrajectoryUntilNode::~TrajectoryUntilNode()
{
}

// Assign the correct type of action client to the variant
bool TrajectoryUntilNode::assign_until_action_client(std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
{
  switch (goal->until_type) {
    case TrajectoryUntilAction::Goal::TOOL_CONTACT:
      until_action_client_variant =
          rclcpp_action::create_client<TCAction>(this, until_action_uri_, clients_callback_group);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Received unknown until-type.");
      return false;
  }

  return true;
}

// Check if the node is capable of accepting a new action goal right now.
rclcpp_action::GoalResponse TrajectoryUntilNode::goal_received_callback(
    const rclcpp_action::GoalUUID& /* uuid */, std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
{
  if (server_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Node is currently busy, rejecting action goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!assign_until_action_client(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Until type not defined, double check the types in the action "
                                     "definition.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Trajectory action server at %s not available.", trajectory_action_uri_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!std::visit([](const auto& client) { return client->wait_for_action_server(std::chrono::seconds(1)); },
                  until_action_client_variant)) {
    RCLCPP_ERROR(this->get_logger(), "Until action server not available.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check until action server, send action goal to until-controller and wait for it to be accepted.
  if (std::holds_alternative<TCClient>(until_action_client_variant)) {
    send_until_goal<TCAction, TCClient>(goal);
  } else {
    throw std::runtime_error("Until type not implemented. This should not happen.");
  }

  // If it is not accepted within 1 seconds, it is
  // assumed to be rejected.
  std::unique_lock<std::mutex> until_lock(mutex_until);
  cv_until_.wait_for(until_lock, 1s);
  if (!until_accepted_) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Send action goal to trajectory controller and wait for it to be accepted. If it is not accepted within 1 seconds,
  // it is assumed to be rejected.
  send_trajectory_goal(goal);
  std::unique_lock<std::mutex> traj_lock(mutex_trajectory);
  cv_until_.wait_for(traj_lock, 1s);
  if (!trajectory_accepted_) {
    reset_node();
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Save the server goal handle for later use.
void TrajectoryUntilNode::goal_accepted_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> goal_handle)
{
  server_goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Accepted goal");
  return;
}

// If the trajectory_until action is canceled, it should cancel the until and trajectory actions, if any are running.
rclcpp_action::CancelResponse
TrajectoryUntilNode::goal_cancelled_callback(const std::shared_ptr<GoalHandleTrajectoryUntil> /* goal_handle */)
{
  RCLCPP_INFO(this->get_logger(), "Cancel received. Cancelling trajectory-until action.");
  if (current_until_goal_handle_) {
    cancel_until_goal();
  }
  if (current_trajectory_goal_handle_) {
    trajectory_action_client_->async_cancel_goal(current_trajectory_goal_handle_);
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

// Send the trajectory goal, using the relevant fields supplied from the action server
void TrajectoryUntilNode::send_trajectory_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> goal)
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
  trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
}

// Send the until goal, using the relevant fields supplied from the action server.
template <class ActionType, class ClientType>
void TrajectoryUntilNode::send_until_goal(std::shared_ptr<const TrajectoryUntilAction::Goal> /* goal */)
{
  // Setup goal
  auto goal_msg = typename ActionType::Goal();
  auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TrajectoryUntilNode::until_response_callback<ActionType>, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&TrajectoryUntilNode::until_result_callback<ActionType>, this, std::placeholders::_1);

  // Send goal
  std::get<ClientType>(until_action_client_variant)->async_send_goal(goal_msg, send_goal_options);
}

void TrajectoryUntilNode::trajectory_response_callback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr& goal_handle)
{
  {
    std::lock_guard<std::mutex> guard(mutex_trajectory);
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Trajectory accepted by server.");
      current_trajectory_goal_handle_ = goal_handle;
      trajectory_accepted_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Trajectory rejected by server.");
    }
  }
  cv_trajectory_.notify_one();
}

template <class T>
void TrajectoryUntilNode::until_response_callback(
    const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr& goal_handle)
{
  {
    std::lock_guard<std::mutex> guard(mutex_until);
    if (goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Until condition accepted by server.");
      current_until_goal_handle_ = goal_handle;
      until_accepted_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Until condition was rejected by server.");
    }
  }
  cv_until_.notify_one();
}

// Just forward feedback from trajectory controller. No feedback from until controllers so far.
void TrajectoryUntilNode::trajectory_feedback_callback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr& /* goal_handle */,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
  if (server_goal_handle_) {
    prealloc_fb_->actual = feedback->actual;
    prealloc_fb_->desired = feedback->desired;
    prealloc_fb_->error = feedback->error;
    prealloc_fb_->header = feedback->header;
    prealloc_fb_->joint_names = feedback->joint_names;
    if (server_goal_handle_) {
      server_goal_handle_->publish_feedback(prealloc_fb_);
    }
  }
}

// When a result is received from either trajectory or until condition, report it back to user
void TrajectoryUntilNode::trajectory_result_callback(const TrajectoryResult& result)
{
  RCLCPP_INFO(this->get_logger(), "Trajectory result received.");
  current_trajectory_goal_handle_.reset();
  report_goal(result);
}

template <class T>
void TrajectoryUntilNode::until_result_callback(
    const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& result)
{
  RCLCPP_INFO(this->get_logger(), "Until result received.");
  current_until_goal_handle_.reset();
  report_goal(result);
}

void TrajectoryUntilNode::report_goal(TrajectoryResult result)
{
  if (server_goal_handle_) {
    prealloc_res_->until_condition_result = TrajectoryUntilAction::Result::NOT_TRIGGERED;
    prealloc_res_->error_code = result.result->error_code;
    prealloc_res_->error_string = result.result->error_string;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        server_goal_handle_->succeed(prealloc_res_);
        break;

      case rclcpp_action::ResultCode::ABORTED:
        prealloc_res_->error_string += " Trajectory action was aborted. Aborting goal.";
        server_goal_handle_->abort(prealloc_res_);
        break;

      case rclcpp_action::ResultCode::CANCELED:
        prealloc_res_->error_string += " Trajectory action was canceled.";
        server_goal_handle_->canceled(prealloc_res_);
        break;

      default:
        prealloc_res_->error_string += " Unknown result code received from trajectory action, this should not happen. "
                                       "Aborting goal.";
        server_goal_handle_->abort(prealloc_res_);
        break;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "%s", prealloc_res_->error_string.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory finished successfully, did not trigger until condition.");
    }
  }
  reset_node();
}

template <typename UntilResult>
void TrajectoryUntilNode::report_goal(UntilResult result)
{
  if (server_goal_handle_) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        prealloc_res_->error_code = TrajectoryUntilAction::Result::SUCCESSFUL;
        prealloc_res_->until_condition_result = TrajectoryUntilAction::Result::TRIGGERED;
        prealloc_res_->error_string += "Trajectory finished successfully by triggering until condition.";
        server_goal_handle_->succeed(prealloc_res_);
        break;

      case rclcpp_action::ResultCode::ABORTED:
        prealloc_res_->error_string += "Until action was aborted. Aborting goal.";
        server_goal_handle_->abort(prealloc_res_);
        break;

      case rclcpp_action::ResultCode::CANCELED:
        prealloc_res_->error_string += "Until action was canceled.";
        server_goal_handle_->canceled(prealloc_res_);
        break;

      default:
        prealloc_res_->error_string += "Unknown result code received from until action, this should not happen. "
                                       "Aborting "
                                       "goal.";
        server_goal_handle_->abort(prealloc_res_);

        break;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "%s", prealloc_res_->error_string.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Trajectory finished by triggering until condition.");
    }
  }
  reset_node();
}

// Reset all variables and cancel running actions, if any.
void TrajectoryUntilNode::reset_node()
{
  if (current_until_goal_handle_) {
    cancel_until_goal();
  }
  if (current_trajectory_goal_handle_) {
    trajectory_action_client_->async_cancel_goal(current_trajectory_goal_handle_);
  }
  current_trajectory_goal_handle_ = nullptr;
  current_until_goal_handle_ = nullptr;
  trajectory_accepted_ = false;
  until_accepted_ = false;
  prealloc_res_ = std::make_shared<TrajectoryUntilAction::Result>(TrajectoryUntilAction::Result());
  prealloc_fb_ = std::make_shared<TrajectoryUntilAction::Feedback>(TrajectoryUntilAction::Feedback());

  server_goal_handle_ = nullptr;
}

// Cancel the action contained in the variant, regardless of what type it is.
void TrajectoryUntilNode::cancel_until_goal()
{
  std::visit([this](const auto& until_client) { until_client->async_cancel_goal(current_until_goal_handle_); },
             until_action_client_variant);
}

}  // namespace ur_robot_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_robot_driver::TrajectoryUntilNode>();
  // Use multithreaded executor because we have two callback groups
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
