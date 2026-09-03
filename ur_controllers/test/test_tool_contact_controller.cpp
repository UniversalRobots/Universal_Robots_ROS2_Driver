// Copyright 2026 Universal Robots A/S
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
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "ur_controllers/tool_contact_controller.hpp"

// Unit tests for ToolContactController::update() state/command interactions.
// Includes the #1939 regression: abort must not be overwritten by EXECUTING.

class ToolContactControllerTest : public ::testing::Test
{
public:
  static constexpr double TOOL_CONTACT_STANDBY = 1.0;
  static constexpr double TOOL_CONTACT_WAITING_BEGIN = 2.0;
  static constexpr double TOOL_CONTACT_EXECUTING = 3.0;
  static constexpr double TOOL_CONTACT_FAILURE_BEGIN = 4.0;
  static constexpr double TOOL_CONTACT_WAITING_END = 5.0;
  static constexpr double TOOL_CONTACT_SUCCESS_END = 6.0;
  static constexpr double TOOL_CONTACT_FAILURE_END = 7.0;

protected:
  void SetUp() override
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "tool_contact_controller_test";
    params.robot_description = "";
    params.update_rate = 500;
    params.controller_manager_update_rate = 500;
    params.node_namespace = "";
    params.node_options = controller_.define_custom_node_options();
    ASSERT_EQ(controller_.init(params), controller_interface::return_type::OK);

    set_state_value_ = TOOL_CONTACT_STANDBY;
    state_value_ = TOOL_CONTACT_STANDBY;
    result_value_ = 3.0;

    command_interface_ = std::make_shared<hardware_interface::CommandInterface>(
        "tool_contact", "tool_contact_set_state", &set_state_value_);
    state_interface_ =
        std::make_shared<hardware_interface::StateInterface>("tool_contact", "tool_contact_state", &state_value_);
    result_interface_ =
        std::make_shared<hardware_interface::StateInterface>("tool_contact", "tool_contact_result", &result_value_);

    loaned_command_ = std::make_unique<hardware_interface::LoanedCommandInterface>(command_interface_);
    loaned_state_ = std::make_unique<hardware_interface::LoanedStateInterface>(state_interface_);
    loaned_result_ = std::make_unique<hardware_interface::LoanedStateInterface>(result_interface_);

    controller_.tool_contact_set_state_interface_ = *loaned_command_;
    controller_.tool_contact_state_interface_ = *loaned_state_;
    controller_.tool_contact_result_interface_ = *loaned_result_;

    // No active goal; try_get succeeds and yields nullptr.
    ASSERT_TRUE(controller_.set_rt_goal_from_non_rt(nullptr));
  }

  void TearDown() override
  {
    if (controller_.get_node()) {
      controller_.get_node()->shutdown();
    }
  }

  void request_abort()
  {
    controller_.tool_contact_abort_ = true;
    controller_.tool_contact_enable_ = false;
  }

  void request_enable()
  {
    controller_.tool_contact_abort_ = false;
    controller_.tool_contact_enable_ = true;
  }

  void request_abort_and_enable()
  {
    controller_.tool_contact_abort_ = true;
    controller_.tool_contact_enable_ = true;
  }

  void clear_requests()
  {
    controller_.tool_contact_abort_ = false;
    controller_.tool_contact_enable_ = false;
  }

  void set_hw_state(double state, double result = 3.0)
  {
    state_value_ = state;
    result_value_ = result;
  }

  void set_active(bool active)
  {
    controller_.tool_contact_active_ = active;
  }

  void set_logged_once(bool logged)
  {
    controller_.logged_once_ = logged;
  }

  bool abort_requested() const
  {
    return controller_.tool_contact_abort_;
  }
  bool enable_requested() const
  {
    return controller_.tool_contact_enable_;
  }
  bool is_active() const
  {
    return controller_.tool_contact_active_;
  }
  bool logged_once() const
  {
    return controller_.logged_once_;
  }
  bool should_reset_goal() const
  {
    return controller_.should_reset_goal;
  }

  controller_interface::return_type run_update()
  {
    return controller_.update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
  }

  // Hold rt_active_goal_'s mutex so update()'s try_get fails (contention path).
  template <typename Fn>
  void with_goal_box_contended(Fn&& fn)
  {
    std::atomic<bool> lock_held{ false };
    std::atomic<bool> release_lock{ false };
    std::thread holder([this, &lock_held, &release_lock]() {
      std::unique_lock lock(controller_.rt_active_goal_.get_mutex());
      lock_held = true;
      while (!release_lock.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!lock_held.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(lock_held.load()) << "Timed out waiting to hold the goal box mutex.";

    std::forward<Fn>(fn)();

    release_lock = true;
    holder.join();
  }

  ur_controllers::ToolContactController controller_;
  double set_state_value_{};
  double state_value_{};
  double result_value_{};
  std::shared_ptr<hardware_interface::CommandInterface> command_interface_;
  std::shared_ptr<hardware_interface::StateInterface> state_interface_;
  std::shared_ptr<hardware_interface::StateInterface> result_interface_;
  std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command_;
  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_state_;
  std::unique_ptr<hardware_interface::LoanedStateInterface> loaned_result_;
};

// ---------------------------------------------------------------------------
// Early-exit command requests
// ---------------------------------------------------------------------------

TEST_F(ToolContactControllerTest, AbortWhileExecutingKeepsWaitingEndCommand)
{
  // Regression (#1939): abort must not be overwritten by the EXECUTING branch.
  request_abort();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 3.0);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);
  EXPECT_FALSE(abort_requested());
  EXPECT_FALSE(enable_requested());
}

TEST_F(ToolContactControllerTest, EnableWhileIdleSetsWaitingBeginAndReturns)
{
  request_enable();
  set_state_value_ = TOOL_CONTACT_STANDBY;
  set_hw_state(TOOL_CONTACT_STANDBY);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_BEGIN);
  EXPECT_FALSE(enable_requested());
}

TEST_F(ToolContactControllerTest, AbortTakesPriorityOverEnable)
{
  request_abort_and_enable();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 3.0);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);
  EXPECT_FALSE(abort_requested());
  EXPECT_FALSE(enable_requested());
}

TEST_F(ToolContactControllerTest, AbortDoesNotFallThroughToStateMachine)
{
  // Even when HW reports SUCCESS_END, abort must win and leave WAITING_END.
  request_abort();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_SUCCESS_END);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);
}

// ---------------------------------------------------------------------------
// TOOL_CONTACT_EXECUTING
// ---------------------------------------------------------------------------

TEST_F(ToolContactControllerTest, ExecutingNonTerminalResultAcknowledgesExecuting)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_BEGIN;
  set_hw_state(TOOL_CONTACT_EXECUTING, 3.0);
  set_logged_once(false);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_EXECUTING);
  EXPECT_TRUE(is_active());
  EXPECT_TRUE(logged_once());
  EXPECT_FALSE(should_reset_goal());
}

TEST_F(ToolContactControllerTest, ExecutingSuccessResultRequestsWaitingEnd)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 0.0);
  set_active(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);
  EXPECT_FALSE(is_active());
  // No active goal handle, so should_reset_goal stays false.
  EXPECT_FALSE(should_reset_goal());
}

TEST_F(ToolContactControllerTest, ExecutingHardwareAbortResultSetsStandby)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 1.0);
  set_active(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(is_active());
}

// ---------------------------------------------------------------------------
// Goal-box contention: try_get fails while another thread holds the mutex
// ---------------------------------------------------------------------------

TEST_F(ToolContactControllerTest, ContendedGoalBoxExecutingNonTerminalAcknowledgesExecuting)
{
  // Non-terminal EXECUTING must still be acknowledged so startToolContact is
  // not retriggered, even when the goal handle cannot be read this cycle.
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_BEGIN;
  set_hw_state(TOOL_CONTACT_EXECUTING, 3.0);
  set_logged_once(false);

  with_goal_box_contended([this]() {
    EXPECT_EQ(run_update(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_EXECUTING);
    EXPECT_TRUE(is_active());
    EXPECT_TRUE(logged_once());
    EXPECT_FALSE(should_reset_goal());
  });
}

TEST_F(ToolContactControllerTest, ContendedGoalBoxExecutingSuccessDefersTerminalHandling)
{
  // Terminal success must NOT write WAITING_END while the goal box is
  // contended; goal/result handling is deferred to a later cycle.
  clear_requests();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 0.0);
  set_active(true);
  set_logged_once(true);

  with_goal_box_contended([this]() {
    EXPECT_EQ(run_update(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_EXECUTING);
    EXPECT_TRUE(is_active());
    EXPECT_FALSE(should_reset_goal());
  });
}

TEST_F(ToolContactControllerTest, ContendedGoalBoxExecutingHardwareAbortDefersTerminalHandling)
{
  // Terminal hardware abort must NOT write STANDBY while the goal box is
  // contended; goal/result handling is deferred to a later cycle.
  clear_requests();
  set_state_value_ = TOOL_CONTACT_EXECUTING;
  set_hw_state(TOOL_CONTACT_EXECUTING, 1.0);
  set_active(true);
  set_logged_once(true);

  with_goal_box_contended([this]() {
    EXPECT_EQ(run_update(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_EXECUTING);
    EXPECT_TRUE(is_active());
    EXPECT_FALSE(should_reset_goal());
  });
}

TEST_F(ToolContactControllerTest, ContendedGoalBoxNonExecutingLeavesCommandUnchanged)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_STANDBY;
  set_hw_state(TOOL_CONTACT_STANDBY);
  set_logged_once(true);

  with_goal_box_contended([this]() {
    EXPECT_EQ(run_update(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
    // STANDBY logging clear only runs in the uncontended switch.
    EXPECT_TRUE(logged_once());
  });
}

// ---------------------------------------------------------------------------
// TOOL_CONTACT_FAILURE_BEGIN / SUCCESS_END / FAILURE_END / STANDBY
// ---------------------------------------------------------------------------

TEST_F(ToolContactControllerTest, FailureBeginSetsStandbyAndClearsActive)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_BEGIN;
  set_hw_state(TOOL_CONTACT_FAILURE_BEGIN);
  set_active(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(is_active());
}

TEST_F(ToolContactControllerTest, SuccessEndSetsStandbyAndClearsActive)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_END;
  set_hw_state(TOOL_CONTACT_SUCCESS_END);
  set_active(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(is_active());
}

TEST_F(ToolContactControllerTest, SuccessEndAlwaysWritesStandbyEvenIfAlreadyInactive)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_END;
  set_hw_state(TOOL_CONTACT_SUCCESS_END);
  set_active(false);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(is_active());
}

TEST_F(ToolContactControllerTest, FailureEndSetsStandby)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_END;
  set_hw_state(TOOL_CONTACT_FAILURE_END);
  set_active(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
}

TEST_F(ToolContactControllerTest, StandbyClearsLoggedOnceWithoutChangingCommand)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_STANDBY;
  set_hw_state(TOOL_CONTACT_STANDBY);
  set_logged_once(true);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(logged_once());
}

TEST_F(ToolContactControllerTest, WaitingBeginStateIsPassthroughDefault)
{
  // Intermediate HW states fall through the default branch; command is unchanged.
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_BEGIN;
  set_hw_state(TOOL_CONTACT_WAITING_BEGIN);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_BEGIN);
}

TEST_F(ToolContactControllerTest, WaitingEndStateIsPassthroughDefault)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_WAITING_END;
  set_hw_state(TOOL_CONTACT_WAITING_END);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);
}

TEST_F(ToolContactControllerTest, UnknownStateLeavesCommandUnchanged)
{
  clear_requests();
  set_state_value_ = TOOL_CONTACT_STANDBY;
  set_hw_state(99.0);

  EXPECT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
}

// ---------------------------------------------------------------------------
// Enable then progress through a happy-path handshake without abort
// ---------------------------------------------------------------------------

TEST_F(ToolContactControllerTest, EnableThenExecutingHandshakeSequence)
{
  // 1) Enable request -> WAITING_BEGIN
  request_enable();
  set_state_value_ = TOOL_CONTACT_STANDBY;
  set_hw_state(TOOL_CONTACT_STANDBY);
  ASSERT_EQ(run_update(), controller_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_BEGIN);

  // 2) HW reports EXECUTING with non-terminal result -> acknowledge EXECUTING
  clear_requests();
  set_hw_state(TOOL_CONTACT_EXECUTING, 3.0);
  ASSERT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_EXECUTING);
  EXPECT_TRUE(is_active());

  // 3) Abort while still executing -> WAITING_END (regression path)
  request_abort();
  ASSERT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_WAITING_END);

  // 4) HW reports SUCCESS_END -> STANDBY
  clear_requests();
  set_hw_state(TOOL_CONTACT_SUCCESS_END);
  ASSERT_EQ(run_update(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(set_state_value_, TOOL_CONTACT_STANDBY);
  EXPECT_FALSE(is_active());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
