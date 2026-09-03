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

// Tests for the concurrency and lifetime bugs reported in
// https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1943
//
// Goal-callback tests use the protected default constructor. Lifecycle tests
// exercise worker join, cancel, and interruptible dashboard waits through a
// real action server/client pair without contacting robot hardware.
// rclcpp::init() is required because RobotStateHelper inherits from rclcpp::Node.

// cppcheck-suppress-file syntaxError

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_client_library/ur/datatypes.h"
#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_robot_driver/robot_state_helper.hpp"

using namespace std::chrono_literals;

namespace ur_robot_driver
{
using SetMode = ur_dashboard_msgs::action::SetMode;

// Thin wrapper that uses the protected default constructor and exposes the
// internal state and callbacks needed by the tests.
class RobotStateHelperTestWrapper : public RobotStateHelper
{
public:
  RobotStateHelperTestWrapper() : RobotStateHelper()
  {
    result_ = std::make_shared<SetMode::Result>();
    feedback_ = std::make_shared<SetMode::Feedback>();
    headless_mode_ = false;
    program_running_ = false;
  }

  void setRobotMode(urcl::RobotMode mode)
  {
    robot_mode_ = mode;
  }
  void setSafetyMode(urcl::SafetyMode mode)
  {
    safety_mode_ = mode;
  }
  void setInAction(bool val)
  {
    in_action_ = val;
  }
  bool isInAction() const
  {
    return in_action_;
  }
  bool isGoalPending() const
  {
    return goal_pending_;
  }
  bool isStopRequested() const
  {
    return stop_requested_;
  }
  bool isWorkerJoinable() const
  {
    return worker_thread_.joinable();
  }

  void setHeadlessMode(bool val)
  {
    headless_mode_ = val;
  }
  void setProgramRunning(bool val)
  {
    program_running_ = val;
  }
  void setResendRobotProgramClient(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client)
  {
    resend_robot_program_srv_ = client;
  }

  // Simulate a worker stuck in a poll loop, as setModeExecute does while waiting.
  void startBlockingWorker()
  {
    stop_requested_ = false;
    in_action_ = true;
    worker_thread_ = std::thread([this]() {
      while (!stop_requested_ && rclcpp::ok()) {
        std::this_thread::sleep_for(10ms);
      }
      in_action_ = false;
    });
  }

  rclcpp_action::GoalResponse callGoalCallback(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const SetMode::Goal> goal)
  {
    return setModeGoalCallback(uuid, goal);
  }

  rclcpp_action::CancelResponse callCancelCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle)
  {
    return setModeCancelCallback(goal_handle);
  }

  void callAcceptCallback(const std::shared_ptr<SetModeGoalHandle> goal_handle)
  {
    setModeAcceptCallback(goal_handle);
  }

  std::optional<bool> callSafeDashboardTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& srv)
  {
    return safeDashboardTrigger(srv);
  }

  void requestStop()
  {
    stop_requested_ = true;
  }
};

// ---------------------------------------------------------------------------
// Helper factories
// ---------------------------------------------------------------------------

static rclcpp_action::GoalUUID makeUuid(uint8_t fill = 0)
{
  rclcpp_action::GoalUUID uuid{};
  uuid.fill(fill);
  return uuid;
}

static std::shared_ptr<SetMode::Goal> makeGoal(int mode = 7 /* RUNNING */, bool play_program = false)
{
  auto g = std::make_shared<SetMode::Goal>();
  g->target_robot_mode = mode;
  g->play_program = play_program;
  g->stop_program = false;
  return g;
}

static bool waitFor(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

// ---------------------------------------------------------------------------
// Goal-callback tests
// ---------------------------------------------------------------------------

// Regression for issue #1943: the goal callback must reject a new goal while
// a SetMode action is already in progress.  Before the fix, in_action_ was
// never checked and a second goal was silently accepted, leading to shared-
// state corruption.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenInAction)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::RUNNING);
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(true);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  // cppcheck-suppress knownConditionTrueFalse
  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Baseline: a goal must be accepted when the robot mode and safety mode are
// known and no action is currently running.
TEST(RobotStateHelperGoalCallback, AcceptsGoalWhenModesKnownAndIdle)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::POWER_OFF);
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  EXPECT_EQ(response, expected);
  EXPECT_TRUE(helper.isGoalPending());
}

// Regression: once a goal is pending acceptance, a second goal must be rejected
// before the accept callback clears goal_pending_.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenGoalPending)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::RUNNING);
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);

  EXPECT_EQ(helper.callGoalCallback(makeUuid(1), makeGoal()), rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  EXPECT_TRUE(helper.isGoalPending());
  EXPECT_EQ(helper.callGoalCallback(makeUuid(2), makeGoal()), rclcpp_action::GoalResponse::REJECT);
}

// Regression: the goal callback must reject when robot mode is still UNKNOWN
// (robot not yet connected), regardless of in_action_.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenRobotModeUnknown)
{
  RobotStateHelperTestWrapper helper;
  // robot_mode_ stays UNKNOWN
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Regression: the goal callback must reject when safety mode is still
// UNDEFINED, regardless of in_action_.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenSafetyModeUndefined)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::RUNNING);
  // safety_mode_ stays UNDEFINED_SAFETY_MODE
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Verify that in_action_ is initially false so a fresh helper is immediately
// ready to accept a goal.
TEST(RobotStateHelperGoalCallback, InActionFalseOnConstruction)
{
  RobotStateHelperTestWrapper helper;
  EXPECT_FALSE(helper.isInAction());
}

// ---------------------------------------------------------------------------
// Lifecycle / interruptibility tests
// ---------------------------------------------------------------------------

TEST(RobotStateHelperLifecycle, DestructorJoinsBlockingWorker)
{
  auto helper = std::make_unique<RobotStateHelperTestWrapper>();
  helper->startBlockingWorker();
  ASSERT_TRUE(helper->isWorkerJoinable());
  ASSERT_TRUE(helper->isInAction());

  const auto start = std::chrono::steady_clock::now();
  helper.reset();  // ~RobotStateHelper sets stop_requested_ and joins.
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 1s);
}

TEST(RobotStateHelperLifecycle, CancelRejectsWhenNotRunning)
{
  RobotStateHelperTestWrapper helper;
  // Short-circuit on !in_action_ before dereferencing the handle.
  EXPECT_EQ(helper.callCancelCallback(nullptr), rclcpp_action::CancelResponse::REJECT);
  EXPECT_FALSE(helper.isStopRequested());
}

TEST(RobotStateHelperLifecycle, SafeDashboardTriggerInterruptedByStopRequest)
{
  auto helper = std::make_shared<RobotStateHelperTestWrapper>();
  auto service_node = std::make_shared<rclcpp::Node>("hanging_trigger_server");
  std::atomic<bool> keep_hanging{ true };
  // Block until the test clears keep_hanging so the client poll loop can be interrupted.
  auto service = service_node->create_service<std_srvs::srv::Trigger>(
      "hanging_trigger", [&keep_hanging](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> /*response*/) {
        while (keep_hanging && rclcpp::ok()) {
          std::this_thread::sleep_for(100ms);
        }
      });

  auto client = helper->create_client<std_srvs::srv::Trigger>("hanging_trigger");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(helper);
  executor.add_node(service_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  ASSERT_TRUE(client->wait_for_service(2s));

  std::promise<std::optional<bool>> promise;
  auto future = promise.get_future();
  std::thread call_thread(
      [helper, client, &promise]() { promise.set_value(helper->callSafeDashboardTrigger(client)); });

  // Give the poll loop time to start, then interrupt it.
  std::this_thread::sleep_for(200ms);
  ASSERT_NE(future.wait_for(0s), std::future_status::ready);
  helper->requestStop();

  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(future.get(), std::nullopt);

  call_thread.join();
  keep_hanging = false;
  executor.cancel();
  spin_thread.join();
}

// Action server/client fixture that drives the real accept/execute/cancel path
// without hardware by blocking on a never-responding Trigger service.
class RobotStateHelperActionFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<RobotStateHelperTestWrapper>();
    helper_->setRobotMode(urcl::RobotMode::RUNNING);
    helper_->setSafetyMode(urcl::SafetyMode::NORMAL);
    helper_->setHeadlessMode(true);
    helper_->setProgramRunning(false);

    keep_hanging_ = true;
    service_node_ = std::make_shared<rclcpp::Node>("hanging_resend_server");
    hanging_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
        "hanging_resend_robot_program", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> /*response*/) {
          while (keep_hanging_ && rclcpp::ok()) {
            std::this_thread::sleep_for(100ms);
          }
        });

    helper_->setResendRobotProgramClient(helper_->create_client<std_srvs::srv::Trigger>("hanging_resend_robot_"
                                                                                        "program"));

    // Use the production goal/cancel/accept callbacks so worker lifecycle is real.
    action_server_ = rclcpp_action::create_server<SetMode>(
        helper_, "~/set_mode",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SetMode::Goal> goal) {
          return helper_->callGoalCallback(uuid, goal);
        },
        [this](const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle) {
          return helper_->callCancelCallback(goal_handle);
        },
        [this](const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle) {
          // Invoke the real accept callback (starts setModeExecute on a worker).
          helper_->callAcceptCallback(goal_handle);
        });

    client_node_ = std::make_shared<rclcpp::Node>("set_mode_test_client");
    action_client_ = rclcpp_action::create_client<SetMode>(client_node_, "/robot_state_helper/set_mode");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(helper_);
    executor_->add_node(service_node_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    ASSERT_TRUE(helper_->create_client<std_srvs::srv::Trigger>("hanging_resend_robot_program")->wait_for_service(2s));
  }

  void TearDown() override
  {
    keep_hanging_ = false;
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    action_client_.reset();
    action_server_.reset();
    helper_.reset();
    service_node_.reset();
    client_node_.reset();
    hanging_service_.reset();
    executor_.reset();
  }

  std::atomic<bool> keep_hanging_{ true };
  std::shared_ptr<RobotStateHelperTestWrapper> helper_;
  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hanging_service_;
  rclcpp_action::Server<SetMode>::SharedPtr action_server_;
  rclcpp_action::Client<SetMode>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(RobotStateHelperActionFixture, CancelInterruptsBlockingExecuteAndJoinsWorker)
{
  auto goal = makeGoal(static_cast<int>(urcl::RobotMode::RUNNING), /*play_program=*/true);
  auto goal_future = action_client_->async_send_goal(*goal);
  ASSERT_EQ(goal_future.wait_for(5s), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(waitFor([this]() { return helper_->isInAction() && helper_->isWorkerJoinable(); }, 5s));

  auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(5s), std::future_status::ready);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  auto wrapped = result_future.get();

  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(wrapped.result, nullptr);
  EXPECT_FALSE(wrapped.result->success);

  ASSERT_TRUE(waitFor([this]() { return !helper_->isInAction(); }, 2s));
}

TEST_F(RobotStateHelperActionFixture, DestructorJoinsWorkerBlockedInDashboardTrigger)
{
  auto goal = makeGoal(static_cast<int>(urcl::RobotMode::RUNNING), /*play_program=*/true);
  auto goal_future = action_client_->async_send_goal(*goal);
  ASSERT_EQ(goal_future.wait_for(5s), std::future_status::ready);
  ASSERT_NE(goal_future.get(), nullptr);

  ASSERT_TRUE(waitFor([this]() { return helper_->isInAction() && helper_->isWorkerJoinable(); }, 5s));

  // Drop action endpoints before destroying the helper so teardown is orderly.
  action_client_.reset();
  action_server_.reset();
  executor_->remove_node(helper_);

  const auto start = std::chrono::steady_clock::now();
  helper_.reset();  // Sets stop_requested_ and joins the worker blocked in safeDashboardTrigger.
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 2s);
}

}  // namespace ur_robot_driver

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
