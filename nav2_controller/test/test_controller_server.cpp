// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

class ControllerServerShim : public nav2_controller::ControllerServer
{
public:
  using nav2_controller::ControllerServer::ControllerServer;

  void setEndPoseFrame(const std::string & frame) {end_pose_.header.frame_id = frame;}
  void callTransformedPlanAndGoal() {transformedPlanAndGoal();}
  nav2::TransformBuffer & getTfBuffer() {return *costmap_ros_->getTfBuffer();}
  void selectGoalChecker(const std::string & id, nav2_core::GoalChecker::Ptr checker)
  {
    checker->initialize(shared_from_this(), id, costmap_ros_);
    goal_checkers_[id] = checker;
    current_goal_checker_ = id;
  }
  using nav2_controller::ControllerServer::Action;
  void callApplyGoalTolerances(const Action::Goal & goal) {applyGoalTolerances(goal);}
  /// Adds plugins built by the test, to be done between configure and activate
  void addPlugins(
    const std::map<std::string, nav2_core::Controller::Ptr> & controllers,
    nav2_core::GoalChecker::Ptr goal_checker, nav2_core::PathHandler::Ptr path_handler)
  {
    controllers_.insert(controllers.begin(), controllers.end());
    goal_checkers_["goal_checker"] = goal_checker;
    path_handlers_["path_handler"] = path_handler;
  }
};

/// Calls made by the control loop to the scripted test plugins, in order
class EventLog
{
public:
  void add(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(event);
  }

  std::vector<std::string> events()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  int64_t count(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::count(events_.begin(), events_.end(), event);
  }

private:
  std::mutex mutex_;
  std::vector<std::string> events_;
};

/// Controller whose velocity commands follow a script, one step per control cycle
class ScriptedController : public nav2_core::Controller
{
public:
  ScriptedController(
    const std::string & name, std::shared_ptr<EventLog> log,
    std::shared_ptr<std::atomic<bool>> goal_reached)
  : name_(name), log_(log), goal_reached_(goal_reached) {}

  /**
   * @brief Queues the next control cycles
   * @param steps nullptr for a valid command, otherwise the exception thrown in that cycle
   * @param reach_goal Whether the goal is reached once the steps are done; until then and
   * afterwards the controller returns valid commands
   */
  void script(std::vector<std::exception_ptr> steps, bool reach_goal = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    steps_.assign(steps.begin(), steps.end());
    reach_goal_ = reach_goal;
  }

  /// Results of the next cancel() calls, after which cancel() returns cancel_result
  void scriptCancel(std::vector<bool> results)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_results_.assign(results.begin(), results.end());
  }

  /// Steps replacing the script at the next cancel() call, as script() does
  void scriptOnCancel(std::vector<std::exception_ptr> steps, bool reach_goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_cancel_steps_ = std::move(steps);
    on_cancel_reach_goal_ = reach_goal;
    has_on_cancel_script_ = true;
  }

  void configure(
    const nav2::LifecycleNode::WeakPtr &, std::string, nav2::TransformBuffer::SharedPtr,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setSpeedLimit(const double &, const bool &) override {}

  void newPathReceived(const nav_msgs::msg::Path &) override {log_->add(name_ + ".path");}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *, const nav_msgs::msg::Path &,
    const geometry_msgs::msg::PoseStamped &) override
  {
    log_->add(name_ + ".compute");
    std::exception_ptr step;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (steps_.empty()) {
        if (reach_goal_) {
          *goal_reached_ = true;
        }
      } else {
        step = steps_.front();
        steps_.pop_front();
      }
    }
    if (step) {
      std::rethrow_exception(step);
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.5;
    return cmd;
  }

  bool cancel() override
  {
    log_->add(name_ + ".cancel");
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_on_cancel_script_) {
      steps_.assign(on_cancel_steps_.begin(), on_cancel_steps_.end());
      reach_goal_ = on_cancel_reach_goal_;
      has_on_cancel_script_ = false;
    }
    if (cancel_results_.empty()) {
      return cancel_result;
    }
    const bool result = cancel_results_.front();
    cancel_results_.pop_front();
    return result;
  }

  void reset() override {log_->add(name_ + ".reset");}

  /// Result of cancel()
  std::atomic<bool> cancel_result{true};

private:
  std::string name_;
  std::shared_ptr<EventLog> log_;
  std::shared_ptr<std::atomic<bool>> goal_reached_;
  std::mutex mutex_;
  std::deque<std::exception_ptr> steps_;
  bool reach_goal_{true};
  std::deque<bool> cancel_results_;
  std::vector<std::exception_ptr> on_cancel_steps_;
  bool on_cancel_reach_goal_{false};
  bool has_on_cancel_script_{false};
};

/// Goal checker reporting the flag set by the scripted controllers, cleared on every new path;
/// logs "goal.check" on every check
class FlagGoalChecker : public nav2_core::GoalChecker
{
public:
  FlagGoalChecker(std::shared_ptr<std::atomic<bool>> goal_reached, std::shared_ptr<EventLog> log)
  : goal_reached_(goal_reached), log_(log) {}

  void initialize(
    const nav2::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {*goal_reached_ = false;}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &, const nav_msgs::msg::Path &) override
  {
    log_->add("goal.check");
    return *goal_reached_;
  }
  bool isGoalXYReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &, const nav_msgs::msg::Path &) override
  {
    return *goal_reached_;
  }
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &, double &) override
  {
    return false;
  }

private:
  std::shared_ptr<std::atomic<bool>> goal_reached_;
  std::shared_ptr<EventLog> log_;
};

/// Path handler handing the whole plan to the controller untransformed
class PassThroughPathHandler : public nav2_core::PathHandler
{
public:
  void initialize(
    const nav2::LifecycleNode::WeakPtr &, const rclcpp::Logger &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>,
    nav2::TransformBuffer::SharedPtr) override {}
  void setPlan(const nav_msgs::msg::Path & path) override {path_ = path;}
  nav2_core::PathSegment findPlanSegment(const geometry_msgs::msg::PoseStamped &) override
  {
    return {path_.poses.begin(), path_.poses.end()};
  }
  nav_msgs::msg::Path transformLocalPlan(
    const nav2_core::PathIterator & closest_point,
    const nav2_core::PathIterator & pruned_plan_end) override
  {
    nav_msgs::msg::Path plan;
    plan.header = path_.header;
    plan.poses.assign(closest_point, pruned_plan_end);
    return plan;
  }
  geometry_msgs::msg::PoseStamped getTransformedGoal(const builtin_interfaces::msg::Time &) override
  {
    return path_.poses.back();
  }

private:
  nav_msgs::msg::Path path_;
};

/// Error code of the ControllerException thrown by the scripted controllers
constexpr uint16_t kControllerErrorCode = 10109;

/// Exception step of a ScriptedController script
template<typename ExceptionT, typename ... Args>
std::exception_ptr fail(Args &&... args)
{
  return std::make_exception_ptr(ExceptionT(std::forward<Args>(args)...));
}

/// Controller failure retried only when retry_all_failures is set
std::exception_ptr blocked()
{
  return fail<nav2_core::ControllerException>("blocked", kControllerErrorCode);
}

/**
 * @brief Active controller server with scripted controllers "A", "B" and "C", driven through its
 * FollowPath action by a client node; each test runs in its own namespace
 */
class FollowPathTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  void startServer(const std::vector<rclcpp::Parameter> & parameters)
  {
    static int test_index = 0;
    const std::string ns = "/follow_path_test_" + std::to_string(test_index++);
    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
      rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
      rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
      rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("filters", std::vector<std::string>{}),
      rclcpp::Parameter("controller_frequency", 100.0),
      rclcpp::Parameter("costmap_update_timeout", 0.0),
      rclcpp::Parameter("bond_heartbeat_period", 0.0),
    };
    overrides.insert(overrides.end(), parameters.begin(), parameters.end());
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    options.parameter_overrides(overrides);

    server_ = std::make_shared<ControllerServerShim>(options);
    ASSERT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = server_->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.rotation.w = 1.0;
    server_->getTfBuffer().setTransform(tf_msg, "test", true);
    server_->addPlugins(
      {{"A", controller_a_}, {"B", controller_b_}, {"C", controller_c_}},
      std::make_shared<FlagGoalChecker>(goal_reached_, log_),
      std::make_shared<PassThroughPathHandler>());
    ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = std::make_shared<rclcpp::Node>("follow_path_client", ns);
    client_ = rclcpp_action::create_client<Action>(client_node_, "follow_path");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(10)));
  }

  void TearDown() override
  {
    if (server_) {
      server_->deactivate();
      server_->cleanup();
    }
  }

  /// Sends a FollowPath goal for the given controller and waits for its acceptance
  GoalHandle::SharedPtr sendGoal(const std::string & controller_id)
  {
    Action::Goal goal;
    goal.controller_id = controller_id;
    goal.path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    pose.pose.orientation.w = 1.0;
    goal.path.poses.push_back(pose);
    auto future = client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return nullptr;
    }
    return future.get();
  }

  /// Waits for the result of a goal
  GoalHandle::WrappedResult result(const GoalHandle::SharedPtr & goal_handle)
  {
    auto future = client_->async_get_result(goal_handle);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(10)),
      rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  /// Sends a goal for the given controller and waits for its result
  GoalHandle::WrappedResult followPath(const std::string & controller_id)
  {
    auto goal_handle = sendGoal(controller_id);
    EXPECT_NE(goal_handle, nullptr);
    return result(goal_handle);
  }

  /// Waits until the log holds the event at least count times
  bool waitFor(const std::string & event, int64_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (log_->count(event) < count) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  std::shared_ptr<EventLog> log_ = std::make_shared<EventLog>();
  std::shared_ptr<std::atomic<bool>> goal_reached_ = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<ScriptedController> controller_a_ =
    std::make_shared<ScriptedController>("A", log_, goal_reached_);
  std::shared_ptr<ScriptedController> controller_b_ =
    std::make_shared<ScriptedController>("B", log_, goal_reached_);
  std::shared_ptr<ScriptedController> controller_c_ =
    std::make_shared<ScriptedController>("C", log_, goal_reached_);
  std::shared_ptr<ControllerServerShim> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
};

using ResultCode = rclcpp_action::ResultCode;
using FollowPathResult = nav2_msgs::action::FollowPath::Result;

TEST_F(FollowPathTest, DefaultRetriesOnlyNoValidControl)
{
  startServer({rclcpp::Parameter("failure_tolerance", -1.0)});

  controller_a_->script({fail<nav2_core::NoValidControl>("no control"),
      fail<nav2_core::NoValidControl>("no control")});
  auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_EQ(log_->count("A.compute"), 3);

  controller_a_->script({blocked()});
  result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, kControllerErrorCode);
  EXPECT_EQ(log_->count("A.compute"), 4);
}

TEST_F(FollowPathTest, RetryAllFailuresRecoversWithinMaxRetries)
{
  startServer({
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 2),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  // The valid command resets the count of consecutive failures
  controller_a_->script({blocked(), blocked(), nullptr, blocked(), blocked()});
  const auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_EQ(log_->count("A.compute"), 6);
}

TEST_F(FollowPathTest, RetryAllFailuresEndsAfterMaxRetries)
{
  startServer({
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 2),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  controller_a_->script({blocked(), blocked(), nullptr, blocked(), blocked(), blocked()});
  auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, kControllerErrorCode);
  EXPECT_EQ(log_->count("A.compute"), 6);

  // A new goal starts with no failures counted
  controller_a_->script({blocked(), blocked()});
  result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
}

TEST_F(FollowPathTest, ZeroMaxRetriesEndsAtFirstFailure)
{
  startServer({
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 0),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  controller_a_->script({blocked()});
  const auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, kControllerErrorCode);
  EXPECT_EQ(log_->count("A.compute"), 1);
}

TEST_F(FollowPathTest, RetryAllFailuresSkipsInvalidControllerAndInternalErrors)
{
  startServer({
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  controller_a_->script({fail<nav2_core::InvalidController>("invalid")});
  auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, FollowPathResult::INVALID_CONTROLLER);
  EXPECT_EQ(log_->count("A.compute"), 1);

  controller_a_->script({fail<std::runtime_error>("plugin bug")});
  result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, FollowPathResult::UNKNOWN);
  EXPECT_EQ(log_->count("A.compute"), 2);
}

TEST_F(FollowPathTest, FailureToleranceEndsRetriesBeforeMaxRetries)
{
  // 200 retries take 2 s at 100 Hz, ten times the failure tolerance
  startServer({
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 200),
    rclcpp::Parameter("failure_tolerance", 0.2)});

  controller_a_->script(std::vector<std::exception_ptr>(250, blocked()));
  const auto result = followPath("A");
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, FollowPathResult::PATIENCE_EXCEEDED);
  EXPECT_GE(log_->count("A.compute"), 2);
  EXPECT_LE(log_->count("A.compute"), 200);
}

TEST_F(FollowPathTest, RetriedFailurePublishesZeroVelocity)
{
  startServer({rclcpp::Parameter("failure_tolerance", -1.0)});

  std::mutex mutex;
  std::vector<double> speeds;
  auto recorder =
    std::make_shared<rclcpp::Node>("cmd_vel_recorder", client_node_->get_namespace());
  auto sub = recorder->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", rclcpp::QoS(100), [&](const geometry_msgs::msg::TwistStamped & msg) {
      std::lock_guard<std::mutex> lock(mutex);
      speeds.push_back(msg.twist.linear.x);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  std::thread spinner([&executor]() {executor.spin();});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  controller_a_->script({nullptr, fail<nav2_core::NoValidControl>("no control"), nullptr});
  const auto result = followPath("A");
  // Valid command, retried failure, valid commands until the goal is reached, goal exit
  const std::vector<double> expected{0.5, 0.0, 0.5, 0.5, 0.0};
  auto received = [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      return speeds.size();
    };
  while (received() < expected.size() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.cancel();
  spinner.join();
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_EQ(speeds, expected);
}

/// Number of occurrences of an event before the first occurrence of another one
int64_t countBefore(
  const std::vector<std::string> & events, const std::string & event, const std::string & until)
{
  return std::count(events.begin(), std::find(events.begin(), events.end(), until), event);
}

TEST_F(FollowPathTest, ControllerChangeResetsFailureCount)
{
  // 20 Hz leaves A 15 cycles (0.75 s) of retries to receive the preemption
  startServer({
    rclcpp::Parameter("controller_frequency", 20.0),
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 50),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  controller_a_->script(std::vector<std::exception_ptr>(1000, blocked()));
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 35));

  // B gets its own max_retries, whatever A failed before
  controller_b_->script(std::vector<std::exception_ptr>(30, blocked()));
  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_LE(countBefore(log_->events(), "A.compute", "B.path"), 50);
  EXPECT_EQ(log_->count("B.compute"), 31);
}

/// Events logged after the first occurrence of an event, at most count of them
std::vector<std::string> eventsAfter(
  const std::vector<std::string> & events, const std::string & event, size_t count)
{
  auto it = std::find(events.begin(), events.end(), event);
  if (it == events.end()) {
    return {};
  }
  ++it;
  const auto last = it + std::min<std::ptrdiff_t>(std::distance(it, events.end()), count);
  return {it, last};
}

TEST_F(FollowPathTest, ControllerSwitchLetsPreviousControllerStop)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->scriptCancel({false, false});
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));

  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  // A keeps driving until its cancel() succeeds, then it is reset and B takes over
  const std::vector<std::string> expected{
    "A.cancel", "A.compute", "A.cancel", "A.compute", "A.cancel", "A.reset", "goal.check",
    "B.compute"};
  EXPECT_EQ(eventsAfter(log_->events(), "B.path", expected.size()), expected);
}

TEST_F(FollowPathTest, ControllerSwitchIsImmediateByDefault)
{
  startServer({});

  controller_a_->script({}, false);
  controller_a_->cancel_result = false;
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));

  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_EQ(
    eventsAfter(log_->events(), "B.path", 2),
    (std::vector<std::string>{"goal.check", "B.compute"}));
  EXPECT_EQ(log_->count("A.cancel"), 0);
}

TEST_F(FollowPathTest, CancelDuringControllerSwitchWaitsForPreviousController)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->cancel_result = false;
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));
  const auto goal_b = sendGoal("B");
  ASSERT_NE(goal_b, nullptr);
  ASSERT_TRUE(waitFor("A.cancel", 2));

  auto cancel_future = client_->async_cancel_goal(goal_b);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, cancel_future, std::chrono::seconds(10)),
    rclcpp::FutureReturnCode::SUCCESS);
  // While A refuses to stop, the goal stays active and A keeps driving
  const int64_t computed = log_->count("A.compute");
  ASSERT_TRUE(waitFor("A.cancel", log_->count("A.cancel") + 10));
  EXPECT_GE(log_->count("A.compute"), computed + 9);
  auto result_future = client_->async_get_result(goal_b);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      client_node_, result_future, std::chrono::milliseconds(200)),
    rclcpp::FutureReturnCode::TIMEOUT);

  controller_a_->cancel_result = true;
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, result_future, std::chrono::seconds(10)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, ResultCode::CANCELED);
  EXPECT_EQ(log_->count("B.compute"), 0);
}

TEST_F(FollowPathTest, ControllerSwitchSkipsGoalCheckWhileStopping)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  // While stopping, A reports the goal reached
  controller_a_->script({}, false);
  controller_a_->scriptCancel({false, false});
  controller_a_->scriptOnCancel({}, true);
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));

  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  // The goal is checked only once A is stopped
  const std::vector<std::string> expected{
    "A.cancel", "A.compute", "A.cancel", "A.compute", "A.cancel", "A.reset", "goal.check"};
  EXPECT_EQ(eventsAfter(log_->events(), "B.path", expected.size()), expected);
  EXPECT_EQ(log_->count("B.compute"), 0);
}

TEST_F(FollowPathTest, StoppingControllerFailureSwitchesController)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->scriptCancel({false});
  controller_a_->scriptOnCancel({blocked()}, false);
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));

  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  const std::vector<std::string> expected{
    "A.cancel", "A.compute", "A.reset", "goal.check", "B.compute"};
  EXPECT_EQ(eventsAfter(log_->events(), "B.path", expected.size()), expected);
}

TEST_F(FollowPathTest, StoppingControllerErrorSwitchesController)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->scriptCancel({false});
  controller_a_->scriptOnCancel({fail<std::runtime_error>("plugin bug")}, false);
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));

  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  const std::vector<std::string> expected{
    "A.cancel", "A.compute", "A.reset", "goal.check", "B.compute"};
  EXPECT_EQ(eventsAfter(log_->events(), "B.path", expected.size()), expected);
}

TEST_F(FollowPathTest, ControllerSwitchResetsFailureCountOfStoppingController)
{
  // 20 Hz leaves A 15 cycles (0.75 s) of retries to receive the preemption
  startServer({
    rclcpp::Parameter("stop_on_controller_switch", true),
    rclcpp::Parameter("controller_frequency", 20.0),
    rclcpp::Parameter("retry_all_failures", true),
    rclcpp::Parameter("max_retries", 50),
    rclcpp::Parameter("failure_tolerance", -1.0)});

  // A fails 30 times while stopping
  controller_a_->script(std::vector<std::exception_ptr>(1000, blocked()));
  controller_a_->scriptCancel(std::vector<bool>(30, false));
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 35));

  // B gets its own max_retries, whatever A failed before
  controller_b_->script(std::vector<std::exception_ptr>(30, blocked()));
  const auto result = followPath("B");
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_LE(countBefore(log_->events(), "A.compute", "B.path"), 50);
  EXPECT_EQ(log_->count("A.cancel"), 31);
  EXPECT_EQ(log_->count("B.compute"), 31);
}

TEST_F(FollowPathTest, ControllerSwitchPreemptedByThirdController)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->cancel_result = false;
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));
  ASSERT_NE(sendGoal("B"), nullptr);
  ASSERT_TRUE(waitFor("A.cancel", 2));

  const auto goal_c = sendGoal("C");
  ASSERT_NE(goal_c, nullptr);
  ASSERT_TRUE(waitFor("C.path", 1));
  ASSERT_TRUE(waitFor("A.cancel", log_->count("A.cancel") + 2));
  controller_a_->cancel_result = true;

  EXPECT_EQ(result(goal_c).code, ResultCode::SUCCEEDED);
  // A still stops after the second preemption, then C takes over
  EXPECT_EQ(
    eventsAfter(log_->events(), "C.path", 2), (std::vector<std::string>{"A.cancel", "A.compute"}));
  EXPECT_EQ(
    eventsAfter(log_->events(), "A.reset", 2),
    (std::vector<std::string>{"goal.check", "C.compute"}));
  EXPECT_EQ(log_->count("B.compute"), 0);
}

TEST_F(FollowPathTest, ControllerSwitchBackToStoppingController)
{
  startServer({rclcpp::Parameter("stop_on_controller_switch", true)});

  controller_a_->script({}, false);
  controller_a_->cancel_result = false;
  ASSERT_NE(sendGoal("A"), nullptr);
  ASSERT_TRUE(waitFor("A.compute", 3));
  ASSERT_NE(sendGoal("B"), nullptr);
  ASSERT_TRUE(waitFor("A.cancel", 2));

  // A reports the goal reached while stopping, which is checked only once A is stopped
  controller_a_->script({}, true);
  const auto goal_a = sendGoal("A");
  ASSERT_NE(goal_a, nullptr);
  ASSERT_TRUE(waitFor("A.path", 2));
  ASSERT_TRUE(waitFor("A.cancel", log_->count("A.cancel") + 2));
  controller_a_->cancel_result = true;

  EXPECT_EQ(result(goal_a).code, ResultCode::SUCCEEDED);
  // A is reset once stopped and gets the plan again
  EXPECT_EQ(
    eventsAfter(log_->events(), "A.reset", 2),
    (std::vector<std::string>{"A.path", "goal.check"}));
  EXPECT_EQ(log_->count("B.compute"), 0);
}

TEST(ControllerServerTest, TransformedPlanAndGoalThrowsOnTfFailure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("filters", std::vector<std::string>{}),
  });

  auto server = std::make_shared<ControllerServerShim>(options);
  ASSERT_EQ(
    server->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = server->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.rotation.w = 1.0;
  server->getTfBuffer().setTransform(tf_msg, "test", true);

  server->setEndPoseFrame("nonexistent_frame_xyz");
  EXPECT_THROW(server->callTransformedPlanAndGoal(), nav2_core::ControllerTFError);
  server->cleanup();
}

/// Server configured without plugins, used to apply FollowPath goal tolerances
std::shared_ptr<ControllerServerShim> makeConfiguredServer()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("progress_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("goal_checker_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("controller_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("path_handler_plugins", std::vector<std::string>{}),
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("filters", std::vector<std::string>{}),
  });
  auto server = std::make_shared<ControllerServerShim>(options);
  EXPECT_EQ(
    server->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  return server;
}

TEST(ControllerServerTest, GoalTolerancesRejectedByGoalChecker)
{
  auto server = makeConfiguredServer();
  server->selectGoalChecker(
    "position_checker", std::make_shared<nav2_controller::PositionGoalChecker>());

  ControllerServerShim::Action::Goal goal;
  goal.tolerance_from_action = true;
  goal.dist_tolerance = 0.1;
  goal.angle_tolerance = 0.2;
  try {
    server->callApplyGoalTolerances(goal);
    FAIL() << "Goal tolerances were applied to a goal checker that does not take them";
  } catch (const nav2_core::ControllerException & ex) {
    EXPECT_EQ(ex.errorCode(), ControllerServerShim::Action::Result::INVALID_CONTROLLER);
  }

  // Without tolerance_from_action the goal checker keeps its configured tolerances
  goal.tolerance_from_action = false;
  EXPECT_NO_THROW(server->callApplyGoalTolerances(goal));
  server->cleanup();
}

TEST(ControllerServerTest, GoalTolerancesAppliedToGoalChecker)
{
  auto server = makeConfiguredServer();
  auto checker = std::make_shared<nav2_controller::SimpleGoalChecker>();
  server->selectGoalChecker("simple_checker", checker);

  ControllerServerShim::Action::Goal goal;
  goal.tolerance_from_action = true;
  goal.dist_tolerance = 0.1;
  goal.angle_tolerance = 0.2;
  EXPECT_NO_THROW(server->callApplyGoalTolerances(goal));

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  double path_length_tolerance;
  ASSERT_TRUE(checker->getTolerances(pose_tolerance, vel_tolerance, path_length_tolerance));
  EXPECT_NEAR(pose_tolerance.position.x, 0.1, 1e-6);
  EXPECT_NEAR(
    2.0 * std::atan2(pose_tolerance.orientation.z, pose_tolerance.orientation.w), 0.2, 1e-6);
  server->cleanup();
}

TEST(ControllerServerTest, GoalCheckerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_goal_checker"};
  node->declare_parameter("goal_checker_plugins", invalid_ids);
  node->declare_parameter("invalid_goal_checker.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, ProgressCheckerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_progress_checker"};
  node->declare_parameter("progress_checker_plugins", invalid_ids);
  node->declare_parameter("invalid_progress_checker.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, ControllerPluginTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_controller"};
  node->declare_parameter("controller_plugins", invalid_ids);
  node->declare_parameter("invalid_controller.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, PathHandlerTypeException)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::vector<std::string> invalid_ids = {"invalid_path_handler"};
  node->declare_parameter("path_handler_plugins", invalid_ids);
  node->declare_parameter("invalid_path_handler.plugin", rclcpp::PARAMETER_STRING);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, InvalidMaxRetriesRejected)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test_node");
  node->declare_parameter("max_retries", -2);
  EXPECT_THROW(std::make_unique<nav2_controller::ParameterHandler>(node, node->get_logger()),
    std::runtime_error);
}

TEST(ControllerServerTest, test_dynamic_parameters)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  auto param_handler_ = std::make_unique<nav2_controller::ParameterHandler>(
      node, node->get_logger());
  param_handler_->activate();
  auto params_ = param_handler_->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("min_x_velocity_threshold", 100.0),
      rclcpp::Parameter("min_y_velocity_threshold", 100.0),
      rclcpp::Parameter("min_theta_velocity_threshold", 100.0),
      rclcpp::Parameter("failure_tolerance", 5.0),
      rclcpp::Parameter("search_window", 10.0),
      rclcpp::Parameter("retry_all_failures", true),
      rclcpp::Parameter("max_retries", 3),
      rclcpp::Parameter("stop_on_controller_switch", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->min_x_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_y_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_theta_velocity_threshold, 100.0);
  EXPECT_EQ(params_->failure_tolerance, 5.0);
  EXPECT_EQ(params_->search_window, 10.0);
  EXPECT_TRUE(params_->retry_all_failures);
  EXPECT_EQ(params_->max_retries, 3);
  EXPECT_TRUE(params_->stop_on_controller_switch);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("min_x_velocity_threshold", -1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->min_x_velocity_threshold, 100.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("search_window", -0.1)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->search_window, 10.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("max_retries", -2)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->max_retries, 3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
