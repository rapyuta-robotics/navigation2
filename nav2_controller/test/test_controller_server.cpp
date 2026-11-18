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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/node_options.hpp"
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
};

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
      rclcpp::Parameter("search_window", 10.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->min_x_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_y_velocity_threshold, 100.0);
  EXPECT_EQ(params_->min_theta_velocity_threshold, 100.0);
  EXPECT_EQ(params_->failure_tolerance, 5.0);
  EXPECT_EQ(params_->search_window, 10.0);

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
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
