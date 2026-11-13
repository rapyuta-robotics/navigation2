// Copyright (c) 2020 Samsung Research
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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"


TEST(LifecylceTest, CheckInitialTfTimeout) {
  rclcpp::init(0, nullptr);

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions());
  costmap->declare_parameter("initial_transform_timeout", 0.0);
  costmap->set_parameter({"initial_transform_timeout", 0.0});

  std::thread spin_thread{[costmap]() {rclcpp::spin(costmap->get_node_base_interface());}};

  {
    const auto state_after_configure = costmap->configure();
    ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    // Without providing the transform from global to robot base the activation should fail
    // and the costmap should transition into the inactive state.
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  // Set a dummy transform from global to robot base
  geometry_msgs::msg::TransformStamped transform_global_to_robot{};
  transform_global_to_robot.header.frame_id = costmap->getGlobalFrameID();
  transform_global_to_robot.child_frame_id = costmap->getBaseFrameID();
  costmap->getTfBuffer()->setTransform(transform_global_to_robot, "test", true);
  // Now the costmap should successful transition into the active state
  {
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

TEST(LifecylceTest, RejectInvalidCostmapSize) {
  rclcpp::init(0, nullptr);

  const auto expect_failure = [](const std::vector<rclcpp::Parameter> & parameters) {
      rclcpp::NodeOptions options;
      options.parameter_overrides(parameters);
      auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
      EXPECT_EQ(
        costmap->on_configure(rclcpp_lifecycle::State()),
        nav2::CallbackReturn::FAILURE);
    };

  expect_failure({rclcpp::Parameter("width", -1)});
  expect_failure({rclcpp::Parameter("height", -1)});
  expect_failure({rclcpp::Parameter("resolution", 0.0)});
  expect_failure(
  {
    rclcpp::Parameter("width", std::numeric_limits<int>::max()),
    rclcpp::Parameter("height", std::numeric_limits<int>::max()),
    rclcpp::Parameter("resolution", 0.01),
  });

  rclcpp::shutdown();
}

TEST(LifecylceTest, FootprintPaddingType) {
  rclcpp::init(0, nullptr);

  const auto make_costmap = [](const std::string & padding_type) {
      rclcpp::NodeOptions options;
      options.parameter_overrides(
    {
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("footprint", "[[0.2, -0.3], [1.0, -0.3], [1.0, 0.3], [0.2, 0.3]]"),
      rclcpp::Parameter("footprint_padding", 0.1),
      rclcpp::Parameter("footprint_padding_type", padding_type),
      });
      return std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    };

  auto rejected = make_costmap("round");
  EXPECT_EQ(rejected->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::FAILURE);

  auto costmap = make_costmap("polygon_buffer");
  ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
  // the footprint grows by the padding on every side, also behind the robot center
  double min_x = 1.0;
  for (const auto & point : costmap->getRobotFootprint()) {
    min_x = std::min(min_x, point.x);
  }
  EXPECT_NEAR(0.1, min_x, 1e-6);
  EXPECT_EQ(4u, costmap->getUnpaddedRobotFootprint().size());
  EXPECT_NEAR(0.2, costmap->getUnpaddedRobotFootprint()[0].x, 1e-6);

  // Shut down before the costmap is destroyed: configure started its executor thread, and
  // cancelling that thread before it starts spinning would make the join wait forever
  rclcpp::shutdown();
}
