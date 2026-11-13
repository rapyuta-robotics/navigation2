// Copyright (c) 2026 Rapyuta Robotics
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
// limitations under the License.

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

namespace
{

/**
 * @brief Lifecycle services of a node whose change_state responses are lost: the node makes the
 * transition (or not, if refuse_transitions) but never answers.
 */
class LostResponseNode
{
public:
  LostResponseNode(const std::string & name, bool refuse_transitions)
  : node_(std::make_shared<rclcpp::Node>(name)), refuse_transitions_(refuse_transitions)
  {
    change_state_ = node_->create_service<lifecycle_msgs::srv::ChangeState>(
      name + "/change_state",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request) {
        // deferred service: no response is ever sent
        if (refuse_transitions_) {
          return;
        }
        if (request->transition.id == Transition::TRANSITION_CONFIGURE) {
          state_ = State::PRIMARY_STATE_INACTIVE;
        } else if (request->transition.id == Transition::TRANSITION_ACTIVATE) {
          state_ = State::PRIMARY_STATE_ACTIVE;
        }
      });
    get_state_ = node_->create_service<lifecycle_msgs::srv::GetState>(
      name + "/get_state",
      [this](
        const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
        std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        response->current_state.id = state_;
      });
  }

  rclcpp::Node::SharedPtr node() const {return node_;}
  uint8_t state() const {return state_;}

private:
  rclcpp::Node::SharedPtr node_;
  bool refuse_transitions_;
  std::atomic<uint8_t> state_{State::PRIMARY_STATE_UNCONFIGURED};
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_;
};

class TransitionTimeoutTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  /** @brief Starts a lifecycle manager for the given node and returns the startup result. */
  std::future<bool> startup(const std::string & managed_node, double transition_timeout)
  {
    managed_ = std::make_shared<LostResponseNode>(managed_node, refuse_transitions_);
    manager_ = std::make_shared<nav2_lifecycle_manager::LifecycleManager>(
      rclcpp::NodeOptions().parameter_overrides(
      {
        {"node_names", std::vector<std::string>{managed_node}},
        {"autostart", false},
        {"bond_timeout", 0.0},
        {"service_timeout", 1.0},
        {"transition_timeout", transition_timeout},
    }));
    client_node_ = std::make_shared<rclcpp::Node>("transition_timeout_test_client");
    executor_.add_node(managed_->node());
    executor_.add_node(manager_->get_node_base_interface());
    spinner_ = std::thread([this]() {executor_.spin();});

    auto client =
      client_node_->create_client<ManageLifecycleNodes>("lifecycle_manager/manage_nodes");
    EXPECT_TRUE(client->wait_for_service(5s));
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = ManageLifecycleNodes::Request::STARTUP;
    return std::async(
      std::launch::async, [this, client, request]() {
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(client_node_, future, 15s) !=
        rclcpp::FutureReturnCode::SUCCESS)
        {
          return false;
        }
        return static_cast<bool>(future.get()->success);
      });
  }

  bool refuse_transitions_ = false;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::shared_ptr<LostResponseNode> managed_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManager> manager_;
  rclcpp::Node::SharedPtr client_node_;
};

TEST_F(TransitionTimeoutTest, LostResponsesDoNotBlockStartup)
{
  auto result = startup("lost_response_node", 1.0);
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_TRUE(result.get());
  EXPECT_EQ(managed_->state(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TransitionTimeoutTest, LostResponseWithoutTransitionFailsStartup)
{
  refuse_transitions_ = true;
  auto result = startup("refusing_node", 1.0);
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_EQ(managed_->state(), State::PRIMARY_STATE_UNCONFIGURED);
}

}  // namespace
