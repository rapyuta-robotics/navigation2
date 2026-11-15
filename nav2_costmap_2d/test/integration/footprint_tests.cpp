/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dave Hershberger
*********************************************************************/
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_costmap_2d/footprint.hpp"

class FootprintTestNode
{
public:
  FootprintTestNode()
  {
    // Default footprint padding and footprint radius from Costmap2DROS
    testFootprint(0.01f, 0.1);
  }

  ~FootprintTestNode() {}

  void testFootprint(double footprint_padding, std::string footprint)
  {
    footprint_padding_ = footprint_padding;
    if (footprint != "" && footprint != "[]") {
      std::vector<geometry_msgs::msg::Point> new_footprint;
      if (nav2_costmap_2d::makeFootprintFromString(footprint, new_footprint)) {
        setRobotFootprint(new_footprint);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("footprint_tester"), "Invalid footprint string");
      }
    }
  }

  void testFootprint(double footprint_padding, double robot_radius)
  {
    footprint_padding_ = footprint_padding;
    setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius));
  }

  std::vector<geometry_msgs::msg::Point> getRobotFootprint()
  {
    return footprint_;
  }

protected:
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
  {
    footprint_ = points;
    nav2_costmap_2d::padFootprint(footprint_, footprint_padding_);
  }

  double footprint_padding_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    footprint_tester_ = std::make_shared<FootprintTestNode>();
  }

  ~TestNode() {}

protected:
  std::shared_ptr<FootprintTestNode> footprint_tester_;
};

// Start with empty test before updating test footprints
TEST_F(TestNode, footprint_empty)
{
  // FootprintTestNode cm("costmap_footprint_empty", *tf_);
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  // With no specification of footprint or radius,
  // defaults to 0.1 meter radius plus 0.01 meter padding.
  EXPECT_EQ(16u, footprint.size());

  EXPECT_NEAR(0.11f, footprint[0].x, 0.0001);
  EXPECT_NEAR(0.0f, footprint[0].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[0].z);
}

TEST_F(TestNode, unpadded_footprint_from_string_param)
{
  footprint_tester_->testFootprint(0.0, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(1.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.0f, footprint[1].x);
  EXPECT_EQ(1.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.0f, footprint[2].x);
  EXPECT_EQ(-1.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, padded_footprint_from_string_param)
{
  footprint_tester_->testFootprint(0.5, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.5f, footprint[0].x);
  EXPECT_EQ(1.5f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.5f, footprint[1].x);
  EXPECT_EQ(1.5f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.5f, footprint[2].x);
  EXPECT_EQ(-1.5f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, radius_param)
{
  footprint_tester_->testFootprint(0, 10.0);
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  // Circular robot has 16-point footprint auto-generated.
  EXPECT_EQ(16u, footprint.size());

  // Check the first point
  EXPECT_EQ(10.0f, footprint[0].x);
  EXPECT_EQ(0.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  // Check the 4th point, which should be 90 degrees around the circle from the first.
  EXPECT_NEAR(0.0f, footprint[4].x, 0.0001);
  EXPECT_NEAR(10.0f, footprint[4].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[4].z);
}

TEST_F(TestNode, footprint_from_same_level_param)
{
  footprint_tester_->testFootprint(0.0, "[[1, 2], [3, 4], [5, 6]]");
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(2.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(3.0f, footprint[1].x);
  EXPECT_EQ(4.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(5.0f, footprint[2].x);
  EXPECT_EQ(6.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

namespace
{
std::vector<geometry_msgs::msg::Point> makePolygon(
  const std::vector<std::pair<double, double>> & vertices)
{
  std::vector<geometry_msgs::msg::Point> polygon;
  for (const auto & [x, y] : vertices) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    polygon.push_back(point);
  }
  return polygon;
}

struct Bounds
{
  double min_x, max_x, min_y, max_y;
};

Bounds bounds(const std::vector<geometry_msgs::msg::Point> & polygon)
{
  Bounds b{polygon[0].x, polygon[0].x, polygon[0].y, polygon[0].y};
  for (const auto & point : polygon) {
    b.min_x = std::min(b.min_x, point.x);
    b.max_x = std::max(b.max_x, point.x);
    b.min_y = std::min(b.min_y, point.y);
    b.max_y = std::max(b.max_y, point.y);
  }
  return b;
}
}  // namespace

TEST(PolygonBufferPadding, grows_every_side_of_a_footprint_off_the_robot_center)
{
  // footprint entirely in front of the robot center
  const auto footprint = makePolygon({{0.2, -0.3}, {1.0, -0.3}, {1.0, 0.3}, {0.2, 0.3}});

  // per_axis moves the rear side forward instead of outward
  auto per_axis = footprint;
  nav2_costmap_2d::padFootprint(per_axis, 0.1, "per_axis");
  EXPECT_NEAR(0.3, bounds(per_axis).min_x, 1e-9);

  auto buffered = footprint;
  nav2_costmap_2d::padFootprint(buffered, 0.1, "polygon_buffer");
  ASSERT_EQ(4u, buffered.size());
  const auto b = bounds(buffered);
  EXPECT_NEAR(0.1, b.min_x, 1e-9);
  EXPECT_NEAR(1.1, b.max_x, 1e-9);
  EXPECT_NEAR(-0.4, b.min_y, 1e-9);
  EXPECT_NEAR(0.4, b.max_y, 1e-9);
}

TEST(PolygonBufferPadding, keeps_the_padding_distance_on_diagonal_edges)
{
  // per_axis moves each vertex of a diamond along one axis only, so its edges move out by
  // padding / sqrt(2); polygon_buffer moves every edge out by the padding (vertex at
  // 1 + padding * sqrt(2))
  const auto diamond = makePolygon({{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}});

  auto per_axis = diamond;
  nav2_costmap_2d::padFootprint(per_axis, 0.1, "per_axis");
  EXPECT_NEAR(1.1, bounds(per_axis).max_x, 1e-9);

  auto buffered = diamond;
  nav2_costmap_2d::padFootprint(buffered, 0.1, "polygon_buffer");
  ASSERT_EQ(4u, buffered.size());
  const auto b = bounds(buffered);
  EXPECT_NEAR(1.0 + 0.1 * std::sqrt(2.0), b.max_x, 1e-9);
  EXPECT_NEAR(-1.0 - 0.1 * std::sqrt(2.0), b.min_y, 1e-9);
}

TEST(PolygonBufferPadding, leaves_degenerate_footprints_and_zero_padding_unchanged)
{
  const auto segment = makePolygon({{0.0, 0.0}, {1.0, 0.0}});
  auto padded_segment = segment;
  nav2_costmap_2d::padFootprintPolygonBuffer(padded_segment, 0.1);
  EXPECT_EQ(segment, padded_segment);

  const auto square = makePolygon({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}});
  auto unpadded_square = square;
  nav2_costmap_2d::padFootprintPolygonBuffer(unpadded_square, 0.0);
  EXPECT_EQ(square, unpadded_square);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
