// Copyright 2026 Álvaro Valencia
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

#ifndef PLAN_BOOKSTORE__MOVE_HPP_
#define PLAN_BOOKSTORE__MOVE_HPP_

#include <string>
#include <map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#ifdef HAS_EASYNAV
#include "easynav_interfaces/msg/navigation_control.hpp"
#endif
#include "nav_msgs/msg/goals.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plan_bookstore
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

class Move : public BT::ActionNodeBase
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal")
    };
  }

private:
  enum class NavState { IDLE, GOAL_SENT, NAVIGATING, FINISHED, FAILED };
  NavState nav_state_ = NavState::IDLE;

#ifdef HAS_EASYNAV
  using NavigationControl = easynav_interfaces::msg::NavigationControl;
  rclcpp::Publisher<NavigationControl>::SharedPtr control_pub_;
  rclcpp::Subscription<NavigationControl>::SharedPtr control_sub_;
  std::string client_id_;
  int64_t seq_ = 0;

  void on_control_msg(NavigationControl::UniquePtr msg);
#endif

  std::map<std::string, Pose2D> waypoints_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  std::vector<std::string> perceptions_during_move_;
  std::string current_goal_name_;

  bool fake_navigation_ = false;
  int fake_tick_count_ = 0;
  static constexpr int FAKE_TICKS_TO_FINISH = 15;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace plan_bookstore

#endif  // PLAN_BOOKSTORE__MOVE_HPP_
