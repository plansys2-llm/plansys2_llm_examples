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

#ifndef PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_
#define PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace plan_bookstore
{

class CheckBookPresent : public BT::ActionNodeBase
{
public:
  explicit CheckBookPresent(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  struct TimedEvent
  {
    rclcpp::Time stamp;
    std::string data;
  };

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  std::vector<TimedEvent> events_;

  std::string displaced_book_;
  std::string displaced_from_;

  bool fake_check_ = false;
  // Default settle outlasts perception_yolo_node's 2 s per-key cooldown so a
  // sighting suppressed while the move-action subscription was alive can still
  // be re-emitted before we declare the book missing.
  double lookback_sec_ = 2.0;
  double settle_sec_ = 2.5;

  rclcpp::Time start_time_;
  bool ticking_ = false;
};

}  // namespace plan_bookstore

#endif  // PLAN_BOOKSTORE__CHECKBOOKPRESENT_HPP_
