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

#include <string>

#include "plan_bookstore/CheckBookPresent.hpp"

#include <nlohmann/json.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plan_bookstore
{

CheckBookPresent::CheckBookPresent(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw std::runtime_error("CheckBookPresent: failed to get 'node' from blackboard");
  }

  try {
    node_->declare_parameter<std::string>("displaced_book", "");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try {
    node_->declare_parameter<bool>("fake_check", false);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try {
    node_->declare_parameter<double>("perception_lookback_sec", lookback_sec_);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  try {
    node_->declare_parameter<double>("perception_settle_sec", settle_sec_);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  node_->get_parameter("displaced_book", displaced_book_);
  node_->get_parameter_or("fake_check", fake_check_, false);
  node_->get_parameter("perception_lookback_sec", lookback_sec_);
  node_->get_parameter("perception_settle_sec", settle_sec_);

  auto sep = displaced_book_.find('_');
  if (sep != std::string::npos) {
    displaced_from_ = "shelf_" + displaced_book_.substr(0, sep);
  }

  if (!fake_check_) {
    perception_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/perception_events", 50,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        events_.push_back(TimedEvent{node_->now(), msg->data});
        const auto cutoff =
          node_->now() - rclcpp::Duration::from_seconds(2.0 * lookback_sec_);
        while (!events_.empty() && events_.front().stamp < cutoff) {
          events_.erase(events_.begin());
        }
      });
  }
}

void CheckBookPresent::halt()
{
  ticking_ = false;
}

BT::NodeStatus CheckBookPresent::tick()
{
  std::string book_name;
  std::string location;
  (void)config().blackboard->get("arg1", book_name);
  (void)config().blackboard->get("arg2", location);

  if (fake_check_) {
    if (book_name == displaced_book_ && location == displaced_from_) {
      const std::string msg = "CheckBookPresent: " + book_name +
        " NOT FOUND at " + location + ". Book is missing.";
      config().blackboard->set("out_msg", msg);
      RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const std::string msg = "CheckBookPresent: " + book_name + " found at " + location + ".";
    config().blackboard->set("out_msg", msg);
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (!ticking_) {
    start_time_ = node_->now();
    ticking_ = true;
  }

  std::string color;
  auto sep = book_name.find('_');
  if (sep != std::string::npos) {
    color = book_name.substr(0, sep);
  }

  const auto window_start = start_time_ - rclcpp::Duration::from_seconds(lookback_sec_);
  for (const auto & ev : events_) {
    if (ev.stamp < window_start) {continue;}
    try {
      auto j = nlohmann::json::parse(ev.data);
      if (j.value("observation", "") != "object_detected") {continue;}
      if (j.value("object", "") != "book") {continue;}
      if (!j.contains("color") || j["color"].is_null()) {continue;}
      if (j.value("color", "") != color) {continue;}
    } catch (const nlohmann::json::exception &) {
      continue;
    }

    ticking_ = false;
    const std::string msg = "CheckBookPresent: " + book_name +
      " seen at " + location + " (perception color match).";
    config().blackboard->set("out_msg", msg);
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() < settle_sec_) {
    return BT::NodeStatus::RUNNING;
  }

  ticking_ = false;
  const std::string msg = "CheckBookPresent: " + book_name +
    " NOT seen at " + location +
    " (no matching perception in last " + std::to_string(lookback_sec_) + "s).";
  config().blackboard->set("out_msg", msg);
  RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace plan_bookstore

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan_bookstore::CheckBookPresent>("CheckBookPresent");
}
