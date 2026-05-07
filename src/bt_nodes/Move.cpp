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
#include <iostream>
#include <vector>
#include <memory>

#include "plan_bookstore/Move.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace plan_bookstore
{

Move::Move(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  if (!config().blackboard->get("node", node_)) {
    throw std::runtime_error("Move: failed to get 'node' from blackboard");
  }

  try {
    node_->declare_parameter<bool>("fake_navigation", false);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
  node_->get_parameter_or("fake_navigation", fake_navigation_, false);

  if (fake_navigation_) {
    RCLCPP_INFO(node_->get_logger(), "Move: FAKE navigation mode (no EasyNav)");
  }

  try {
    node_->declare_parameter<std::vector<std::string>>("waypoints");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

  if (node_->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;
    node_->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      try {
        node_->declare_parameter<std::vector<double>>("waypoint_coords." + wp);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}

      std::vector<double> coords;
      if (node_->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        waypoints_[wp] = Pose2D{coords[0], coords[1], coords[2]};
      } else {
        RCLCPP_ERROR(node_->get_logger(), "No coordinate for waypoint [%s]", wp.c_str());
      }
    }
  }

#ifdef HAS_EASYNAV
  if (!fake_navigation_) {
    client_id_ = std::string(node_->get_name()) + "_move_client";
    rclcpp::QoS qos(100);

    control_pub_ = node_->create_publisher<NavigationControl>("easynav_control", qos);
    control_sub_ = node_->create_subscription<NavigationControl>(
      "easynav_control", qos,
      std::bind(&Move::on_control_msg, this, std::placeholders::_1));
  }
#else
  if (!fake_navigation_) {
    RCLCPP_WARN(node_->get_logger(),
      "Move: built without easynav_interfaces; forcing fake_navigation=true");
    fake_navigation_ = true;
  }
#endif

  perception_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/perception_events", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      perceptions_during_move_.push_back(msg->data);
    });
}

#ifdef HAS_EASYNAV
void
Move::on_control_msg(NavigationControl::UniquePtr msg)
{
  if (msg->user_id == client_id_) {return;}
  if (msg->nav_current_user_id != client_id_) {return;}

  switch (nav_state_) {
    case NavState::GOAL_SENT:
      if (msg->type == NavigationControl::ACCEPT) {
        RCLCPP_INFO(node_->get_logger(), "EasyNav accepted goal for %s",
          current_goal_name_.c_str());
        nav_state_ = NavState::NAVIGATING;
      } else if (msg->type == NavigationControl::REJECT ||
                 msg->type == NavigationControl::ERROR) {
        RCLCPP_ERROR(node_->get_logger(), "EasyNav rejected goal: %s",
          msg->status_message.c_str());
        nav_state_ = NavState::FAILED;
      }
      break;

    case NavState::NAVIGATING:
      if (msg->type == NavigationControl::FINISHED) {
        RCLCPP_INFO(node_->get_logger(), "EasyNav navigation finished for %s",
          current_goal_name_.c_str());
        nav_state_ = NavState::FINISHED;
      } else if (msg->type == NavigationControl::FAILED ||
                 msg->type == NavigationControl::CANCELLED ||
                 msg->type == NavigationControl::ERROR) {
        RCLCPP_ERROR(node_->get_logger(), "EasyNav navigation failed: %s",
          msg->status_message.c_str());
        nav_state_ = NavState::FAILED;
      }
      break;

    default:
      break;
  }
}
#endif

BT::NodeStatus
Move::tick()
{
  if (fake_navigation_) {
    if (fake_tick_count_ == 0) {
      std::string goal;
      getInput<std::string>("goal", goal);
      current_goal_name_ = goal;
      RCLCPP_INFO(node_->get_logger(), "[FAKE] Move to %s started", goal.c_str());
      config().blackboard->set("out_msg",
        std::string("Navigating to " + goal + " (fake)."));
    }
    fake_tick_count_++;
    if (fake_tick_count_ >= FAKE_TICKS_TO_FINISH) {
      std::string out = "Arrived at " + current_goal_name_ + " (fake).";
      if (!perceptions_during_move_.empty()) {
        out += " Perceptions during move:";
        for (const auto & p : perceptions_during_move_) {
          out += " [" + p + "]";
        }
      }
      config().blackboard->set("out_msg", out);
      RCLCPP_INFO(node_->get_logger(), "[FAKE] Move to %s finished",
        current_goal_name_.c_str());
      fake_tick_count_ = 0;
      perceptions_during_move_.clear();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

#ifdef HAS_EASYNAV
  switch (nav_state_) {
    case NavState::IDLE:
    {
      std::string goal;
      getInput<std::string>("goal", goal);

      auto it = waypoints_.find(goal);
      if (it == waypoints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "No coordinate for waypoint [%s]", goal.c_str());
        return BT::NodeStatus::FAILURE;
      }

      const auto & pose2d = it->second;
      current_goal_name_ = goal;

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.header.stamp = node_->now();
      goal_pose.pose.position.x = pose2d.x;
      goal_pose.pose.position.y = pose2d.y;
      goal_pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, pose2d.theta);
      goal_pose.pose.orientation = tf2::toMsg(q);

      NavigationControl msg;
      msg.type = NavigationControl::REQUEST;
      msg.header.frame_id = "map";
      msg.header.stamp = node_->now();
      msg.user_id = client_id_;
      msg.seq = ++seq_;

      nav_msgs::msg::Goals goals;
      goals.header = msg.header;
      goals.goals.push_back(goal_pose);
      msg.goals = goals;

      control_pub_->publish(msg);
      nav_state_ = NavState::GOAL_SENT;

      RCLCPP_INFO(node_->get_logger(), "Move goal sent: %s (%.2f, %.2f)",
        goal.c_str(), pose2d.x, pose2d.y);

      config().blackboard->set("out_msg",
        std::string("Navigating to " + current_goal_name_ + "."));
      return BT::NodeStatus::RUNNING;
    }

    case NavState::GOAL_SENT:
      config().blackboard->set("out_msg",
        std::string("Waiting for navigation to " + current_goal_name_ + "."));
      return BT::NodeStatus::RUNNING;

    case NavState::NAVIGATING:
      config().blackboard->set("out_msg",
        std::string("Navigating to " + current_goal_name_ + "."));
      return BT::NodeStatus::RUNNING;

    case NavState::FINISHED:
    {
      std::string out = "Arrived at " + current_goal_name_ + ".";
      if (!perceptions_during_move_.empty()) {
        out += " Perceptions during move:";
        for (const auto & p : perceptions_during_move_) {
          out += " [" + p + "]";
        }
      }
      config().blackboard->set("out_msg", out);
      nav_state_ = NavState::IDLE;
      perceptions_during_move_.clear();
      return BT::NodeStatus::SUCCESS;
    }

    case NavState::FAILED:
      nav_state_ = NavState::IDLE;
      return BT::NodeStatus::FAILURE;
  }
#endif

  return BT::NodeStatus::FAILURE;
}

void
Move::halt()
{
#ifdef HAS_EASYNAV
  if (nav_state_ == NavState::NAVIGATING || nav_state_ == NavState::GOAL_SENT) {
    NavigationControl msg;
    msg.type = NavigationControl::CANCEL;
    msg.header.stamp = node_->now();
    msg.user_id = client_id_;
    msg.seq = ++seq_;
    control_pub_->publish(msg);

    RCLCPP_INFO(node_->get_logger(), "Move halted — sent cancel to EasyNav");
  }
#endif

  nav_state_ = NavState::IDLE;
  fake_tick_count_ = 0;
}

}  // namespace plan_bookstore

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan_bookstore::Move>("Move");
}
