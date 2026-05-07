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

#include <chrono>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

class PerceptionSimNode : public rclcpp::Node
{
public:
  PerceptionSimNode()
  : Node("perception_sim_node"), event_index_(0), robot_moving_(false)
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/perception_events", 10);

    declare_parameter<std::string>("detected_object", "blue_book");
    declare_parameter<std::string>("detected_location", "middle_path");
    declare_parameter<double>("observed_x", -1.5);
    declare_parameter<double>("observed_y", -3.0);

    object_ = get_parameter("detected_object").as_string();
    location_ = get_parameter("detected_location").as_string();
    obs_x_ = get_parameter("observed_x").as_double();
    obs_y_ = get_parameter("observed_y").as_double();

    build_event_sequence();

    action_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
      "/actions_hub", rclcpp::SensorDataQoS().reliable(),
      std::bind(&PerceptionSimNode::action_hub_callback, this, std::placeholders::_1));

    // One event per move — dedup on /actions_hub.
    RCLCPP_INFO(get_logger(),
      "Perception sim ready: %zu events queued "
      "(displaced book: %s at %s). Waiting for move actions...",
      events_.size(), object_.c_str(), location_.c_str());
  }

private:
  // "blue_book" -> "blue"; anything not ending in "_book" returns empty.
  static std::string split_color(const std::string & full)
  {
    const std::string suffix = "_book";
    if (full.size() > suffix.size() &&
      full.compare(full.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return full.substr(0, full.size() - suffix.size());
    }
    return {};
  }

  void build_event_sequence()
  {
    nlohmann::ordered_json e;

    e = {{"observation", "nothing_detected"},
         {"location", "reception"},
         {"detail", "Robot departing reception area. No objects detected."}};
    events_.push_back(e.dump());

    e = {{"observation", "object_detected"},
         {"object", "person"},
         {"color", nullptr},
         {"location", "middle_path"},
         {"confidence", 0.9},
         {"detail", "Person detected walking through middle corridor."}};
    events_.push_back(e.dump());

    // Displaced-book sighting — the signal the LLM solver uses to correct state.
    e = {{"observation", "object_detected"},
         {"object", "book"},
         {"color", split_color(object_)},
         {"location", location_},
         {"confidence", 0.95},
         {"detail", object_ + " detected on the floor at " + location_ + "."}};
    events_.push_back(e.dump());

    e = {{"observation", "nothing_detected"},
         {"location", "shelf_red"},
         {"detail", "Arrived at shelf_red. Shelf contents look normal."}};
    events_.push_back(e.dump());

    e = {{"observation", "nothing_detected"},
         {"location", "deposit_table"},
         {"detail", "Passing deposit table area. No anomalies."}};
    events_.push_back(e.dump());
  }

  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
    if (msg->action != "move") {
      return;
    }

    if (msg->type == plansys2_msgs::msg::ActionExecution::RESPONSE && !robot_moving_) {
      robot_moving_ = true;
      publish_next_event();
    }

    if (msg->type == plansys2_msgs::msg::ActionExecution::FINISH) {
      robot_moving_ = false;
    }
  }

  void publish_next_event()
  {
    if (event_index_ >= events_.size()) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = events_[event_index_];
    publisher_->publish(msg);

    auto j = nlohmann::json::parse(events_[event_index_]);
    std::string obs_type = j.value("observation", "unknown");
    std::string detail = j.value("detail", "");

    RCLCPP_INFO(get_logger(),
      "[PERCEPTION %zu/%zu] %s: %s",
      event_index_ + 1, events_.size(), obs_type.c_str(), detail.c_str());

    event_index_++;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_sub_;

  std::string object_;
  std::string location_;
  double obs_x_;
  double obs_y_;

  std::vector<std::string> events_;
  size_t event_index_;
  bool robot_moving_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionSimNode>());
  rclcpp::shutdown();
  return 0;
}
