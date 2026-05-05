// Copyright 2026 Intelligent Robotics Lab
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

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

#include "cv_bridge/cv_bridge.hpp"

class PerceptionYoloNode : public rclcpp::Node
{
public:
  PerceptionYoloNode()
  : Node("perception_yolo_node")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/perception_events", 10);

    declare_parameter<double>("color_cooldown", 2.0);
    declare_parameter<double>("min_confidence", 0.4);
    declare_parameter<int>("min_saturation", 60);
    declare_parameter<int>("min_value", 60);
    declare_parameter<double>("min_color_fraction", 0.05);
    declare_parameter<std::string>("displaced_book", "");
    declare_parameter<std::string>("displaced_location", "");
    declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"book"});

    cooldown_sec_ = get_parameter("color_cooldown").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();
    min_saturation_ = get_parameter("min_saturation").as_int();
    min_value_ = get_parameter("min_value").as_int();
    min_color_fraction_ = get_parameter("min_color_fraction").as_double();
    displaced_color_ = strip_book_suffix(get_parameter("displaced_book").as_string());
    displaced_location_ = get_parameter("displaced_location").as_string();
    target_classes_ = get_parameter("target_classes").as_string_array();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&PerceptionYoloNode::on_image, this, std::placeholders::_1));

    detection_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/detections", 10,
      std::bind(&PerceptionYoloNode::on_detections, this, std::placeholders::_1));

    std::string classes_csv;
    for (size_t i = 0; i < target_classes_.size(); ++i) {
      classes_csv += (i ? "," : "") + target_classes_[i];
    }
    RCLCPP_INFO(get_logger(),
      "perception_yolo_node ready (target_classes=[%s], cooldown %.1fs, "
      "min_conf %.2f, min_sat %d, min_val %d, min_color_frac %.2f, "
      "displaced %s -> %s)",
      classes_csv.c_str(),
      cooldown_sec_, min_confidence_, min_saturation_, min_value_, min_color_fraction_,
      displaced_color_.empty() ? "none" : displaced_color_.c_str(),
      displaced_location_.empty() ? "none" : displaced_location_.c_str());
  }

private:
  // OpenCV HSV: H ∈ [0, 179]. Red wraps so it has two ranges.
  struct HueRange { const char * name; int h_lo; int h_hi; };
  static constexpr HueRange HUE_RANGES[] = {
    {"red",     0,  10},
    {"red",   165, 179},
    {"yellow", 20,  35},
    {"green",  40,  85},
    {"blue",  100, 130},
  };

  // "blue_book" -> "blue"; anything not ending in "_book" returns empty.
  static std::string strip_book_suffix(const std::string & full)
  {
    const std::string suffix = "_book";
    if (full.size() > suffix.size() &&
      full.compare(full.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return full.substr(0, full.size() - suffix.size());
    }
    return {};
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(image_mutex_);
    last_image_ = msg;
  }

  void on_detections(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    sensor_msgs::msg::Image::SharedPtr image;
    {
      std::lock_guard<std::mutex> lk(image_mutex_);
      image = last_image_;
    }
    if (!image) {return;}

    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(image, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    auto now = this->now();
    for (const auto & det : msg->detections) {
      if (det.score < min_confidence_) {continue;}
      if (std::find(target_classes_.begin(), target_classes_.end(), det.class_name) ==
        target_classes_.end())
      {
        continue;
      }

      std::string color = classify_color(bgr, det.bbox);
      if (color.empty()) {continue;}

      const std::string key = det.class_name + ":" + color;
      auto it = last_emit_.find(key);
      if (it != last_emit_.end() && (now - it->second).seconds() < cooldown_sec_) {
        continue;
      }
      last_emit_[key] = now;

      const bool location_known =
        !color.empty() && color == displaced_color_ && !displaced_location_.empty();

      nlohmann::ordered_json event = {
        {"observation", "object_detected"},
        {"object", det.class_name},
        {"color",
          color.empty() ? nlohmann::ordered_json(nullptr) : nlohmann::ordered_json(color)},
        {"location",
          location_known ? nlohmann::ordered_json(displaced_location_) :
                           nlohmann::ordered_json(nullptr)},
        {"confidence", det.score},
        {"detail",
          (color.empty() ? det.class_name : color + " " + det.class_name) +
          " detected by camera"},
      };

      std_msgs::msg::String out;
      out.data = event.dump();
      publisher_->publish(out);

      RCLCPP_INFO(get_logger(), "[YOLO] %s%s%s (conf %.2f)",
        color.c_str(), color.empty() ? "" : " ",
        det.class_name.c_str(), det.score);
    }
  }

  // Returns the dominant color name in the bbox crop, or "" when no color
  // bucket reaches min_color_fraction_ of the crop area.
  std::string classify_color(
    const cv::Mat & bgr, const yolo_msgs::msg::BoundingBox2D & bbox) const
  {
    int cx = static_cast<int>(bbox.center.position.x);
    int cy = static_cast<int>(bbox.center.position.y);
    int w = static_cast<int>(bbox.size.x);
    int h = static_cast<int>(bbox.size.y);
    int x = std::max(0, cx - w / 2);
    int y = std::max(0, cy - h / 2);
    w = std::min(w, bgr.cols - x);
    h = std::min(h, bgr.rows - y);
    if (w <= 0 || h <= 0) {return {};}

    cv::Mat hsv;
    cv::cvtColor(bgr(cv::Rect(x, y, w, h)), hsv, cv::COLOR_BGR2HSV);

    std::map<std::string, long> votes;
    for (const auto & range : HUE_RANGES) {
      cv::Mat mask;
      cv::inRange(
        hsv,
        cv::Scalar(range.h_lo, min_saturation_, min_value_),
        cv::Scalar(range.h_hi, 255, 255),
        mask);
      votes[range.name] += cv::countNonZero(mask);
    }

    std::string winner;
    long best = 0;
    for (const auto & [c, v] : votes) {
      if (v > best) {best = v; winner = c;}
    }
    const long crop_area = static_cast<long>(hsv.rows) * hsv.cols;
    if (best < static_cast<long>(crop_area * min_color_fraction_)) {return {};}
    return winner;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;

  std::mutex image_mutex_;
  sensor_msgs::msg::Image::SharedPtr last_image_;

  std::map<std::string, rclcpp::Time> last_emit_;

  double cooldown_sec_;
  double min_confidence_;
  int min_saturation_;
  int min_value_;
  double min_color_fraction_;

  std::string displaced_color_;
  std::string displaced_location_;
  std::vector<std::string> target_classes_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionYoloNode>());
  rclcpp::shutdown();
  return 0;
}
