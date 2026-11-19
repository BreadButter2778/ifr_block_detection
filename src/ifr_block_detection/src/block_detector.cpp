#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "visualization_msgs/msg/image_marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "foxglove_msgs/msg/image_annotations.hpp"
#include "foxglove_msgs/msg/points_annotation.hpp"
#include "foxglove_msgs/msg/point2.hpp"
#include "foxglove_msgs/msg/text_annotation.hpp"
#include "foxglove_msgs/msg/color.hpp"
#include "ifr_interfaces/msg/block_result.hpp"

// ---------------------------------------------------------------------------
// 一个颜色 = 若干段 HSV 阈值 + 一个显示用的 BGR 颜色
// ---------------------------------------------------------------------------
struct ColorSpec
{
  std::string name;
  std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;
  cv::Scalar bgr;
};

static std_msgs::msg::ColorRGBA to_rgba(const cv::Scalar & bgr, float alpha)
{
  std_msgs::msg::ColorRGBA c;
  c.b = static_cast<float>(bgr[0] / 255.0);
  c.g = static_cast<float>(bgr[1] / 255.0);
  c.r = static_cast<float>(bgr[2] / 255.0);
  c.a = alpha;
  return c;
}

static foxglove_msgs::msg::Color to_fox_color(const cv::Scalar & bgr, double alpha)
{
  foxglove_msgs::msg::Color c;
  c.b = bgr[0] / 255.0;
  c.g = bgr[1] / 255.0;
  c.r = bgr[2] / 255.0;
  c.a = alpha;
  return c;
}

class BlockDetector : public rclcpp::Node
{
public:
  BlockDetector() : Node("block_detector")
  {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/compressed");
    min_area_    = this->declare_parameter<int>("min_area", 3000);

    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    build_specs();

    pub_result_ = this->create_publisher<ifr_interfaces::msg::BlockResult>("/block_result", 10);
    pub_ann_    = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>(
      "/block_annotations", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::ImageMarker>(
      "/block_marker", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, 10,
      std::bind(&BlockDetector::on_image, this, std::placeholders::_1));

    // /target_color: 只画这个颜色的框（空 = 三个都画），由服务端在收到请求时下发
    sub_target_ = this->create_subscription<std_msgs::msg::String>(
      "/target_color", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        target_color_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "[block_detector] 目标颜色切到: %s",
                    target_color_.empty() ? "(全部)" : target_color_.c_str());
      });

    RCLCPP_INFO(this->get_logger(),
                "[block_detector] 订阅 %s, min_area=%d, 颜色数=%zu",
                image_topic_.c_str(), min_area_, specs_.size());
  }

private:
  void build_specs()
  {
    // red 跨越 0/180 => 两段
    specs_.push_back({
      "red",
      {{cv::Scalar(0, 100, 80),   cv::Scalar(10, 255, 255)},
       {cv::Scalar(170, 100, 80), cv::Scalar(180, 255, 255)}},
      cv::Scalar(0, 0, 255)});

    specs_.push_back({
      "blue",
      {{cv::Scalar(90, 100, 60), cv::Scalar(115, 255, 255)}},
      cv::Scalar(255, 0, 0)});

    specs_.push_back({
      "green",
      {{cv::Scalar(40, 80, 60), cv::Scalar(85, 255, 255)}},
      cv::Scalar(0, 255, 0)});
  }

  bool detect_one(const cv::Mat & hsv, const ColorSpec & spec, cv::Rect & box) const
  {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto & rg : spec.ranges) {
      cv::Mat m;
      cv::inRange(hsv, rg.first, rg.second, m);
      cv::bitwise_or(mask, m, mask);
    }
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best = 0.0;
    for (const auto & c : contours) {
      const double a = cv::contourArea(c);
      if (a > best) {
        best = a;
        box = cv::boundingRect(c);
      }
    }
    return best >= static_cast<double>(min_area_);
  }

  void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1, msg->data.data());
    cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[block_detector] 解码失败, format=%s", msg->format.c_str());
      return;
    }

    cv::Mat blurred, hsv;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    foxglove_msgs::msg::ImageAnnotations ann;
    ann.timestamp = msg->header.stamp;

    std::string summary;

    for (size_t i = 0; i < specs_.size(); ++i) {
      const ColorSpec & spec = specs_[i];
      cv::Rect box;
      const bool found = detect_one(hsv, spec, box);

      ifr_interfaces::msg::BlockResult res;
      res.header = msg->header;
      res.color  = spec.name;
      res.found  = found;
      res.x = found ? box.x : -1;
      res.y = found ? box.y : -1;
      res.w = found ? box.width : -1;
      res.h = found ? box.height : -1;
      pub_result_->publish(res);

      if (!found) {
        continue;
      }

      summary += " " + spec.name + "(" + std::to_string(box.x) + "," + std::to_string(box.y) +
                 "," + std::to_string(box.width) + "," + std::to_string(box.height) + ")";

      // 只给"目标颜色"画框；target_color_ 为空时三个都画
      if (!target_color_.empty() && spec.name != target_color_) {
        continue;
      }

      // ---- 1) visualization_msgs/ImageMarker（课程要求的经典写法）----
      visualization_msgs::msg::ImageMarker mk;
      mk.header        = msg->header;
      mk.ns            = "ifr_blocks";
      mk.id            = static_cast<int32_t>(i);
      mk.type          = visualization_msgs::msg::ImageMarker::POLYGON;
      mk.action        = visualization_msgs::msg::ImageMarker::ADD;
      mk.scale         = 3.0f;
      mk.outline_color = to_rgba(spec.bgr, 1.0f);
      mk.filled        = 0;
      mk.points        = corners(box);
      pub_marker_->publish(mk);

      // ---- 2) foxglove_msgs/ImageAnnotations（现代 foxglove 真正会画的）----
      foxglove_msgs::msg::PointsAnnotation pa;
      pa.timestamp     = msg->header.stamp;
      pa.type          = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
      pa.thickness     = 3.0;
      pa.outline_color = to_fox_color(spec.bgr, 1.0);
      pa.fill_color    = to_fox_color(spec.bgr, 0.0);
      for (const auto & p : corners(box)) {
        foxglove_msgs::msg::Point2 p2;
        p2.x = p.x;
        p2.y = p.y;
        pa.points.push_back(p2);
      }
      ann.points.push_back(pa);

      foxglove_msgs::msg::TextAnnotation ta;
      ta.timestamp  = msg->header.stamp;
      ta.position.x = static_cast<double>(box.x);
      ta.position.y = static_cast<double>(std::max(0, box.y - 8));
      ta.text       = spec.name;
      ta.font_size  = 28.0;
      ta.text_color = to_fox_color(spec.bgr, 1.0);
      ta.background_color = to_fox_color(cv::Scalar(0, 0, 0), 0.0);
      ann.texts.push_back(ta);
    }

    pub_ann_->publish(ann);

    if (!summary.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[block_detector] 本帧识别:%s", summary.c_str());
    }
  }

  static std::vector<geometry_msgs::msg::Point> corners(const cv::Rect & r)
  {
    const double x0 = r.x, y0 = r.y;
    const double x1 = r.x + r.width, y1 = r.y + r.height;
    std::vector<geometry_msgs::msg::Point> pts(4);
    pts[0].x = x0; pts[0].y = y0;
    pts[1].x = x1; pts[1].y = y0;
    pts[2].x = x1; pts[2].y = y1;
    pts[3].x = x0; pts[3].y = y1;
    return pts;
  }

  std::string image_topic_;
  std::string target_color_;   // 空 = 三个颜色都画；否则只画这个颜色
  int min_area_{3000};
  cv::Mat kernel_;
  std::vector<ColorSpec> specs_;

  rclcpp::Publisher<ifr_interfaces::msg::BlockResult>::SharedPtr pub_result_;
  rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr pub_ann_;
  rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_target_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlockDetector>());
  rclcpp::shutdown();
  return 0;
}
