#include <cctype>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ifr_interfaces/msg/block_result.hpp"
#include "ifr_interfaces/srv/detect_block.hpp"

using ifr_interfaces::msg::BlockResult;
using ifr_interfaces::srv::DetectBlock;

// 一帧里某个颜色的识别结果快照
struct Snapshot
{
  bool found = false;
  int x = -1, y = -1, w = -1, h = -1;
  rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

class DetectBlockServer : public rclcpp::Node
{
public:
  DetectBlockServer() : Node("detect_block_server")
  {
    max_age_ms_ = this->declare_parameter<int>("max_age_ms", 2000);

    sub_ = this->create_subscription<BlockResult>(
      "/block_result", 10,
      std::bind(&DetectBlockServer::on_result, this, std::placeholders::_1));

    // 联动：把"当前要看的颜色"广播出去，detector 据此只画一个框
    pub_target_ = this->create_publisher<std_msgs::msg::String>("/target_color", 10);

    srv_ = this->create_service<DetectBlock>(
      "/detect_block",
      std::bind(&DetectBlockServer::on_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "[detect_block_server] 服务已就绪: /detect_block (结果有效期 %d ms)",
                max_age_ms_);
  }

private:
  // red / RED / 红 / 红色 都归一到 red
  static std::string normalize(std::string s)
  {
    for (char & c : s) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    static const std::unordered_map<std::string, std::string> alias = {
      {"red", "red"},   {"红", "red"},   {"红色", "red"},
      {"blue", "blue"}, {"蓝", "blue"},  {"蓝色", "blue"},
      {"green", "green"}, {"绿", "green"}, {"绿色", "green"}};
    auto it = alias.find(s);
    return (it == alias.end()) ? s : it->second;
  }

  void on_result(const BlockResult::SharedPtr msg)
  {
    Snapshot s;
    s.found = msg->found;
    s.x = msg->x;
    s.y = msg->y;
    s.w = msg->w;
    s.h = msg->h;
    s.stamp = rclcpp::Time(msg->header.stamp);
    cache_[msg->color] = s;
  }

  void on_request(const std::shared_ptr<DetectBlock::Request> req,
                  std::shared_ptr<DetectBlock::Response> res)
  {
    const std::string color = normalize(req->color);
    RCLCPP_INFO(this->get_logger(),
                "[detect_block_server] 收到识别请求, 目标颜色: %s", color.c_str());

    // 只要请求的是三种合法颜色之一，就通知 detector 切换目标颜色
    if (color == "red" || color == "blue" || color == "green") {
      std_msgs::msg::String t;
      t.data = color;
      pub_target_->publish(t);
    }

    res->x = -1; res->y = -1; res->w = -1; res->h = -1;

    const auto it = cache_.find(color);
    if (it == cache_.end()) {
      res->success = false;
      res->message = "没有该颜色的识别结果: '" + req->color +
                     "' (可用: red / blue / green)";
      RCLCPP_WARN(this->get_logger(), "[detect_block_server] %s", res->message.c_str());
      return;
    }

    const Snapshot & s = it->second;
    const double age_ms = (this->now() - s.stamp).seconds() * 1000.0;

    if (age_ms > static_cast<double>(max_age_ms_)) {
      res->success = false;
      res->message = "识别结果已过期 (" + std::to_string(static_cast<int>(age_ms)) + " ms)";
      RCLCPP_WARN(this->get_logger(), "[detect_block_server] %s", res->message.c_str());
      return;
    }

    if (!s.found) {
      res->success = false;
      res->message = "画面中没有 " + color + " 方块";
      RCLCPP_INFO(this->get_logger(), "[detect_block_server] %s", res->message.c_str());
      return;
    }

    res->success = true;
    res->x = s.x;
    res->y = s.y;
    res->w = s.w;
    res->h = s.h;
    res->message = "找到 " + color + " 方块 (结果 " + std::to_string(static_cast<int>(age_ms)) + " ms 前)";

    RCLCPP_INFO(this->get_logger(), "[detect_block_server] 返回 %s: x=%d y=%d w=%d h=%d",
                color.c_str(), res->x, res->y, res->w, res->h);
  }

  int max_age_ms_{2000};
  std::unordered_map<std::string, Snapshot> cache_;
  rclcpp::Subscription<BlockResult>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_target_;
  rclcpp::Service<DetectBlock>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectBlockServer>());
  rclcpp::shutdown();
  return 0;
}
