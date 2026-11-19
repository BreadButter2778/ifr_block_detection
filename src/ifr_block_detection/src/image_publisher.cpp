#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher() : Node("ifr_image_publisher")
  {
    const std::string default_path =
      ament_index_cpp::get_package_share_directory("ifr_block_detection") + "/images/blocks.png";

    image_path_ = this->declare_parameter<std::string>("image_path", default_path);
    fps_        = this->declare_parameter<int>("fps", 10);
    topic_name_ = this->declare_parameter<std::string>("topic", "/camera/compressed");

    image_ = cv::imread(image_path_, cv::IMREAD_COLOR);
    if (image_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "读不到图片: %s", image_path_.c_str());
      throw std::runtime_error("image not found: " + image_path_);
    }
    RCLCPP_INFO(this->get_logger(), "[image_publisher] 图片路径: %s (%d x %d)",
                image_path_.c_str(), image_.cols, image_.rows);

    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_, 10);

    const auto period = std::chrono::milliseconds(1000 / fps_);
    timer_ = this->create_wall_timer(period, std::bind(&ImagePublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "[image_publisher] 以 %d Hz 发布 %s",
                fps_, topic_name_.c_str());
  }

private:
  void on_timer()
  {
    std::vector<uchar> buffer;
    cv::imencode(".jpg", image_, buffer, {cv::IMWRITE_JPEG_QUALITY, 90});

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "camera";
    msg.format          = "jpeg";
    msg.data            = buffer;

    publisher_->publish(msg);
  }

  std::string image_path_;
  std::string topic_name_;
  int fps_{10};
  cv::Mat image_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("image_publisher"), "启动失败: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
