#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS2 utils
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
// Custom
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxDownsampler : public rclcpp::Node {
 public:
  LivoxDownsampler() : Node("livox_downsample_node") {
    declare_parameter("topic.in", "/livox/lidar");
    declare_parameter("topic.out", "/livox/lidar_ds");
    declare_parameter("decimation", 2);  // decimation factor for downsampling

    topic_out_ = get_parameter("topic.out").as_string();
    topic_in_ = get_parameter("topic.in").as_string();
    decimation_ = get_parameter("decimation").as_int();

    // Initialize subscriptions
    cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        topic_in_, 10,
        std::bind(&LivoxDownsampler::pointcloudCallback, this,
                  std::placeholders::_1));
    cloud_pub_ =
        create_publisher<livox_ros_driver2::msg::CustomMsg>(topic_out_, 10);

    printParams();

    RCLCPP_INFO(this->get_logger(),
                "livox_lidar_utils: livox message downsampler node initialized "
                "successfully.");
  }

 private:
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  std::string topic_out_;
  std::string topic_in_;
  int decimation_;

  void pointcloudCallback(const livox_ros_driver2::msg::CustomMsg &msg) {
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Node received first message. Applying downsampling...");
    try {
      livox_ros_driver2::msg::CustomMsg downsampled_msg;
      downsampled_msg.header = msg.header;
      for (size_t i = 0; i < msg.points.size(); i += decimation_) {
        // Add the downsampled point to the new message
        downsampled_msg.points.push_back(msg.points[i]);
      }
      downsampled_msg.point_num = downsampled_msg.points.size();
      cloud_pub_->publish(downsampled_msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in pointcloudCallback: %s",
                   e.what());
    }
  }

  void printParams() {
    std::ostringstream general_params;
    general_params << "\n--- Parameters ---\n"
                   << "  - CustomMsg input topic: " << topic_in_ << "\n"
                   << "  - CustomMsg output topic: " << topic_out_ << "\n"
                   << "  - Decimation: " << decimation_ << "\n";

    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxDownsampler>());
  rclcpp::shutdown();
  return 0;
}
