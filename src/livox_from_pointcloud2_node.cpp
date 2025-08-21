#include <chrono>
#include <functional>
#include <memory>
#include <string>
// Matrix/vector utils
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
// Point cloud utils
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// ROS2 utils
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
// OpenCV
#include "sensor_msgs/msg/point_cloud2.hpp"
// Custom
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "livox_lidar_utils/conversion.h"

class LivoxConverter : public rclcpp::Node {
 public:
  LivoxConverter() : Node("livox_from_pointcloud2") {
    declare_parameter("topic.out", "/livox/lidar_custom");
    declare_parameter("topic.in", "/livox/lidar_pointcloud2");

    topic_out_ = get_parameter("topic.out").as_string();
    topic_in_ = get_parameter("topic.in").as_string();

    // Initialize subscriptions
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_in_, 10,
        std::bind(&LivoxConverter::pointcloudCallback, this,
                  std::placeholders::_1));
    cloud_pub_ =
        create_publisher<livox_ros_driver2::msg::CustomMsg>(topic_out_, 10);

    printParams();

    RCLCPP_INFO(this->get_logger(),
                "livox_lidar_utils: livox message convert node initialized "
                "successfully.");
  }

 private:
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::string topic_out_;
  std::string topic_in_;

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2 &msg) {
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Node received a message. Converting to CustomMsg...");
    try {
      livox_ros_driver2::msg::CustomMsg msg_livox =
          convertLivoxFromPointcloud2(msg);
      msg_livox.header = msg.header;
      cloud_pub_->publish(msg_livox);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in pointcloudCallback: %s",
                   e.what());
    }
  }

  void printParams() {
    std::ostringstream general_params;
    general_params << "\n--- Parameters ---\n"
                   << "  - Image output topic: " << topic_out_ << "\n"
                   << "  - Pointcloud input topic: " << topic_in_ << "\n";
    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxConverter>());
  rclcpp::shutdown();
  return 0;
}
