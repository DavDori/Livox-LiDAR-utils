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
  LivoxConverter() : Node("livox_to_pointcloud2") {
    declare_parameter("topic.out", "/livox/lidar_converted");
    declare_parameter("topic.in", "/livox/lidar_custom");
    declare_parameter("elevation.fov_deg", 60.0);
    declare_parameter("elevation.min_deg", -7.0);
    declare_parameter("num_scan_lines", 32);
    declare_parameter(
        "override_line",
        false);  // when true, the line is calculated from the pitch angle
    declare_parameter("enable_lightweight",
                      false);  // Point cloud output is XYZ or XYZITLT

    topic_out_ = get_parameter("topic.out").as_string();
    topic_in_ = get_parameter("topic.in").as_string();
    override_line_ = get_parameter("override_line").as_bool();
    en_lightweight_ = get_parameter("enable_lightweight").as_bool();

    /* It is more efficient to store the inverse of the resolution*/
    params_.num_scan_lines = get_parameter("num_scan_lines").as_int();

    params_.elev_fov_rad =
        (float)DEG2RAD(get_parameter("elevation.fov_deg").as_double());
    params_.inv_elev_res_rad = params_.num_scan_lines / params_.elev_fov_rad;
    params_.elev_min_rad =
        (float)DEG2RAD(get_parameter("elevation.min_deg").as_double());

    // Initialize subscriptions
    cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        topic_in_, 10,
        std::bind(&LivoxConverter::pointcloudCallback, this,
                  std::placeholders::_1));
    cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(topic_out_, 10);

    printParams();

    RCLCPP_INFO(this->get_logger(),
                "livox_lidar_utils: livox message convert node initialized "
                "successfully.");
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  PointCloudOrganizationParams params_;
  std::string topic_out_;
  std::string topic_in_;
  bool override_line_;
  bool en_lightweight_;

  void pointcloudCallback(const livox_ros_driver2::msg::CustomMsg &msg) {
    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Node received first message. Converting to PointCloud2...");
    try {
      if (override_line_) {
        sensor_msgs::msg::PointCloud2 msg_cloud =
            convertLivoxToPointcloud2(msg, params_);
        msg_cloud.header = msg.header;
        cloud_pub_->publish(msg_cloud);
      } else {
        sensor_msgs::msg::PointCloud2 msg_cloud =
            convertLivoxToPointcloud2(msg, en_lightweight_);
        msg_cloud.header = msg.header;
        cloud_pub_->publish(msg_cloud);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in pointcloudCallback: %s",
                   e.what());
    }
  }

  void printParams() {
    std::ostringstream general_params;
    general_params << "\n--- Parameters ---\n"
                   << "  - Line override: "
                   << (override_line_ ? "true" : "false") << "\n"
                   << "  - CustomMsg output topic: " << topic_out_ << "\n"
                   << "  - Pointcloud2 input topic: " << topic_in_ << "\n"
                   << "  - Enable lightweight mode: "
                   << (en_lightweight_ ? "true" : "false") << "\n";
    if (override_line_) {
      general_params << "  - Elevation\n"
                     << "      * num scan lines: " << params_.num_scan_lines
                     << "\n"
                     << "      * resolution: "
                     << RAD2DEG(1.0f / params_.inv_elev_res_rad) << " deg\n"
                     << "      * FOV: " << RAD2DEG(params_.elev_fov_rad)
                     << " deg\n"
                     << "      * start angle: " << RAD2DEG(params_.elev_min_rad)
                     << " deg\n";
    }

    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxConverter>());
  rclcpp::shutdown();
  return 0;
}
