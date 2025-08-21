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
#include <pcl_conversions/pcl_conversions.h>
// ROS2 utils
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
// OpenCV
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
// Custom
#include "livox_lidar_utils/conversion.h"

class RangeImage : public rclcpp::Node {
  using AffineTransform3f = Eigen::Transform<float, 3, Eigen::Affine>;

 public:
  RangeImage() : Node("range_image") {
    declare_parameter("topic.out", "/image/pointcloud");
    declare_parameter("topic.in", "/pointcloud");
    declare_parameter("azimuth.res_deg", 0.225);
    declare_parameter("azimuth.fov_deg", 360.0);
    declare_parameter("azimuth.min_deg", -180.0);
    declare_parameter("elevation.res_deg", 1.0);
    declare_parameter("elevation.fov_deg", 30.0);
    declare_parameter("elevation.min_deg", -15.0);
    declare_parameter("max_range_m", 50.0);

    std::string topic_out = get_parameter("topic.out").as_string();
    std::string topic_in = get_parameter("topic.in").as_string();

    /* It is more efficient to store the inverse of the resolution*/
    params_.inv_azim_res_rad =
        1.0f / (float)DEG2RAD(get_parameter("azimuth.res_deg").as_double());
    params_.azim_fov_rad =
        (float)DEG2RAD(get_parameter("azimuth.fov_deg").as_double());
    params_.azim_min_rad =
        (float)DEG2RAD(get_parameter("azimuth.min_deg").as_double());
    params_.inv_elev_res_rad =
        1.0f / (float)DEG2RAD(get_parameter("elevation.res_deg").as_double());
    params_.elev_fov_rad =
        (float)DEG2RAD(get_parameter("elevation.fov_deg").as_double());
    params_.elev_min_rad =
        (float)DEG2RAD(get_parameter("elevation.min_deg").as_double());
    float max_range_m = (float)(get_parameter("max_range_m").as_double());
    params_.max_range_sq = max_range_m * max_range_m;
    // Set Quality Of Service for subscribers as sensor to prioritize the
    // most recent message, and improve reliability + low latency
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // Initialize subscriptions
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_in, sensor_qos,
        std::bind(&RangeImage::pointcloudCallback, this,
                  std::placeholders::_1));
    img_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_out, 5);

    std::ostringstream general_params;
    general_params
        << "\n--- Parameters ---\n"
        << "  - Image output topic: " << topic_out << "\n"
        << "  - Pointcloud input topic: " << topic_in << "\n"
        << "  - Max range squared: " << params_.max_range_sq << " m^2\n"
        << "  - Azimuth\n"
        << "      * resolution: " << RAD2DEG(1.0f / params_.inv_azim_res_rad)
        << " deg\n"
        << "      * FOV: " << RAD2DEG(params_.azim_fov_rad) << " deg\n"
        << "      * start angle: " << RAD2DEG(params_.azim_min_rad) << " deg\n"
        << "  - Elevation\n"
        << "      * resolution: " << RAD2DEG(1.0f / params_.inv_elev_res_rad)
        << " deg\n"
        << "      * FOV: " << RAD2DEG(params_.elev_fov_rad) << " deg\n"
        << "      * start angle: " << RAD2DEG(params_.elev_min_rad) << " deg\n";

    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
    RCLCPP_INFO(this->get_logger(),
                "livox_lidar_utils: point cloud to range image initialized "
                "successfully.");
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;
  PointCloudOrganizationParams params_;

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2 &msg) {
    try {
      // Convert to PCL format
      auto pcl_cloud = convertToXYZITLT(msg);

      Eigen::Matrix3f rotation_matrix;
      rotation_matrix = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
      AffineTransform3f center_rotation = AffineTransform3f::Identity();
      center_rotation.rotate(rotation_matrix);
      pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, center_rotation);

      auto organized_cloud = organizePCLPointCloud(pcl_cloud, params_);
      // Convert pointcloud to organized pointcloud
      cv::Mat range_image = pointCloudToRangeImage(
          organized_cloud, std::sqrt(params_.max_range_sq));

      cv::Mat normalized_image;
      cv::normalize(range_image, normalized_image, 0, 255, cv::NORM_MINMAX,
                    CV_8UC1);
      // cv::imwrite("./range_image.jpg", normalized_image);
      std_msgs::msg::Header header = msg.header;
      sensor_msgs::msg::Image::SharedPtr msg_img =
          cv_bridge::CvImage(header, "mono8", normalized_image).toImageMsg();
      msg_img->header.stamp = rclcpp::Clock().now();

      img_pub_->publish(*msg_img);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in pointcloudCallback: %s",
                   e.what());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeImage>());
  rclcpp::shutdown();
  return 0;
}
