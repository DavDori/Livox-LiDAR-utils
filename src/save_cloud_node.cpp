#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "livox_lidar_utils/conversion.h"

class SaveCloudNode : public rclcpp::Node {
 public:
  SaveCloudNode() : Node("save_cloud_node") {
    declare_parameter("topic.in", "/livox/lidar");
    declare_parameter("format.in", 1);  // 0: CustomMsg, 1: PointCloud2
    declare_parameter("path", "./pointclouds/");
    declare_parameter("format.out", "pcd");
    declare_parameter("format.save_with_timestamp", true);

    std::string topic_in = get_parameter("topic.in").as_string();
    save_path_ = get_parameter("path").as_string();
    save_format_ = get_parameter("format.out").as_string();
    save_with_timestamp_ =
        get_parameter("format.save_with_timestamp").as_bool();

    // Initialize subscriptions
    if (get_parameter("format.in").as_int() == 0) {
      livox_cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topic_in, 100,
          std::bind(&SaveCloudNode::livoxCloudCallback, this,
                    std::placeholders::_1));
      std_cloud_sub_ = nullptr;
    } else {
      livox_cloud_sub_ = nullptr;
      std_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          topic_in, 100,
          std::bind(&SaveCloudNode::stdCloudCallback, this,
                    std::placeholders::_1));
    }

    count_ = 0;

    std::ostringstream general_params;
    general_params << "\n--- Parameters ---\n"
                   << "  - cloud input topic: " << topic_in << "\n"
                   << "  - point cloud format: "
                   << (get_parameter("format.in").as_int() != 0
                           ? "sesnor_msgs/PointCloud2"
                           : "livox_ros_driver2/CustomMsg")
                   << "\n"
                   << "  - saving data in : " << save_path_ << "\n"
                   << "  - format data in : " << save_format_ << "\n";
    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());

    RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr std_cloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_cloud_sub_;
  std::string save_path_;
  std::string save_format_;
  size_t count_ = 0;
  bool save_with_timestamp_ = true;

  void stdCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Node received first PointCloud2 message. Converting to %s...",
        save_format_.c_str());
    setupSavePath();

    auto cloud = convertToXYZ(*msg);
    if (!cloud) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to convert PointCloud2 to PCL format");
      return;
    }

    saveCloudToFile(cloud, msg->header);
    count_++;
  }

  void livoxCloudCallback(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Node received first CustomMsg message. Converting to %s...",
        save_format_.c_str());
    setupSavePath();

    auto cloud = convertLivoxToPCLXYZ(*msg);
    if (!cloud) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to convert CustomMsg to PCL format");
      return;
    }

    saveCloudToFile(cloud, msg->header);
    count_++;
  }

  std::string getCountString() {
    // Convert count_ to string with leading zeros
    std::ostringstream oss;
    if (count_ < 10)
      oss << "0000";
    else if (count_ < 100)
      oss << "000";
    else if (count_ < 1000)
      oss << "00";
    else if (count_ < 10000)
      oss << "0";
    oss << count_;
    return oss.str();
  }

  void saveCloudToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       const std_msgs::msg::Header &header) {
    // Save the point cloud to a file
    std::string filename = "";
    if (save_with_timestamp_) {
      filename = save_path_ + std::to_string(header.stamp.sec) + "_" +
                 std::to_string(header.stamp.nanosec) + "." + save_format_;
    } else {
      filename = save_path_ + getCountString() + "." + save_format_;
    }
    if (save_format_ == "pcd") {
      pcl::io::savePCDFileASCII(filename, *cloud);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported format: %s",
                   save_format_.c_str());
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Saved %ld point clouds", count_);
  }

  void setupSavePath() {
    if (!std::filesystem::exists(save_path_)) {
      std::filesystem::create_directories(save_path_);
      RCLCPP_INFO(this->get_logger(), "Created directory: %s",
                  save_path_.c_str());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveCloudNode>());
  rclcpp::shutdown();
  return 0;
}
