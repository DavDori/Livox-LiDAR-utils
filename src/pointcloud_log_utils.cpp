#include "livox_lidar_utils/pointcloud_log_utils.h"

void printPointCloudInfo(const sensor_msgs::msg::PointCloud2 &msg) {
  // Print basic information about the PointCloud2 message
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PointCloud2 Info:");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  width: %u", msg.width);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  height: %u", msg.height);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  is_dense: %s",
              msg.is_dense ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  is_bigendian: %s",
              msg.is_bigendian ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  point_step: %u", msg.point_step);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  row_step: %u", msg.row_step);

  // Print fields (except for the raw data)
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fields:");
  for (const auto &field : msg.fields) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Field name: %s, Offset: %u, Data type: %d, Count: %u",
                field.name.c_str(), field.offset, field.datatype, field.count);
  }
}