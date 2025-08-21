#ifndef POINTCLOUD_LOG_UTILS_H
#define POINTCLOUD_LOG_UTILS_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define ENABLE_TIMING_LOGS true
#define ENABLE_DEBUG_LOGS true

#if ENABLE_TIMING_LOGS
#define LOG_TIMING(function_name, start, end)                              \
  do {                                                                     \
    RCLCPP_INFO(                                                           \
        rclcpp::get_logger("timing"), "%s took %ld microseconds",          \
        function_name,                                                     \
        std::chrono::duration_cast<std::chrono::microseconds>(end - start) \
            .count());                                                     \
  } while (0)
#else
#define LOG_TIMING(function_name, start, end) \
  do {                                        \
  } while (0)
#endif

#if ENABLE_DEBUG_LOGS
#define LOG_DEBUG_INFO(function_name)                              \
  do {                                                             \
    RCLCPP_INFO(rclcpp::get_logger("debug"), "%s", function_name); \
  } while (0)
#else
#define LOG_DEBUG_INFO(function_name) \
  do {                                \
  } while (0)
#endif

void printPointCloudInfo(const sensor_msgs::msg::PointCloud2 &msg);

#endif