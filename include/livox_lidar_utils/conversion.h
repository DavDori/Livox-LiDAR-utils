#ifndef CONVERSION_H
#define CONVERSION_H

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct PointXYZITLT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  uint8_t tag;
  uint8_t line;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZITLT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint8_t, tag, tag)(uint8_t, line, line)(double, time, time))

struct PointCloudOrganizationParams {
  float azim_fov_rad = 0.0f;
  float inv_azim_res_rad = 0.0f;
  float azim_min_rad = 0.0f;
  float elev_fov_rad = 0.0f;
  float inv_elev_res_rad = 0.0f;
  float elev_min_rad = 0.0f;
  float max_range_sq = 0.0f;
  int num_scan_lines = 16;  // Number of scan lines
};

pcl::PointCloud<PointXYZITLT>::Ptr convertToXYZITLT(
    const sensor_msgs::msg::PointCloud2 &msg);
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(
    const sensor_msgs::msg::PointCloud2 &msg);

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &unorganized_cloud,
    const PointCloudOrganizationParams &params);

pcl::PointCloud<PointXYZITLT>::Ptr organizePCLPointCloud(
    const pcl::PointCloud<PointXYZITLT>::Ptr &unorganized_cloud,
    const PointCloudOrganizationParams &params);

inline std::vector<float> convToFloat(const std::vector<double> &input);

cv::Mat pointCloudToRangeImage(const pcl::PointCloud<PointXYZITLT>::Ptr &cloud,
                               float max_range);

pcl::PointCloud<PointXYZITLT>::Ptr convertLivoxToPCLXYZITL(
    const livox_ros_driver2::msg::CustomMsg &msg);

pcl::PointCloud<pcl::PointXYZ>::Ptr convertLivoxToPCLXYZ(
    const livox_ros_driver2::msg::CustomMsg &msg);

pcl::PointCloud<PointXYZITLT>::Ptr convertLivoxToPCLXYZITLT(
    const livox_ros_driver2::msg::CustomMsg &msg,
    const PointCloudOrganizationParams &param);

sensor_msgs::msg::PointCloud2 convertLivoxToPointcloud2(
    const livox_ros_driver2::msg::CustomMsg &msg, bool only_xyz);

sensor_msgs::msg::PointCloud2 convertLivoxToPointcloud2(
    const livox_ros_driver2::msg::CustomMsg &msg,
    const PointCloudOrganizationParams &param);

livox_ros_driver2::msg::CustomMsg convertLivoxFromPCL(
    const pcl::PointCloud<PointXYZITLT>::Ptr &cloud);

livox_ros_driver2::msg::CustomMsg convertLivoxFromPointcloud2(
    const sensor_msgs::msg::PointCloud2 &msg);

double get_time_sec(const builtin_interfaces::msg::Time &time);

inline double sphericalToCartesianX(double range, double azim,
                                    double elev) noexcept;
inline double sphericalToCartesianY(double range, double azim,
                                    double elev) noexcept;
inline double sphericalToCartesianZ(double range, double elev) noexcept;
inline double sphericalFromCartesianR(double x, double y, double z) noexcept;
inline double sphericalFromCartesianAzim(double x, double y) noexcept;
inline double sphericalFromCartesianElev(double z, double r) noexcept;

#endif
