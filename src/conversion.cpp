#include "livox_lidar_utils/conversion.h"

pcl::PointCloud<PointXYZITLT>::Ptr convertToXYZITLT(
    const sensor_msgs::msg::PointCloud2 &msg) {
  auto cloud = std::make_shared<pcl::PointCloud<PointXYZITLT>>();
  cloud->is_dense = msg.is_dense;
  cloud->height = msg.height;
  cloud->width = msg.width;
  cloud->points.resize(msg.width * msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_tag(msg, "tag");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(msg, "line");
  sensor_msgs::PointCloud2ConstIterator<double> iter_time(msg, "time");

  for (size_t i = 0; i < cloud->points.size(); ++i, ++iter_x, ++iter_y,
              ++iter_z, ++iter_intensity, ++iter_tag, ++iter_line,
              ++iter_time) {
    PointXYZITLT &pt = cloud->points[i];
    pt.x = *iter_x;
    pt.y = *iter_y;
    pt.z = *iter_z;
    pt.intensity = *iter_intensity;
    pt.tag = *iter_tag;
    pt.line = *iter_line;
    pt.time = *iter_time;
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(
    const sensor_msgs::msg::PointCloud2 &msg) {
  // Create a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Set PCL point cloud metadata
  pcl_cloud->width = msg.width;
  pcl_cloud->height = msg.height;
  pcl_cloud->is_dense = msg.is_dense == true;
  pcl_cloud->points.resize(msg.width * msg.height);

  // Use iterators to extract fields
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  for (size_t i = 0; i < pcl_cloud->points.size();
       ++i, ++iter_x, ++iter_y, ++iter_z) {
    pcl_cloud->points[i].x = *iter_x;
    pcl_cloud->points[i].y = *iter_y;
    pcl_cloud->points[i].z = *iter_z;
  }
  return pcl_cloud;
}

sensor_msgs::msg::PointCloud2 organizePointCloud2(
    const pcl::PointCloud<PointXYZITLT>::Ptr &unorganized_cloud,
    const PointCloudOrganizationParams &params) {
  pcl::PointCloud<PointXYZITLT>::Ptr organized_cloud =
      organizePCLPointCloud(unorganized_cloud, params);
  // Initialize the organized PointCloud2 message
  sensor_msgs::msg::PointCloud2 ros_cloud;
  ros_cloud.width = organized_cloud->width;
  ros_cloud.height = organized_cloud->height;
  ros_cloud.is_dense = organized_cloud->is_dense;
  ros_cloud.is_bigendian = false;
  ros_cloud.point_step = 16;  // 4 fields of float32 (x, y, z, intensity)
  ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before modifier: w=%d, h=%d",
              ros_cloud.width, ros_cloud.height);

  // Define fields for XYZ and Intensity
  sensor_msgs::PointCloud2Modifier modifier(ros_cloud);
  modifier.setPointCloud2Fields(4, "x", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32);

  // Resize the cloud to accommodate width * height points
  modifier.resize(organized_cloud->width * organized_cloud->height);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After modifier: w=%d, h=%d",
              ros_cloud.width, ros_cloud.height);

  // Use iterators to fill in the data for ros_cloud
  sensor_msgs::PointCloud2Iterator<float> iter_x(ros_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(ros_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(ros_cloud, "z");

  for (const auto &point : organized_cloud->points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  ros_cloud.width = organized_cloud->width;
  ros_cloud.height = organized_cloud->height;
  return ros_cloud;
}

pcl::PointCloud<PointXYZITLT>::Ptr organizePCLPointCloud(
    const pcl::PointCloud<PointXYZITLT>::Ptr &unorganized_cloud,
    const PointCloudOrganizationParams &params) {
  // Calculate organized cloud dimensions
  int width = static_cast<int>(
      std::round(params.azim_fov_rad * params.inv_azim_res_rad));
  int height = static_cast<int>(
      std::round(params.elev_fov_rad * params.inv_elev_res_rad));

  // Initialize the organized cloud
  pcl::PointCloud<PointXYZITLT>::Ptr organized_cloud(
      new pcl::PointCloud<PointXYZITLT>);
  organized_cloud->width = width;
  organized_cloud->height = height;
  organized_cloud->is_dense = false;

  organized_cloud->points.resize(organized_cloud->width *
                                 organized_cloud->height);

  // Set all points to NaN
  for (auto &point : organized_cloud->points) {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
  }

  // Map points to grid
#pragma omp parallel for
  for (size_t i = 0; i < unorganized_cloud->points.size(); ++i) {
    const auto &point = unorganized_cloud->points[i];
    float dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;

    bool is_finite = std::isfinite(point.x) && std::isfinite(point.y) &&
                     std::isfinite(point.z);
    if (!is_finite || dist_sq >= params.max_range_sq) {
      continue;
    }

    float azim_rad = std::atan2(point.y, point.x);
    float azim_delta_rad = azim_rad - params.azim_min_rad;
    float dist = std::sqrt(dist_sq);
    float elev_rad = std::asin(point.z / dist);
    float elev_delta_rad = elev_rad - params.elev_min_rad;

    int row =
        static_cast<int>(std::floor(elev_delta_rad * params.inv_elev_res_rad));
    int col =
        static_cast<int>(std::floor(azim_delta_rad * params.inv_azim_res_rad));
    if (col < 0 || col >= width || row < 0 || row >= height) {
      continue;
    }
    int idx = row * width + col;

    float dist_sq_old =
        organized_cloud->points[idx].x * organized_cloud->points[idx].x +
        organized_cloud->points[idx].y * organized_cloud->points[idx].y +
        organized_cloud->points[idx].z * organized_cloud->points[idx].z;

    // if the cell is empty or the point is closer add the current point to the
    // organized cloud
    if (!std::isfinite(dist_sq_old) || dist_sq < dist_sq_old) {
      organized_cloud->points[idx] = point;
    }
  }
  return organized_cloud;
}

inline std::vector<float> convToFloat(const std::vector<double> &input) {
  return std::vector<float>(input.begin(), input.end());
}

cv::Mat pointCloudToRangeImage(const pcl::PointCloud<PointXYZITLT>::Ptr &cloud,
                               float max_range) {
  // Ensure the PointCloud2 is valid
  if (!cloud || cloud->height == 0 || cloud->width == 0) {
    throw std::runtime_error("Invalid PCL pointcloud: Empty cloud");
  }

  // Initialize an OpenCV Mat with the same height and width as the PointCloud2
  cv::Mat img(cloud->height, cloud->width, CV_32FC1, cv::Scalar(0));
  float inv_max_range = 1.0f / max_range;
  // Iterate through the organized point cloud
  for (size_t row = 0; row < cloud->height; ++row) {
    for (size_t col = 0; col < cloud->width; ++col) {
      const auto &point = cloud->at(col, row);
      size_t u = col;
      size_t v = cloud->height - row - 1;  // image coordinates are flipped

      // Check if the point is valid (not NaN)
      if (pcl::isFinite(point)) {
        // Calculate the range value (distance from the origin)
        float range = std::sqrt(point.x * point.x + point.y * point.y +
                                point.z * point.z);
        if (range > max_range) {
          range = max_range;
        }
        img.at<float>(v, u) = range * inv_max_range;  // access as [row, col]
      } else {
        img.at<float>(v, u) = 0;  // Set invalid points to 0
      }
    }
  }
  return img;
}

pcl::PointCloud<PointXYZITLT>::Ptr convertLivoxToPCLXYZITLT(
    const livox_ros_driver2::msg::CustomMsg &msg,
    const PointCloudOrganizationParams &param) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointXYZITLT>>();
  cloud->points.resize(msg.point_num);
  cloud->width = msg.point_num;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->header.frame_id = msg.header.frame_id;

  for (size_t i = 0; i < msg.point_num; ++i) {
    const auto &src = msg.points[i];
    double pitch = atan2(src.z, sqrt(src.x * src.x + src.y * src.y));
    int line = int((pitch - param.elev_min_rad) * param.inv_elev_res_rad);
    if (line < 0 || line >= param.num_scan_lines) {
      continue;
    }

    auto &dst = cloud->points[i];

    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = static_cast<float>(src.reflectivity);
    dst.tag = static_cast<uint8_t>(src.tag);
    dst.line = static_cast<uint8_t>(line);
    dst.time = static_cast<double>(src.offset_time) * 1e-9f;  // ns -> s
  }

  return cloud;
}

pcl::PointCloud<PointXYZITLT>::Ptr convertLivoxToPCLXYZITLT(
    const livox_ros_driver2::msg::CustomMsg &msg) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointXYZITLT>>();
  cloud->points.resize(msg.point_num);
  cloud->width = msg.point_num;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->header.frame_id = msg.header.frame_id;

  for (size_t i = 0; i < msg.point_num; ++i) {
    const auto &src = msg.points[i];
    auto &dst = cloud->points[i];

    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = static_cast<float>(src.reflectivity);
    dst.tag = static_cast<uint8_t>(src.tag);
    dst.line = static_cast<uint8_t>(src.line);
    dst.time = static_cast<double>(src.offset_time) * 1e-9f;  // ns -> s
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertLivoxToPCLXYZ(
    const livox_ros_driver2::msg::CustomMsg &msg) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->points.resize(msg.point_num);
  cloud->width = msg.point_num;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->header.frame_id = msg.header.frame_id;

  for (size_t i = 0; i < msg.point_num; ++i) {
    const auto &src = msg.points[i];
    auto &dst = cloud->points[i];

    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
  }

  return cloud;
}

sensor_msgs::msg::PointCloud2 convertLivoxToPointcloud2(
    const livox_ros_driver2::msg::CustomMsg &msg,
    const PointCloudOrganizationParams &param) {
  pcl::PointCloud<PointXYZITLT>::Ptr cloud =
      convertLivoxToPCLXYZITLT(msg, param);
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  return cloud_msg;
}

/*
Convert Livox CustomMsg to PointCloud2 message.
*/
sensor_msgs::msg::PointCloud2 convertLivoxToPointcloud2(
    const livox_ros_driver2::msg::CustomMsg &msg, bool only_xyz) {
  sensor_msgs::msg::PointCloud2 cloud_msg;

  if (only_xyz) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertLivoxToPCLXYZ(msg);
    pcl::toROSMsg(*cloud, cloud_msg);
  } else {
    pcl::PointCloud<PointXYZITLT>::Ptr cloud = convertLivoxToPCLXYZITLT(msg);
    pcl::toROSMsg(*cloud, cloud_msg);
  }
  return cloud_msg;
}

livox_ros_driver2::msg::CustomMsg convertLivoxFromPCL(
    const pcl::PointCloud<PointXYZITLT>::Ptr &cloud) {
  livox_ros_driver2::msg::CustomMsg msg;
  msg.header.frame_id = cloud->header.frame_id;
  msg.point_num = cloud->points.size();
  msg.points.resize(msg.point_num);
  for (size_t i = 0; i < msg.point_num; ++i) {
    livox_ros_driver2::msg::CustomPoint point_msg;
    const auto &src = cloud->points[i];

    point_msg.x = src.x;
    point_msg.y = src.y;
    point_msg.z = src.z;
    point_msg.reflectivity = static_cast<uint8_t>(src.intensity);
    point_msg.tag = static_cast<uint8_t>(src.tag);
    point_msg.line = static_cast<uint8_t>(src.line);
    point_msg.offset_time = static_cast<uint64_t>(src.time * 1e9f);  // s -> ns
    msg.points[i] = std::move(point_msg);
  }

  return msg;
}

livox_ros_driver2::msg::CustomMsg convertLivoxFromPointcloud2(
    const sensor_msgs::msg::PointCloud2 &msg) {
  pcl::PointCloud<PointXYZITLT>::Ptr cloud = convertToXYZITLT(msg);
  return convertLivoxFromPCL(cloud);
}

double get_time_sec(const builtin_interfaces::msg::Time &time) {
  return rclcpp::Time(time).seconds();
}

inline double sphericalToCartesianX(double range, double azim,
                                    double elev) noexcept {
  return range * std::cos(elev) * std::cos(azim);
}
inline double sphericalToCartesianY(double range, double azim,
                                    double elev) noexcept {
  return range * std::cos(elev) * std::sin(azim);
}
inline double sphericalToCartesianZ(double range, double elev) noexcept {
  return range * std::sin(elev);
}
inline double sphericalFromCartesianR(double x, double y, double z) noexcept {
  return std::sqrt(x * x + y * y + z * z);
}
inline double sphericalFromCartesianAzim(double x, double y) noexcept {
  return std::atan2(y, x);
}
inline double sphericalFromCartesianElev(double z, double r) noexcept {
  return std::asin(z / r);
}