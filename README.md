# LiDAR SLAM Utils

A collection of ROS 2 tools to process, visualize, and save data from **Livox Mid-360** sensors.

## Overview

This package provides a set of utilities for working with Livox Mid-360 LiDAR data in ROS 2.  
It includes nodes for converting, visualizing, saving, and extracting features from point clouds and related sensor data.

## Nodes

### 🖼️ `range_image_node`
- **Description**: Converts a point cloud into a 2D range image using vertical and horizontal fields of view.
- **Input**: `/PointCloud2`
- **Parameters**:
    - `topic.in`: input topic containing a `sensor_msgs/msg/PointCloud2` message.
    - `topic.out`: output topic to publish the generated range image as a `sensor_msgs/msg/Image`.
    - `azimuth.res_deg`: angular resolution in degrees along the horizontal (azimuth) axis.
    - `azimuth.fov_deg`: field of view in degrees along the azimuth axis.
    - `azimuth.min_deg`: minimum azimuth angle to start populating the image.
    - `elevation.res_deg`: angular resolution in degrees along the vertical (elevation) axis.
    - `elevation.fov_deg`: field of view in degrees along the elevation axis.
    - `elevation.min_deg`: minimum elevation angle to start populating the image.
    - `max_range_m`: maximum range (in meters) to consider when generating the image. Points beyond this range are ignored.

- **Output**: Range image as `sensor_msgs/msg/Image`

---

### 🔁 `livox_renoise_node`
- **Description**: Add white Gaussian noise on the Livox measurements.
- **Parameter**: 
    - `topic.in`: Topic of the point cloud to subscribe to
    - `topic.out`: Topic of the point cloud after renoise
    - `sigma.range_m`: standard deviation of the noise on the range mesurement [m]
    - `sigma.angle_deg`: standard deviation of the noise on the angle mesurement [deg]
- **Use Case**: Useful for testing robustness of algorithms dependent on Livox LiDARs.

---

### 💾 `save_cloud_node`
- **Description**: Saves incoming `/PointCloud2` or Livox `/CustomMsg` messages as `.pcd` point clouds.
- **Parameters**:
    - `topic.in`: Topic to subscribe to
    - `format.in`: `0` for `CustomMsg`, `1` for `PointCloud2`
    - `path`: Save directory
    - `format.out`: Currently only `"pcd"` is supported
    - `format.save_with_timestamp`: When enabled, the point cloud is saved with a name that is `seconds_nanoseconds.pcd`. Otherwise, they are saved with 0-padding in order from 0 to a max of 99999
- **Output**: `.pcd` files

---

### 🔁 `livox_to_pointcloud2_node`
- **Description**: Converts Livox `/CustomMsg` to standard ROS 2 `/PointCloud2`.
- **Parameter**: 
    - `topic.in`: Topic to subscribe to
    - `topic.out`: Topic of the message after conversion
    - `enable_lightweight`: condidered only when not using `override_line`, it publishes the point cloud with fields `xyz` instead of `xyzitlt`
    - `override_line`: When `false` uses the point cloud field `ring`, when `true` the ring number is computed with the following parameters
    - `elevation.fov_deg`: vertical field of view of the LiDAR
    - `elevation.min_deg`: vertical minimum angle of the LiDAR
    - `num_scan_lines`: number of rings 
- **Use Case**: Useful for standard visualization tools like RViz

---

### 🔁 `livox_from_pointcloud2_node`
- **Description**: Converts `/PointCloud2` into Livox `/CustomMsg`.
- **Parameter**: 
    - `topic.in`: Topic to subscribe to
    - `topic.out`: Topic of the message after conversion
- **Use Case**: Useful for simulations or toolchains expecting Livox message formats

## Dependencies

This package depends on the following ROS 2 and system packages:

```cmake
PCL
pcl_conversions
pcl_ros
geometry_msgs
sensor_msgs
livox_ros_driver2
std_msgs
std_srvs
Eigen3
OpenCV
cv_bridge
OpenMP
```

## Usage

Build the package in your ROS 2 workspace:

```bash
colcon build --packages-select liovx_lidar_utils
source install/setup.bash
```

Then launch or run the desired node using `ros2 run` or `ros2 launch`.


