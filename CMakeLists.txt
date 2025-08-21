cmake_minimum_required(VERSION 3.5)
project(livox_lidar_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_library(conversion include/${PROJECT_NAME}/conversion.h src/conversion.cpp)
target_include_directories(conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${livox_ros_driver2_INCLUDE_DIRS}
)


ament_target_dependencies(conversion 
  OpenCV
  rclcpp 
  sensor_msgs
  livox_ros_driver2
  PCL
  pcl_conversions
  pcl_ros
)


add_library(pointcloud_log_utils src/pointcloud_log_utils.cpp)
target_include_directories(pointcloud_log_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(save_cloud_node src/save_cloud_node.cpp)
add_executable(range_image_node src/range_image_node.cpp)
add_executable(livox_to_pointcloud2_node src/livox_to_pointcloud2_node.cpp)
add_executable(livox_from_pointcloud2_node src/livox_from_pointcloud2_node.cpp)
add_executable(livox_downsample_node src/livox_downsample_node.cpp)
add_executable(livox_renoise_node src/livox_renoise_node.cpp)


target_link_libraries(range_image_node
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  conversion
  pointcloud_log_utils
)

target_link_libraries(livox_to_pointcloud2_node
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  conversion
)

target_link_libraries(livox_from_pointcloud2_node
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  conversion
)

target_link_libraries(livox_renoise_node
  ${catkin_LIBRARIES}
)

target_link_libraries(save_cloud_node 
  conversion
)

target_link_libraries(livox_downsample_node
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

target_include_directories(save_cloud_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(range_image_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(livox_to_pointcloud2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(livox_from_pointcloud2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(livox_renoise_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(livox_downsample_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_compile_options(range_image_node PRIVATE "${OpenMP_CXX_FLAGS}")

ament_target_dependencies(save_cloud_node 
  rclcpp
  sensor_msgs
  std_srvs
  Eigen3
  PCL
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(range_image_node 
  Eigen3 
  rclcpp 
  std_msgs
  geometry_msgs 
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
  cv_bridge
)

ament_target_dependencies(livox_to_pointcloud2_node
  Eigen3 
  rclcpp 
  sensor_msgs
  livox_ros_driver2
  PCL
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(livox_from_pointcloud2_node
  Eigen3 
  rclcpp 
  sensor_msgs
  livox_ros_driver2
  PCL
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(pointcloud_log_utils 
  PCL
  rclcpp 
  sensor_msgs
)

ament_target_dependencies(livox_downsample_node 
  rclcpp 
  livox_ros_driver2
)

ament_target_dependencies(livox_renoise_node 
  rclcpp 
  livox_ros_driver2
)

install(TARGETS
  range_image_node
  save_cloud_node
  livox_to_pointcloud2_node
  livox_from_pointcloud2_node
  livox_downsample_node
  livox_renoise_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
