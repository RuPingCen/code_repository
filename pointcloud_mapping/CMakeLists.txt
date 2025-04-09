cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_mapping
  CATKIN_DEPENDS sensor_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_mapping src/pointcloud_mapping_node.cpp)
target_link_libraries(pointcloud_mapping
  ${catkin_LIBRARIES}
  pcl_common
  pcl_io
)

install(TARGETS pointcloud_mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
