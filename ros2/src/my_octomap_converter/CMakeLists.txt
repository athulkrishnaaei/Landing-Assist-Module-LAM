cmake_minimum_required(VERSION 3.5)
project(my_octomap_converter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include CMake module for pcl
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable
add_executable(octomap_to_pcd src/octomap_to_pcd.cpp)
ament_target_dependencies(octomap_to_pcd
  rclcpp
  pcl_conversions
  sensor_msgs
)

target_link_libraries(octomap_to_pcd
  ${PCL_LIBRARIES}
  ${octomap_LIBRARIES}
)


# Declare a C++ executable
add_executable(pcd_to_octomap src/pcd_to_octomap.cpp)
ament_target_dependencies(pcd_to_octomap
  rclcpp
  pcl_conversions
  sensor_msgs
)

target_link_libraries(pcd_to_octomap
  ${PCL_LIBRARIES}
  ${octomap_LIBRARIES}
)

# Install targets
install(TARGETS
  octomap_to_pcd
  pcd_to_octomap
  DESTINATION lib/${PROJECT_NAME}
)

# If this is a ROS package, you might also need to uncomment and adapt the following lines:
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

ament_package()