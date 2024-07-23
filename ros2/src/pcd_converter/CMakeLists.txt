cmake_minimum_required(VERSION 3.5)
project(pcd_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io)  # Adjust version as necessary
find_package(octomap REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OCTOMAP_LIBRARY_DIRS}
)

add_executable(bt_to_pcd_service src/bt_to_pcd_service.cpp)
ament_target_dependencies(bt_to_pcd_service rclcpp sensor_msgs)
target_link_libraries(bt_to_pcd_service ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS
  bt_to_pcd_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
