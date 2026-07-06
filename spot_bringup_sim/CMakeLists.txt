cmake_minimum_required(VERSION 3.8)
project(spot_bringup_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_transform src/pointcloud_transform.cpp)
ament_target_dependencies(pointcloud_transform
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)

add_executable(contact_adapter_node src/contact_adapter_node.cpp)
ament_target_dependencies(contact_adapter_node
  rclcpp
  champ_msgs
  ros_gz_interfaces
)

install(TARGETS pointcloud_transform
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS contact_adapter_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/thermal_to_rgb.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME thermal_to_rgb
)

install(PROGRAMS
  scripts/stow_arm.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stow_arm
)

install(PROGRAMS
  scripts/stow_arm_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stow_arm_client
)

install(PROGRAMS
  scripts/sim_manipulation_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_manipulation_driver
)

install(DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
