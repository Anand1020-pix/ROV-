cmake_minimum_required(VERSION 3.8)
project(rov_underwater_control)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(rov_thruster_controller src/rov_thruster_controller.cpp)
ament_target_dependencies(rov_thruster_controller
  rclcpp rclcpp_lifecycle geometry_msgs std_msgs)

add_executable(sediment_collection_server src/sediment_collection_server.cpp)
ament_target_dependencies(sediment_collection_server
  rclcpp rclcpp_action rclcpp_lifecycle geometry_msgs std_msgs rov_interfaces)

add_executable(collection_sensor_sim src/collection_sensor_sim.cpp)
ament_target_dependencies(collection_sensor_sim rclcpp std_msgs)

add_executable(rov_sim_bridge src/rov_sim_bridge.cpp)
ament_target_dependencies(rov_sim_bridge
  rclcpp geometry_msgs tf2_msgs rov_interfaces)
target_link_libraries(rov_sim_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

add_executable(rov_teleop src/rov_teleop.cpp)
ament_target_dependencies(rov_teleop rclcpp geometry_msgs std_msgs)

install(TARGETS
  rov_thruster_controller
  sediment_collection_server
  collection_sensor_sim
  rov_sim_bridge
  rov_teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()
