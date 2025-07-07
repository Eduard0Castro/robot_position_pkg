cmake_minimum_required(VERSION 3.8)
project(robot_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_position_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)


add_executable(robot_position_server src/robot_position_server)
ament_target_dependencies(robot_position_server rclcpp robot_position_interface rclcpp_action rclcpp_lifecycle rcl_interfaces)
add_executable(robot_position_client src/robot_position_client)
ament_target_dependencies(robot_position_client rclcpp robot_position_interface rclcpp_action rclcpp_lifecycle)
add_executable(startup src/startup)
ament_target_dependencies(startup rclcpp lifecycle_msgs)

install(TARGETS

  robot_position_server
  robot_position_client
  startup
  
  DESTINATION lib/${PROJECT_NAME}
)

install (

  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/

)

ament_package()
