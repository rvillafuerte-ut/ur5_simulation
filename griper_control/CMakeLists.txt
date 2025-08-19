cmake_minimum_required(VERSION 3.8)
project(griper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(/usr/include/modbus)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(gripper ${SRC_DIR}/gripper.cpp)
ament_target_dependencies(gripper 
        rclcpp 
        sensor_msgs
        omni_msgs
    )


target_link_libraries(gripper modbus)
install(DIRECTORY 
    src DESTINATION share/${PROJECT_NAME})
    
install(TARGETS gripper
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
