cmake_minimum_required(VERSION 3.8)
project(geomagic_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(geomagic_processor_node src/geomagic_processor.cpp)

target_include_directories(geomagic_processor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(geomagic_processor_node
  rclcpp
  geometry_msgs
  std_msgs
  omni_msgs
  Eigen3
)

# Install executable
install(TARGETS geomagic_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
