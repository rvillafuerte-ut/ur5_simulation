cmake_minimum_required(VERSION 3.8)
project(ur5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ur5_kinematics REQUIRED)
find_package(ur5_impedance REQUIRED)
find_package(ur5_sliding REQUIRED)
find_package(geomagic_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Agregar ejecutable para el controller_backup (controlador original)
add_executable(controller_backup 
  src/controller_node_backup.cpp
)

target_include_directories(controller_backup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(controller_backup
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  omni_msgs
  std_msgs
  pinocchio
  Eigen3
  ament_index_cpp
  ur5_kinematics
  ur5_impedance
  ur5_sliding
)

target_link_libraries(controller_backup
  ur5_kinematics::ur5_kinematics
  ur5_impedance::ur5_impedance
  ur5_sliding::ur5_sliding
)

install(TARGETS controller_backup
  DESTINATION lib/${PROJECT_NAME}
)

# Nuevo ejecutable: manual_ik_node
add_executable(manual_ik_node
  src/manual_ik_node.cpp
)

target_include_directories(manual_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(manual_ik_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  pinocchio
  Eigen3
  ament_index_cpp
  ur5_kinematics
)

target_link_libraries(manual_ik_node
  ur5_kinematics::ur5_kinematics
)

install(TARGETS manual_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ur5_pos 
  src/ur5_pos.cpp
)

target_include_directories(ur5_pos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Configurar dependencias del ejecutable
ament_target_dependencies(ur5_pos
  rclcpp
  trajectory_msgs
  ament_index_cpp
)

# Instalar el ejecutable
install(TARGETS ur5_pos
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
