cmake_minimum_required(VERSION 3.8)
project(ur5_dynamics_pinocchio)

# C++17 es necesario para Pinocchio moderno
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Importante para compatibilidad

# Warnings útiles
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Nuevo
find_package(std_msgs REQUIRED)     # Nuevo
find_package(Eigen3 3.4 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED) 

# Configuración de include
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}  # Asegura acceso a headers de Pinocchio
)

# Tu ejecutable principal
add_executable(ur5_control_node src/ur5_dynamics_node.cpp)  # Nombre más descriptivo

# Dependencias de ament
ament_target_dependencies(ur5_control_node
  rclcpp
  pinocchio
  Eigen3
  ament_index_cpp
  trajectory_msgs
  yaml-cpp
  sensor_msgs    # Nuevo
  std_msgs       # Nuevo
)

# Enlazado de bibliotecas
target_link_libraries(ur5_control_node
  pinocchio::pinocchio
  yaml-cpp
  ${EIGEN3_LIBRARIES}
)

# Instalación
install(TARGETS
  ur5_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  configs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)




# Descubrimiento de dependencias
ament_export_dependencies(
  rclcpp
  pinocchio
  Eigen3
  ament_index_cpp
  trajectory_msgs
  sensor_msgs
  std_msgs
  yaml-cpp
)

# Linters y tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
