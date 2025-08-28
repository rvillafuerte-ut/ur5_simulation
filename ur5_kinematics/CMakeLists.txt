cmake_minimum_required(VERSION 3.8)
project(ur5_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)


# Crear la biblioteca
add_library(${PROJECT_NAME} SHARED
  src/kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PINOCCHIO_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PINOCCHIO_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

# Instalar la biblioteca, cabeceras y recursos
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/
)

# Exportar para que otros paquetes puedan encontrarlo
ament_export_dependencies(pinocchio Eigen3 OsqpEigen)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

export(EXPORT export_${PROJECT_NAME}
  FILE "${PROJECT_NAME}Config.cmake"
  NAMESPACE ur5_kinematics::
)

install(
  FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake"
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
