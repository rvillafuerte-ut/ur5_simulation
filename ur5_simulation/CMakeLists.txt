cmake_minimum_required(VERSION 3.8)
project(ur5_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLOG REQUIRED)
find_package(pinocchio REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ur5_kinematics REQUIRED) # AÑADIR ESTA LÍNEA
find_package(ur5_impedance REQUIRED) # AÑADIR ESTA LÍNEA



include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/share)
include_directories(/usr/include/modbus)
# Definir ejecutables
set(EXECUTABLES       
    ur5_ik_node
    ur5_impedance
)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

foreach(EXEC ${EXECUTABLES})
    add_executable(${EXEC} ${SRC_DIR}/${EXEC}.cpp)
    target_include_directories(${EXEC} 
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(${EXEC} 
        rclcpp 
        trajectory_msgs 
        sensor_msgs
        omni_msgs
        geometry_msgs
        ur5_kinematics
        ur5_impedance
    )
endforeach()

set(EXECUTABLES_WITH_LIBRARIES
    ur5_ik_node
    ur5_impedance
)

set(COMMON_LIBRARIES
    pinocchio::pinocchio
    osqp::osqp
    OsqpEigen::OsqpEigen
    modbus
)

foreach(EXEC ${EXECUTABLES_WITH_LIBRARIES})
    target_link_libraries(${EXEC} ${COMMON_LIBRARIES})
endforeach()

install(DIRECTORY 
    config 
    launch include urdf 
    rviz src meshes worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)


install(TARGETS ${EXECUTABLES} DESTINATION lib/${PROJECT_NAME})


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
ament_export_dependencies(rclcpp)
ament_package()
