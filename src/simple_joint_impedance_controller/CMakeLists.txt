cmake_minimum_required(VERSION 3.16)
project(simple_joint_impedance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(
  simple_joint_impedance_controller_parameters
  src/simple_joint_impedance_controller.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/simple_joint_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    simple_joint_impedance_controller_parameters
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    sensor_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface simple_joint_impedance_controller.xml
)

install(
  TARGETS ${PROJECT_NAME} simple_joint_impedance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  generate_parameter_library
  sensor_msgs
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_math tests/test_math.cpp)
  target_include_directories(test_math PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
