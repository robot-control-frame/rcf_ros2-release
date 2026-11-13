cmake_minimum_required(VERSION 3.16)
project(rcf_standard_hardware LANGUAGES CXX)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror -Wno-enum-compare)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rcf_hardware_interface
    rcf_standard_driver_interface
    rcf_joint_limits_interface
    transmission_interface
    rcf_transmission_interface
    yaml-cpp
    realtime_tools
    control_msgs
    std_srvs
    sensor_msgs
    rcf_common)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(rcf_standard_hardware SHARED src/rcf_standard_hardware.cpp
                                         src/topic_imu_hardware.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp yaml-cpp::yaml-cpp realtime_tools::realtime_tools
         rcf_joint_limits_interface::rcf_joint_limits_interface)
target_include_directories(
  rcf_standard_hardware
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/rcf_standard_hardware>)
ament_target_dependencies(rcf_standard_hardware PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

add_executable(servo_enable src/servo_enable.cpp)
ament_target_dependencies(servo_enable PUBLIC rclcpp std_srvs)

add_executable(servo_disable src/servo_disable.cpp)
ament_target_dependencies(servo_disable PUBLIC rclcpp std_srvs)

# INSTALL
install(DIRECTORY include/ DESTINATION include/rcf_standard_hardware)
install(
  TARGETS rcf_standard_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS servo_enable DESTINATION lib/${PROJECT_NAME})
install(TARGETS servo_disable DESTINATION lib/${PROJECT_NAME})

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
