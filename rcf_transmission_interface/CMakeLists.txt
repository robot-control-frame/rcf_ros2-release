cmake_minimum_required(VERSION 3.16)
project(rcf_transmission_interface LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp fmt
                                 transmission_interface yaml-cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(rcf_transmission_interface SHARED
            src/rcf_simple_transmission_loader.cpp)
target_include_directories(
  rcf_transmission_interface
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/rcf_transmission_interface>)
target_link_libraries(
  rcf_transmission_interface
  PUBLIC hardware_interface::hardware_interface pluginlib::pluginlib
         rclcpp::rclcpp fmt::fmt transmission_interface::transmission_interface
         yaml-cpp::yaml-cpp)

pluginlib_export_plugin_description_file(transmission_interface plugins.xml)

install(DIRECTORY include/ DESTINATION include/rcf_transmission_interface)
install(
  TARGETS rcf_transmission_interface
  EXPORT export_transmission_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_transmission_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
