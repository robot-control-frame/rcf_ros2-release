cmake_minimum_required(VERSION 3.10)
project(rcf_fake_driver)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcf_standard_driver_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
# ##############################################################################
# # Build ## #
# ##############################################################################

add_library(${PROJECT_NAME} SHARED src/rcf_fake_driver.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} rclcpp::rclcpp yaml-cpp::yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp pluginlib
                          rcf_standard_driver_interface)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
pluginlib_export_plugin_description_file(rcf_standard_driver_interface
                                         plugins.xml)

ament_package()
