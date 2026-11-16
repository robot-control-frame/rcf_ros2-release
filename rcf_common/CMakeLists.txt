cmake_minimum_required(VERSION 3.16)
project(rcf_common LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp Eigen3 geometry_msgs joint_limits
                                 hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

file(GLOB_RECURSE source "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${source})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} rclcpp::rclcpp Eigen3::Eigen geometry_msgs::geometry_msgs
  joint_limits::joint_limits hardware_interface::hardware_interface)

install(DIRECTORY include/ DESTINATION include/rcf_common)
install(
  TARGETS rcf_common
  EXPORT export_rcf_common
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_rcf_common HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
