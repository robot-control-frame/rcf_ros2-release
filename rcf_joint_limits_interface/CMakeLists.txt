cmake_minimum_required(VERSION 3.16)
project(rcf_joint_limits_interface)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror -Wno-enum-compare)

set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface rclcpp joint_limits
                                 rcf_hardware_interface)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/rcf_joint_limits.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC hardware_interface::hardware_interface rclcpp::rclcpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
