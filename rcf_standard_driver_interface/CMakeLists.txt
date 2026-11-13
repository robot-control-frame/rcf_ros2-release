cmake_minimum_required(VERSION 3.10)
project(rcf_standard_driver_interface)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
# ##############################################################################
# Build ##
# ##############################################################################
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE rclcpp::rclcpp)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
# ##############################################################################
# Install ##
# ##############################################################################

ament_package()
