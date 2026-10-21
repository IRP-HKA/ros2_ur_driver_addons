cmake_minimum_required(VERSION 3.5)
project(cartesian_twist_passthrough_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Set build type to "Release" by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_lifecycle
        pluginlib
        controller_interface
        hardware_interface
        realtime_tools
        geometry_msgs
        generate_parameter_library
        Eigen3
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


generate_parameter_library(
  cartesian_twist_passthrough_controller_parameters
  src/cartesian_twist_passthrough_controller_parameters.yaml
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/cartesian_twist_passthrough_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  cartesian_twist_passthrough_controller_parameters
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface cartesian_twist_passthrough_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
