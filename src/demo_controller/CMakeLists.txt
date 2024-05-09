cmake_minimum_required(VERSION 3.8)
project(demo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  pluginlib
  realtime_tools
  demo_interfaces
)

message("BUILD_INTERFACE: $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>")

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(demo_parameters
  src/demo_parameters.yaml
)

add_library(demo_controller SHARED
  src/demo_controller.cpp
)
# target_compile_features(demo_controller PUBLIC cxx_std_17)
# target_include_directories(demo_controller PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/demo_controller>
#   $<INSTALL_INTERFACE:include/demo_controller>
# )
target_include_directories(demo_controller PRIVATE include/demo_controller)
target_link_libraries(demo_controller PRIVATE demo_parameters)
ament_target_dependencies(
  demo_controller PUBLIC
  ${PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(controller_interface demo_controller_plugin.xml)

install(
  DIRECTORY include/demo_controller
  DESTINATION include/demo_controller
)
install(TARGETS demo_controller demo_parameters
  EXPORT export_demo_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_demo_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})
ament_package()
