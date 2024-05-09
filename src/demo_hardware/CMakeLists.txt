cmake_minimum_required(VERSION 3.8)
project(demo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

# COMPILE
add_library(demo_hardware SHARED
  src/demobot_hardware_test.cpp
  # src/demobot_hardware.cpp
  src/wheel.cpp
  # src/MotorDriver.c
  # src/motor_encoder.c
  # src/PCA9685.c
  # src/DEV_Config.c
)

target_include_directories(demo_hardware PRIVATE include/demo_hardware)

ament_target_dependencies(demo_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link WiringPi library to lidarbot_hardware target
# target_link_libraries(demo_hardware wiringPi)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface demo_hardware.xml)

# INSTALL
install(
  DIRECTORY include/demo_hardware/
  DESTINATION include/demo_hardware
)

install(
  TARGETS demo_hardware
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# EXPORTS 
ament_export_libraries(demo_hardware)

ament_package()
