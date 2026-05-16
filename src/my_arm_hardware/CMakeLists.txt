cmake_minimum_required(VERSION 3.8)
project(my_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(my_arm_hardware SHARED
  src/esp32_serial_hardware.cpp
)

target_include_directories(my_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_arm_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(my_arm_hardware Boost::system)

pluginlib_export_plugin_description_file(hardware_interface my_arm_hardware_plugins.xml)

install(TARGETS my_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_arm_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(my_arm_hardware)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib Boost)

ament_package()
