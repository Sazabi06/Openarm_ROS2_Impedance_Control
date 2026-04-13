cmake_minimum_required(VERSION 3.8)
project(openarm_compliance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(openarm_compliance_controller SHARED
  src/compliance_controller.cpp
)
target_link_libraries(openarm_compliance_controller ${orocos_kdl_LIBRARIES})
ament_target_dependencies(openarm_compliance_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  kdl_parser
  orocos_kdl_vendor
  std_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface openarm_compliance_controller.xml)

install(TARGETS openarm_compliance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/impedance_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(openarm_compliance_controller)
ament_export_include_directories(include)
ament_package()
