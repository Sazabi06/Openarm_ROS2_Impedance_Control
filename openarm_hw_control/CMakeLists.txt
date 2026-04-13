cmake_minimum_required(VERSION 3.8)
project(openarm_hw_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -Wno-error=cpp -Wno-error)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(openarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Pinocchio for dynamics
find_package(pinocchio REQUIRED)

include_directories(include)

# 1. Thermal Watchdog Node (Non-RT)
add_executable(thermal_watchdog src/thermal_watchdog.cpp)
target_link_libraries(thermal_watchdog pinocchio::pinocchio)
ament_target_dependencies(thermal_watchdog rclcpp std_msgs sensor_msgs openarm_msgs std_srvs)

# 2. Impedance Controller Plugin (RT)
add_library(openarm_impedance_controller SHARED src/impedance_controller.cpp)
target_link_libraries(openarm_impedance_controller pinocchio::pinocchio)
target_compile_definitions(openarm_impedance_controller PRIVATE "OPENARM_IMPEDANCE_CONTROLLER_BUILDING_LIBRARY")
ament_target_dependencies(openarm_impedance_controller rclcpp controller_interface hardware_interface pluginlib realtime_tools openarm_msgs)
pluginlib_export_plugin_description_file(controller_interface openarm_impedance_controller.xml)

install(TARGETS thermal_watchdog DESTINATION lib/${PROJECT_NAME})
install(TARGETS openarm_impedance_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES openarm_impedance_controller.xml DESTINATION share/${PROJECT_NAME})

ament_package()
