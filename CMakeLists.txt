cmake_minimum_required(VERSION 3.16)
project(supre_robot_control LANGUAGES CXX)

#find_package(ros2_control_cmake REQUIRED)
#set_compiler_options()
#export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  ros2_controllers
  xacro
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  supre_robot_control
  SHARED
  hardware/eyou_system_interface.cpp
  hardware/misumi_gripper_hardware.cpp
)
find_library(MISUMI_GRIPPER_LIBRARY
 NAMES misumi_gripper_lib
 PATHS $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/lib>
 REQUIRED
)

find_library(EUMOTOR_LIBRARY
  NAMES eu_motor  # CMake automatically adds 'lib' prefix and '.so' suffix
  PATHS $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/lib>
)

pkg_check_modules(MODBUS REQUIRED libmodbus)


target_compile_features(supre_robot_control PUBLIC cxx_std_17)
target_include_directories(supre_robot_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/supre_robot_control>
${MODBUS_INCLUDE_DIRS}
)
target_link_libraries(supre_robot_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${MISUMI_GRIPPER_LIBRARY} 
  ${EUMOTOR_LIBRARY}
  ${MODBUS_LIBRARIES} 
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface supre_robot_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/supre_robot_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/supre_robot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/supre_robot_control
)
install(TARGETS supre_robot_control
  EXPORT export_supre_robot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_supre_robot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()