cmake_minimum_required(VERSION 3.16)
project(odri_forward_command_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# --- Parameter library ---
generate_parameter_library(
  odri_forward_command_controller_parameters
  src/odri_forward_command_controller_parameters.yaml
)

# --- Controller library ---
add_library(${PROJECT_NAME} SHARED
  src/odri_forward_command_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/odri_forward_command_controller>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  odri_forward_command_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface odri_forward_command_controller_plugin.xml)

# --- Install ---
install(
  DIRECTORY include/
  DESTINATION include/odri_forward_command_controller
)
install(
  TARGETS
    ${PROJECT_NAME}
    odri_forward_command_controller_parameters
  EXPORT export_odri_forward_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_odri_forward_command_controller
    test/test_odri_forward_command_controller.cpp
  )
  target_include_directories(test_odri_forward_command_controller PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_odri_forward_command_controller
    ${PROJECT_NAME}
  )
endif()

ament_export_targets(export_odri_forward_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
