cmake_minimum_required(VERSION 3.16)
project(factory_ops_goal_checker)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(factory_ops_goal_checker SHARED
  src/telemetry_goal_checker.cpp
  src/telemetry_regulated_pure_pursuit_controller.cpp)
target_compile_features(factory_ops_goal_checker PUBLIC cxx_std_17)
target_include_directories(factory_ops_goal_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(factory_ops_goal_checker
  diagnostic_msgs nav2_controller nav2_regulated_pure_pursuit_controller pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS factory_ops_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_telemetry_goal_checker test/test_telemetry_goal_checker.cpp)
  target_link_libraries(test_telemetry_goal_checker factory_ops_goal_checker)
  ament_target_dependencies(test_telemetry_goal_checker
    diagnostic_msgs nav2_controller rclcpp rclcpp_lifecycle)
  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  add_dependencies(test_plugin_loading factory_ops_goal_checker)
  target_compile_definitions(test_plugin_loading PRIVATE
    FACTORY_OPS_GOAL_CHECKER_LIBRARY="$<TARGET_FILE:factory_ops_goal_checker>")
  ament_target_dependencies(test_plugin_loading class_loader nav2_core)
  ament_add_gtest(test_telemetry_rpp test/test_telemetry_rpp.cpp)
  target_link_libraries(test_telemetry_rpp factory_ops_goal_checker)
  ament_target_dependencies(test_telemetry_rpp
    nav2_costmap_2d nav2_regulated_pure_pursuit_controller rclcpp rclcpp_lifecycle tf2_ros)
endif()

ament_export_include_directories(include)
ament_export_libraries(factory_ops_goal_checker)
ament_export_dependencies(diagnostic_msgs nav2_controller nav2_regulated_pure_pursuit_controller pluginlib rclcpp rclcpp_lifecycle)
ament_package()
