cmake_minimum_required(VERSION 3.5)
project(teb_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(laserline REQUIRED)
find_package(wall_line_detection_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dwb_core REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(garage_utils_msgs REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(teb_local_planner SHARED
   src/timed_elastic_band.cpp
   src/optimal_planner.cpp
   src/obstacles.cpp
   src/visualization.cpp
   src/recovery_behaviors.cpp
   src/teb_config.cpp
   src/homotopy_class_planner.cpp
   src/teb_local_planner_ros.cpp
   src/teb_plan_window.cpp
   src/teb_obstacle_update.cpp
   src/teb_edge_line_update.cpp
   src/teb_edge_following.cpp
   src/teb_narrow_backward.cpp
   src/teb_costmap_params.cpp
   src/teb_path_hold.cpp
   src/teb_rotation.cpp
   src/teb_corner_collision_debug.cpp
   src/vehicle_scan_grid.cpp
   src/front_scan_protrusion.cpp
   src/wall_protrusion_check.cpp
   src/graph_search.cpp
   src/near_horizon_dynamic_safety.cpp
   src/path_dynamic_wait.cpp
   src/front_steering_plot.cpp
)

set(ament_dependencies
    costmap_converter
    costmap_converter_msgs
    sensor_msgs
    tf2_sensor_msgs
    dwb_critics
    nav_2d_utils
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    builtin_interfaces
    laserline
    wall_line_detection_msgs
    rclcpp_components
    dwb_core
    capella_ros_msg
    garage_utils_msgs
)
ament_target_dependencies(teb_local_planner
   ${ament_dependencies}
)
# Plugin .so is dlopen'd; link C++ typesupport explicitly so LaneCenterPaths
# rosidl_typesupport_cpp symbols are in DT_NEEDED (not undefined at runtime).
target_link_libraries(teb_local_planner
   ${EXTERNAL_LIBS}
   capella_ros_msg::capella_ros_msg__rosidl_typesupport_cpp
)

target_compile_definitions(teb_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(path_wait_test_node src/path_wait_test_node.cpp)
ament_target_dependencies(path_wait_test_node ${ament_dependencies})
target_link_libraries(path_wait_test_node teb_local_planner ${EXTERNAL_LIBS})

add_executable(split_modules_viz_node src/split_modules_viz_node.cpp)
target_include_directories(split_modules_viz_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
ament_target_dependencies(split_modules_viz_node ${ament_dependencies})
target_link_libraries(split_modules_viz_node teb_local_planner ${EXTERNAL_LIBS})

add_executable(front_steering_viz_node src/front_steering_viz_node.cpp)
ament_target_dependencies(front_steering_viz_node ${ament_dependencies})
target_link_libraries(front_steering_viz_node teb_local_planner ${EXTERNAL_LIBS})

add_executable(teb_main_flow_viz_node src/teb_main_flow_viz_node.cpp)
target_include_directories(teb_main_flow_viz_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  /opt/ros/humble/src/gtest_vendor/include)
ament_target_dependencies(teb_main_flow_viz_node ${ament_dependencies})
target_link_libraries(teb_main_flow_viz_node teb_local_planner ${EXTERNAL_LIBS})
target_compile_definitions(teb_main_flow_viz_node PRIVATE
  "TEB_PARAMS_YAML=\"${CMAKE_CURRENT_SOURCE_DIR}/params/teb_params.yaml\"")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS teb_local_planner
   DESTINATION lib
)

install(TARGETS path_wait_test_node split_modules_viz_node teb_main_flow_viz_node front_steering_viz_node
   DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  teb_local_planner_plugin.xml
  DESTINATION share
  )

install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params launch cfg DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(teb_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(nav2_core teb_local_planner_plugin.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_split_module_tests
    teb_plan_window_test
    teb_obstacle_update_test
    teb_edge_line_update_test
    teb_edge_following_test
    teb_narrow_backward_test
    teb_costmap_params_test
    teb_path_hold_test
    teb_rotation_test
    teb_corner_collision_debug_test
    teb_main_flow_test
    teb_front_steering_test
    teb_plan_curvature_test
  )
  foreach(_split_test ${_split_module_tests})
    ament_add_gtest(${_split_test} test/${_split_test}.cpp)
    target_link_libraries(${_split_test} teb_local_planner ${EXTERNAL_LIBS})
    ament_target_dependencies(${_split_test} ${ament_dependencies})
    target_compile_definitions(${_split_test} PRIVATE
      "TEB_PARAMS_YAML=\"${CMAKE_CURRENT_SOURCE_DIR}/params/teb_params.yaml\"")
  endforeach()
endif()

ament_package()