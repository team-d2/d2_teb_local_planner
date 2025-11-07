cmake_minimum_required(VERSION 3.14)
project(d2_teb_local_planner)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
  set(OpenGL_GL_PREFERENCE GLVND)
endif()

if(NOT G2O_SOLVER_CHOLMOD_LIB)
  list(REMOVE_ITEM G2O_LIBRARIES ${G2O_SOLVER_CHOLMOD_LIB})
endif()

# Set to Release in order to speed up the program significantly
# set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

  if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_definitions(NDEBUG)
  endif()

  message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters segmentation io)
find_package(d2_costmap_converter REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
# find_package(Boost REQUIRED)

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

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${PCL_LIBRARIES})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(d2_teb_local_planner SHARED
   src/timed_elastic_band.cpp
   src/optimal_planner.cpp
   src/obstacles.cpp
   src/visualization.cpp
   src/recovery_behaviors.cpp
   src/teb_config.cpp
   src/homotopy_class_planner.cpp
   src/teb_local_planner_ros.cpp
   src/graph_search.cpp
)

set(ament_dependencies
    d2_costmap_converter
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
)
ament_target_dependencies(d2_teb_local_planner
   ${ament_dependencies}
)
target_link_libraries(d2_teb_local_planner
   ${EXTERNAL_LIBS}
)

target_compile_definitions(d2_teb_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#------------------------------------------------------------------------------------------
# Obstacle Fusion Component
add_library(obs_fusion_component SHARED
  src/component/obs_fusion_component.cpp
)

set(obs_fusion_dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  d2_costmap_converter_msgs
  d2_costmap_converter
  pluginlib
  pcl_conversions
  nav2_costmap_2d
)

ament_target_dependencies(obs_fusion_component
  ${obs_fusion_dependencies}
)

target_include_directories(obs_fusion_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(obs_fusion_component
  d2_teb_local_planner
  ${PCL_LIBRARIES}
)

target_compile_definitions(obs_fusion_component PUBLIC ${PCL_DEFINITIONS})

rclcpp_components_register_nodes(obs_fusion_component "d2_teb_local_planner::ObstacleFusionComponent")

install(TARGETS obs_fusion_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#------------------------------------------------------------------------------------------
# TEB Planner Standalone Component
add_library(teb_planner_component SHARED
  src/component/teb_planner_component.cpp
)

set(teb_planner_dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  d2_costmap_converter_msgs
)

ament_target_dependencies(teb_planner_component
  ${teb_planner_dependencies}
)

target_link_libraries(teb_planner_component
  d2_teb_local_planner
)

rclcpp_components_register_nodes(teb_planner_component "d2_teb_local_planner::TebMotionStandaloneComponent")

install(TARGETS teb_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#------------------------------------------------------------------------------------------

#############
## Install ##
#############
# Demo / test optimization node (ROS2 port of legacy test_optim_node)
# add_executable(test_optim_node
#   src/test_optim_node.cpp
# )
# ament_target_dependencies(test_optim_node
#   rclcpp nav2_util geometry_msgs nav_msgs interactive_markers
# )
# target_link_libraries(test_optim_node
#   d2_teb_local_planner
# )

# install(TARGETS d2_teb_local_planner test_optim_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

## Mark executables and/or libraries for installation
install(TARGETS d2_teb_local_planner
   DESTINATION lib
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  d2_teb_local_planner_plugin.xml
  DESTINATION share
  )

install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(d2_teb_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(nav2_core d2_teb_local_planner_plugin.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(teb_basics_test
      test/teb_basics.cpp
  )
  target_link_libraries(teb_basics_test
      d2_teb_local_planner
  )

  # ament_add_gtest(homotopy_class_planner_test
  #     test/homotopy_class_planner_test.cpp
  # )
  # target_link_libraries(homotopy_class_planner_test
  #     d2_teb_local_planner
  # )
endif()

ament_package()
