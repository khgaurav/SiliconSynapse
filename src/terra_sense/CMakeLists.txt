cmake_minimum_required(VERSION 3.8)
project(terra_sense)

set(lib_name ${PROJECT_NAME}_terrain_layer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rclpy
  std_msgs
  grid_map_ros
  grid_map_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

# Build the terrain layer as a component
add_library(${lib_name} SHARED
  src/terrain_layer.cpp)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${lib_name} ${dep_pkgs})

# Register component
rclcpp_components_register_node(${lib_name}
  PLUGIN "terra_sense::TerrainLayer"
  EXECUTABLE terrain_layer_node
)

# Install targets
install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/terrain_publisher.py
  scripts/speed_adjust.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dep_pkgs})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
