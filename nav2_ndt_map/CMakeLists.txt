cmake_minimum_required(VERSION 3.8)
project(nav2_ndt_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs)

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(ndt_map_2d_lib SHARED src/ndt_map_2d.cpp include/nav2_ndt_map/ndt_map_2d.hpp)
ament_target_dependencies(ndt_map_2d_lib pcl_ros nav2_msgs)
ament_export_targets(ndt_map_2d_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(pcl_ros)

add_executable(mapper src/mapper.cpp)
ament_target_dependencies(mapper
  "rclcpp"
  std_srvs
  sensor_msgs
  geometry_msgs
  laser_geometry
  pcl_ros
  nav2_msgs
)
target_link_libraries(mapper ndt_map_2d_lib)

add_executable(ndt_to_occ src/ndt_to_occ.cpp)
ament_target_dependencies(ndt_to_occ
  "rclcpp"
  nav_msgs
  nav2_msgs
)
target_link_libraries(ndt_to_occ ndt_map_2d_lib)

ament_export_include_directories(include)

install(
  TARGETS ndt_map_2d_lib
  EXPORT ndt_map_2d_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ndt_to_occ
  mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
