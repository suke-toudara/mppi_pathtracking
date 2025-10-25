cmake_minimum_required(VERSION 3.8)
project(mppi_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Create mppi_planner library
ament_auto_add_library(mppi_planner SHARED
  src/mppi_component.cpp
  src/mppi_controller.cpp
)

# Create executable node
ament_auto_add_executable(mppi_node
  src/mppi_node.cpp
)

rclcpp_components_register_node(mppi_planner
  PLUGIN "mppi::MPPI"
  EXECUTABLE mppi_planner_component
)


# Link Eigen3
target_link_libraries(mppi_planner Eigen3::Eigen)
target_link_libraries(mppi_node mppi_planner)

# Register as component
rclcpp_components_register_nodes(mppi_planner "mppi::MPPI")


# Install
ament_auto_package(
  INSTALL_TO_SHARE
  USE_SCOPED_HEADER_INSTALL_DIR
)