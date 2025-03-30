cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)  # Added for working with occupancy grid
find_package(nav2_bringup REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Added for Python scripts
find_package(rclpy REQUIRED)  # Added for Python nodes

# Add a C++ executable if needed
# add_executable(pub_occupancy_grid src/pub_occupancy_grid.cpp)
# ament_target_dependencies(pub_occupancy_grid rclcpp nav_msgs)

# Install C++ executable
# install(TARGETS
#   pub_occupancy_grid
#   DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

# install(PROGRAMS scripts/single_goal_nav.py
#         DESTINATION lib/${PROJECT_NAME})

# install(PROGRAMS scripts/multi_waypoints.py
#         DESTINATION lib/${PROJECT_NAME})

# Enable linting if testing is enabled
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
