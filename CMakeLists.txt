cmake_minimum_required(VERSION 3.5)
project(faruk)

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
find_package(xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(dependencies "geometry_msgs" "rclcpp" "sensor_msgs")
set(ROBOT_MOVE_NAME cpp_robot_controller)
add_executable(${ROBOT_MOVE_NAME} src/cpp_robot_controller.cpp)
target_link_libraries(${ROBOT_MOVE_NAME} Qt5::Widgets)
ament_target_dependencies(${ROBOT_MOVE_NAME} ${dependencies})

install(
  DIRECTORY src launch rviz config world
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${ROBOT_MOVE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
