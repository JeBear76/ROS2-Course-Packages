cmake_minimum_required(VERSION 3.8)
project(assignment_turtle_sim_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(assignment_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_spawn src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn rclcpp assignment_interfaces turtlesim)

add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control rclcpp assignment_interfaces geometry_msgs turtlesim)

install(TARGETS
  turtle_spawn
  turtle_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
