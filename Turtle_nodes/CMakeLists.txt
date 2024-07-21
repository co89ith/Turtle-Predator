cmake_minimum_required(VERSION 3.8)
project(turtle_predator_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner turtlesim rclcpp my_robot_interfaces)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller turtlesim geometry_msgs rclcpp my_robot_interfaces)


install(TARGETS
turtle_spawner
turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
