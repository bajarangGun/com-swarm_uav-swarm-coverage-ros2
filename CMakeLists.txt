cmake_minimum_required(VERSION 3.8)
project(con_plan_exec_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellInfo.msg"
  "msg/DiscreteClock.msg"
  "msg/PlanInstance.msg"
  "srv/PlanForHorizon.srv"
  "srv/ShareLocalInformation.srv"
  "srv/StopRobot.srv"
  DEPENDENCIES std_msgs
)
# Add the include directory so the compiler can find basics.h and config.h
include_directories(include)

# Create a C++ library out of the pure math files
add_library(concpp_math_lib
  src/gap.cpp
  src/ma.cpp
)

# Tell the library it needs the custom ROS 2 messages we just built
ament_target_dependencies(concpp_math_lib
  rclcpp
  std_msgs
  ament_index_cpp
  
)
# Install the input maps so the runtime nodes can find them
install(DIRECTORY input
  DESTINATION share/${PROJECT_NAME}
)
# Build the robot node
add_executable(robot_node src/robot.cpp)

# Tell the robot node it needs external ROS 2 packages
ament_target_dependencies(robot_node
  rclcpp
  std_msgs
  ament_index_cpp
  mavros_msgs
  geometry_msgs
)

# Get the typesupport target for our custom messages (Jazzy standard)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Link the math library AND the custom messages
target_link_libraries(robot_node concpp_math_lib "${cpp_typesupport_target}")

# Install the executable so we can run it with 'ros2 run'
install(TARGETS
  robot_node
  DESTINATION lib/${PROJECT_NAME}
)
# Build the central server node
add_executable(concpp_node src/concpp_main.cpp src/rviz.cpp)

# Tell the server it needs external packages
ament_target_dependencies(concpp_node
  rclcpp
  std_msgs
  ament_index_cpp
  visualization_msgs
)

# Link the custom messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(concpp_node concpp_math_lib "${cpp_typesupport_target}")

# Install the server executable
install(TARGETS
  concpp_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install the shared math library so the nodes can find it at runtime
install(TARGETS concpp_math_lib
  DESTINATION lib
)
# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
