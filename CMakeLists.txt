cmake_minimum_required(VERSION 3.8)
project(rogilinkflex2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceData.msg"
  "msg/ReadRequest.msg"
  "msg/WriteRequest.msg"
  "msg/PeriodicData.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Add executables
add_executable(${PROJECT_NAME}_node src/main.cpp src/communication_node.cpp)
add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_test_node
  rclcpp
  std_msgs
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node ${JSONCPP_LIBRARIES} yaml-cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${JSONCPP_INCLUDE_DIRS})

# Rosidl target dependencies
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME}_test_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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
