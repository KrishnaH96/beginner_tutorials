# Minimum required CMake version
cmake_minimum_required(VERSION 3.8)

# Project name
project(first_ros_package)

# Check if the C++ compiler is GNU C++ or Clang, and add compile options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Adding executables
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"

  DEPENDENCIES std_msgs 
)

rosidl_target_interfaces(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_include_directories(talker PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp)

# rosidl_get_typesupport_target(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_include_directories(talker PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp)
# target_link_libraries(talker ${rosidl_runtime_cpp_LIBRARIES})

# Install executables to the 'lib' directory of the project
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Enable testing if BUILD_TESTING is set
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license are added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Define this package as an Ament package
ament_package()
