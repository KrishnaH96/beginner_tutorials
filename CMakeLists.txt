# Minimum required CMake version
cmake_minimum_required(VERSION 3.8)

# Project name
project(first_ros_package)

# Check if the C++ compiler is GNU C++ or Clang, and add compile options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

 # Enable compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Adding executables
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"

  DEPENDENCIES std_msgs 
)

rosidl_target_interfaces(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables to the 'lib' directory of the project
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

# Enable testing if BUILD_TESTING is set
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  ament_add_gtest(${PROJECT_NAME}_tutorial_test test/talker_test.cpp)
  ament_lint_auto_find_test_dependencies()
  ament_target_dependencies(${PROJECT_NAME}_tutorial_test rclcpp)
  ament_export_dependencies(rosidl_default_runtime)

  rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${PROJECT_NAME}_tutorial_test "${cpp_typesupport_target}")

  endif()

# Define this package as an Ament package
ament_package()
