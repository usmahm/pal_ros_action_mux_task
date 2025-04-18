cmake_minimum_required(VERSION 3.8)
project(generic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(gen_subber_cpp src/generic_subscriber.cpp)
ament_target_dependencies(gen_subber_cpp rclcpp std_msgs)

add_executable(gen_tester_int_cpp src/generic_tester_int_pub.cpp)
ament_target_dependencies(gen_tester_int_cpp rclcpp std_msgs)

add_executable(gen_tester_string_cpp src/generic_tester_string_pub.cpp)
ament_target_dependencies(gen_tester_string_cpp rclcpp std_msgs)

install(TARGETS
  gen_subber_cpp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  gen_tester_int_cpp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  gen_tester_string_cpp
  DESTINATION lib/${PROJECT_NAME})

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
