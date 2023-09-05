cmake_minimum_required(VERSION 3.8)
project(in_class_day03_cpp_solutions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(neato2_interfaces REQUIRED)

add_executable(emergency_stop src/emergency_stop.cpp)
target_include_directories(emergency_stop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(emergency_stop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  emergency_stop
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "neato2_interfaces"
)

add_executable(distance_emergency_stop src/distance_emergency_stop.cpp)
target_include_directories(distance_emergency_stop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(distance_emergency_stop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  distance_emergency_stop
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "neato2_interfaces"
)

install(TARGETS emergency_stop distance_emergency_stop
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
