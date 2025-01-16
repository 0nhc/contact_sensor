cmake_minimum_required(VERSION 3.8)
project(contact_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables
add_executable(contact_sensor_node src/contact_sensor_node.cpp)
add_executable(visualizer_node src/visualizer_node.cpp)

# Add include directories
target_include_directories(contact_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Link dependencies
ament_target_dependencies(contact_sensor_node rclcpp std_msgs serial_driver)
ament_target_dependencies(visualizer_node rclcpp std_msgs visualization_msgs tf2_ros geometry_msgs)

# Set C++ standards
target_compile_features(contact_sensor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(visualizer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install the targets
install(TARGETS contact_sensor_node visualizer_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter for copyrights (remove for production use)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (remove for production use)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
