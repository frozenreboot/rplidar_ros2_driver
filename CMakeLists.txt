cmake_minimum_required(VERSION 3.16)
project(rplidar_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# =========================================================
# 1. Find Dependencies
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# =========================================================
file(
  GLOB
  SDK_SOURCES
  "src/sdk/src/*.cpp"
  "src/sdk/src/hal/*.cpp"
  "src/sdk/src/dataunpacker/*.cpp"
  "src/sdk/src/dataunpacker/unpacker/*.cpp"
  "src/sdk/src/arch/linux/*.cpp")

add_library(rplidar_sdk STATIC ${SDK_SOURCES})
set_target_properties(rplidar_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(rplidar_sdk PUBLIC src/sdk/include src/sdk/src)

# =========================================================
# 1. Build Component (Shared Library)
# =========================================================
# Create a shared library containing the node logic. This allows the node to be
# loaded into a container process for zero-copy communication.
add_library(rplidar_node_component SHARED src/rplidar_node.cpp
                                          src/lidar_driver_wrapper.cpp)

target_include_directories(
  rplidar_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Link against the RPLIDAR SDK
target_link_libraries(rplidar_node_component PUBLIC rplidar_sdk)

# Link ROS 2 dependencies
target_link_libraries(
  rplidar_node_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    diagnostic_updater::diagnostic_updater
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${std_srvs_TARGETS})

# Register the component to make it discoverable by rclcpp_components
rclcpp_components_register_node(rplidar_node_component
  PLUGIN "rplidar_driver::RPlidarNode"
  EXECUTOR "SingleThreadedExecutor"
  EXECUTABLE "rplidar_node")

# =========================================================
# 1. Install Rules
# =========================================================
# Install the shared library (Component)
install(
  TARGETS rplidar_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files, parameters, and configurations
install(DIRECTORY launch param config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Linting Tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  #Google Test (Unit Test) Setup
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  ament_add_gtest(test_rplidar_wrapper test/test_rplidar_wrapper.cpp)
  target_include_directories(test_rplidar_wrapper PUBLIC include)
  target_link_libraries(
    test_rplidar_wrapper
    rplidar_node_component
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs)

  ament_add_gtest(test_rplidar_node test/test_rplidar_node.cpp)
  target_include_directories(test_rplidar_node PUBLIC include)
  target_link_libraries(test_rplidar_node rplidar_node_component)
endif()
ament_package()
