cmake_minimum_required(VERSION 3.8)
project(rplidar_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

file(GLOB SDK_SOURCES 
    "src/sdk/src/*.cpp"
    "src/sdk/src/hal/*.cpp"
    "src/sdk/src/dataunpacker/*.cpp"
    "src/sdk/src/dataunpacker/unpacker/*.cpp"
    "src/sdk/src/arch/linux/*.cpp" 
)

add_library(rplidar_sdk STATIC ${SDK_SOURCES})

target_include_directories(rplidar_sdk PUBLIC
  src/sdk/include
  src/sdk/src
)

add_executable(rplidar_node
  src/rplidar_node.cpp
  src/lidar_driver_wrapper.cpp 
)
target_include_directories(rplidar_node PRIVATE include)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(rplidar_node PRIVATE -Wall -Wextra -Wpedantic)
endif()


target_link_libraries(rplidar_node rplidar_sdk)

ament_target_dependencies(rplidar_node
  rclcpp
  sensor_msgs
  std_srvs
  rclcpp_lifecycle
  rclcpp_components
)

install(TARGETS 
  rplidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()