cmake_minimum_required(VERSION 3.8)
project(ublox_driver_ros2)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  ${PROJECT_SOURCE_DIR}/third_party/
)

add_executable(${PROJECT_NAME} src/ublox_driver.cpp 
                               src/ublox_message_processor.cpp
                               src/file_loader.cpp
                               src/file_dumper.cpp
                               src/serial_handler.cpp
                               src/socket_handler.cpp
                               third_party/yaml/Yaml.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  Eigen3
  Glog
)

#同步时间节点
add_executable(sync_system_time src/sync_system_time.cpp)
ament_target_dependencies(sync_system_time
  rclcpp
  std_msgs
)

install(TARGETS ${PROJECT_NAME} sync_system_time
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
  