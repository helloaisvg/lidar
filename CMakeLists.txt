cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

## 编译标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

## 系统依赖
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

## 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 链接目录
link_directories(${PCL_LIBRARY_DIRS})

## 添加定义
add_definitions(${PCL_DEFINITIONS})

## 源文件
set(SOURCES
  src/ndt_registration.cpp
  src/gn_icp_registration.cpp
  src/lidar_odometry.cpp
  src/imu_integration.cpp
  src/ros_odometry_node.cpp
  src/trajectory_evaluator.cpp
)

## 头文件
set(HEADERS
  include/lidar_odometry/ndt_registration.h
  include/lidar_odometry/gn_icp_registration.h
  include/lidar_odometry/lidar_odometry.h
  include/lidar_odometry/imu_integration.h
  include/lidar_odometry/ros_odometry_node.h
  include/lidar_odometry/trajectory_evaluator.h
)

## 创建库
add_library(${PROJECT_NAME}_lib ${SOURCES} ${HEADERS})

## 链接库
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

## 可执行文件
add_executable(ndt_odometry src/ndt_odometry.cpp)
target_link_libraries(ndt_odometry ${PROJECT_NAME}_lib)

add_executable(icp_odometry src/icp_odometry.cpp)
target_link_libraries(icp_odometry ${PROJECT_NAME}_lib)

add_executable(ros_odometry_node src/ros_odometry_node.cpp)
target_link_libraries(ros_odometry_node ${PROJECT_NAME}_lib)

add_executable(trajectory_evaluator src/trajectory_evaluator.cpp)
target_link_libraries(trajectory_evaluator ${PROJECT_NAME}_lib)

## 安装
install(TARGETS ${PROJECT_NAME}_lib ndt_odometry icp_odometry ros_odometry_node trajectory_evaluator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

## ament包配置
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions pcl_ros)

ament_package()
