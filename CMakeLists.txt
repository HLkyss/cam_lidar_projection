cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_projection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  #cv_bridge
  pcl_ros
  pcl_conversions
)

set(cv_bridge_DIR /home/hl/Downloads/vision_opencv-noetic/cv_bridge/build/devel/share/cv_bridge/cmake)  #在find_package前面
find_package(cv_bridge)
set(OpenCV_DIR "/home/hl/Downloads/opencv-3.4.15/build")
find_package(OpenCV 3)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(point_cloud_projection src/cam_lidar_project.cpp)
add_executable(test_direction src/test_direction.cpp)

target_link_libraries(point_cloud_projection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

target_link_libraries(test_direction
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${cv_bridge_LIBRARIES}
)

