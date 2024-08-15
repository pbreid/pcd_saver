cmake_minimum_required(VERSION 3.0.2)
project(pcd_saver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_saver src/pcd_saver.cpp)

target_link_libraries(pcd_saver
  ${catkin_LIBRARIES}
)

## Install the executable
install(TARGETS pcd_saver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)