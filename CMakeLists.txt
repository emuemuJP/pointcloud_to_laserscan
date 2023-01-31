cmake_minimum_required(VERSION 3.0.2)
project(detect_area_distance)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan pointcloud
  CATKIN_DEPENDS laser_geometry message_filters nodelet roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(laserscan src/laserscan_nodelet.cpp)
target_link_libraries(laserscan ${catkin_LIBRARIES})

add_executable(laserscan_node src/laserscan_node.cpp)
target_link_libraries(laserscan_node laserscan ${catkin_LIBRARIES})

add_library(pointcloud src/pointcloud_nodelet.cpp)
target_link_libraries(pointcloud ${catkin_LIBRARIES})

add_executable(pointcloud_node src/pointcloud_node.cpp)
target_link_libraries(pointcloud_node pointcloud ${catkin_LIBRARIES})

install(TARGETS
  laserscan
  laserscan_node
  pointcloud
  pointcloud_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
