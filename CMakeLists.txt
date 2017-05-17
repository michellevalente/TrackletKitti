cmake_minimum_required(VERSION 2.8)
project(tracklet_kitti)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros cv_bridge image_transport message_generation sensor_msgs)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV 3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(tracklet_kitti src/tracklet_kitti.cpp)
add_dependencies(tracklet_kitti ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tracklet_kitti ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS tracklet_kitti 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
