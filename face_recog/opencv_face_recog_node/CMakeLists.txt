
cmake_minimum_required(VERSION 2.8.3)
project(opencv_face_recog_node)

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs roscpp cv_bridge image_geometry image_transport)
find_package( OpenCV REQUIRED )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
add_executable( face_detect_node src/face_detector_node.cpp )
target_link_libraries ( face_detect_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

# add_executable( face_detect_sample src/face_detector.cpp )
# add_executable( compressed_image_sub_test src/compressed_image_subscriber.cpp )
# target_link_libraries ( face_detect_sample ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
# target_link_libraries ( compressed_image_sub_test ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
