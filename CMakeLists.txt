cmake_minimum_required(VERSION 3.0.2)
project(bbox_position)

find_package(catkin REQUIRED COMPONENTS
  tf2
  roscpp
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs  
  message_generation
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
set (LIBS ${LIBS} ${PCL_LIBS})

add_message_files(
   FILES
   ImageBoundingBoxListStamped.msg
   ImageBoundingBox.msg
   ImagePoint.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimator src/main.cpp)

target_link_libraries(pose_estimator
  ${catkin_LIBRARIES} ${LIBS}
)