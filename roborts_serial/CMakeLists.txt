project(roborts_serial)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions
  pcl_ros
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  interactive_markers
#  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(Boost REQUIRED)

catkin_package()

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SerialProtoSrc SerialProtoHds ${ProtoFiles})

add_executable(serial_com_node serial_com_node.cpp ${SerialProtoSrc} ${SerialProtoHds})
target_include_directories(serial_com_node PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${catkin_INCLUDE_DIRS}
  		${OpenCV_INCLUDE_DIRECTORIES}
  )
target_link_libraries(serial_com_node
        PRIVATE
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${GLOG_LIBRARY}
        )

add_dependencies(serial_com_node
	roborts_msgs_generate_messages)
