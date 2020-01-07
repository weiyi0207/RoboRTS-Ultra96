project(armor_detection)

add_subdirectory(constraint_set)


file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ArmorDetectionProtoSrc
  ArmorDetectionProtoHds
  ${ProtoFiles})

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

include_directories( ${PROJECT_SOURCE_DIR}/autoAiming/include)
FILE(GLOB_RECURSE sourcefiles "autoAiming/src/*.cpp" )

#armor detection node
add_executable(${PROJECT_NAME}_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  armor_detection_node.cpp
  gimbal_control.cpp
  ${sourcefiles}
)

TARGET_LINK_LIBRARIES(armor_detection_node  /lib/libMVSDK.so)
TARGET_LINK_LIBRARIES(armor_detection_node  ${OpenMP_LIBS})

target_link_libraries(armor_detection_node
  PRIVATE
  detection::tool
  detection::constraint_set
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(armor_detection_node
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
add_dependencies(armor_detection_node
  roborts_msgs_generate_messages)

#armor_detection_client
add_executable(${PROJECT_NAME}_client
  armor_detection_client.cpp
)

target_include_directories(armor_detection_client
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

target_link_libraries(armor_detection_client
  PRIVATE
  detection::tool
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(armor_detection_client
  roborts_msgs_generate_messages)
