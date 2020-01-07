project(yolo_FPGA)

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++14 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

include_directories( ${PROJECT_SOURCE_DIR}/include)
FILE(GLOB_RECURSE sourcefiles "./src/*.cpp" )

#armor detection node
add_executable(${PROJECT_NAME}_node
  ${sourcefiles}
)

TARGET_LINK_LIBRARIES(yolo_FPGA_node  /usr/lib/libdpyolov3.so)
#TARGET_LINK_LIBRARIES(yolo_FPGA_node  /usr/lib/libdpumodelyolo.so)
TARGET_LINK_LIBRARIES(yolo_FPGA_node  /usr/lib/libdpssd.so)
TARGET_LINK_LIBRARIES(yolo_FPGA_node  /usr/lib/libglog.so)
TARGET_LINK_LIBRARIES(yolo_FPGA_node  ${OpenMP_LIBS})

target_link_libraries(yolo_FPGA_node
  PRIVATE
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(yolo_FPGA_node
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
add_dependencies(yolo_FPGA_node
  roborts_msgs_generate_messages)


