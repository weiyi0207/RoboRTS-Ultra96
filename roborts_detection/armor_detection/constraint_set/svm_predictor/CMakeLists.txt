project(svm_predictor)

add_compile_options(-std=c++11)

# find_package(OpenCV 3 REQUIRED)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SVMPredictorProtoSrc SVMPredictorProtoHds ${ProtoFiles})

add_library(svm_predictor SHARED 
	${SVMPredictorProtoSrc}
	${SVMPredictorProtoHds}
	svm_predictor.cpp
)

add_library(detection::svm_predictor ALIAS svm_predictor)

target_link_libraries(svm_predictor
    PUBLIC
    detection::tool
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
