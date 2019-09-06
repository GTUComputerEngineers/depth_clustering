find_package(Protobuf REQUIRED)
add_library(visualization SHARED visualizer.cpp)
target_link_libraries(visualization
                      identifiable
                      ZmqCommunication
                      CommonLib
                      ProtoFiles
                      ${PROTOBUF_LIBRARY}
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})
if(${PCL_FOUND})
  add_library(cloud_saver SHARED cloud_saver.cpp)
  target_link_libraries(cloud_saver ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()
