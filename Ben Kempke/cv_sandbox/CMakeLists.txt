cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

set(CMAKE_C_FLAGS "-Wall")
set(OpenCV_DIR "/usr/dssdr/opencv/build/")

add_executable(tracker tracker.cpp)
add_executable(tiltdemo tiltdemo.cpp)
add_executable(balltrack balltrack.cpp)
add_executable(capture capture.cpp)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Freenect REQUIRED)

include_directories(${Freenect_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${USB_INCLUDE_DIRS})

target_link_libraries(tracker ${Freenect_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${OpenCV_LIBS})
target_link_libraries(tiltdemo ${Freenect_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${OpenCV_LIBS})
target_link_libraries(balltrack ${Freenect_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${OpenCV_LIBS})
target_link_libraries(capture ${Freenect_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB} ${OpenCV_LIBS})

install (TARGETS tracker
  DESTINATION bin)
