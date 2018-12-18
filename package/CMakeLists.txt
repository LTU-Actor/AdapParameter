cmake_minimum_required(VERSION 2.8.3)
project(adap_parameter CXX)
add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/externals/sanitizers-cmake/cmake" ${CMAKE_MODULE_PATH})

find_package(Sanitizers)
find_package(OpenCV)

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
    message_generation
    cv_bridge
    image_transport
)

add_service_files(
  FILES
    Register.srv
    Tune.srv
    Feedback.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

# This in the function that exports the client library and required include
# files
catkin_package(
  INCLUDE_DIRS
    include/
  CATKIN_DEPENDS
    roscpp
    message_runtime
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  server
  src/server/main.cpp
  src/server/server.h
  src/server/server.cpp
  src/server/client.h
  src/server/client.cpp
  src/server/tuner_gradients.cpp
)

add_dependencies(
  server
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
)

macro(add_client name)
  add_executable(${name} ${ARGN})
  add_dependencies(${name} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES})
endmacro()

add_client(client_synth_single src/clients/synth_single.cpp)
add_client(client_synth_multiple src/clients/synth_multiple.cpp)
add_client(client_synth_poly src/clients/synth_poly.cpp)
add_client(client_synth_codependent src/clients/synth_codependent.cpp)

add_client(cam_pub src/util/cam_pub.cpp)
add_client(client_v4l src/clients/v4l.cpp)
add_client(client_thresh src/clients/thresh.cpp)
add_client(client_canny src/clients/canny.cpp)
target_link_libraries(cam_pub ${OpenCV_LIBRARIES})
target_link_libraries(client_v4l ${OpenCV_LIBRARIES})
target_link_libraries(client_thresh ${OpenCV_LIBRARIES})
target_link_libraries(client_canny ${OpenCV_LIBRARIES})

if(Sanitizers_FOUND)
  add_sanitizers(server)
  add_sanitizers(cam_pub)
  add_sanitizers(client_synth_single)
  add_sanitizers(client_synth_multile)
  add_sanitizers(client_synth_poly)
  add_sanitizers(client_synth_codependent)
  add_sanitizers(client_v4l)
  add_sanitizers(client_thresh)
  add_sanitizers(client_canny)
endif(Sanitizers_FOUND)
