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
  LIBRARIES
    adap
  CATKIN_DEPENDS
    roscpp
    message_runtime
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  adap
  src/libadap/libadap.cpp
)

add_dependencies(
  adap
  ${catkin_EXPORTED_TARGETS} # standard message/service files
  ${${PROJECT_NAME}_EXPORTED_TARGETS} # this packages message/service files
)

target_link_libraries(
  adap
  ${catkin_LIBRARIES}
)

add_executable(
  server
  src/server/main.cpp
  src/server/server.h
  src/server/server.cpp
  src/server/client.h
  src/server/client.cpp
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


add_executable(client_basic src/clients/basic.cpp)
add_dependencies(client_basic ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(client_basic adap ${catkin_LIBRARIES})

add_executable(client_v4l src/clients/v4l.cpp)
add_dependencies(client_v4l ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(client_v4l adap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(client_thresh src/clients/thresh.cpp)
add_dependencies(client_thresh ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(client_thresh adap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


if(Sanitizers_FOUND)
  add_sanitizers(adap)
  add_sanitizers(server)
  add_sanitizers(client_basic)
  add_sanitizers(client_v4l)
endif(Sanitizers_FOUND)
