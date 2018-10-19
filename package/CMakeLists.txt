cmake_minimum_required(VERSION 2.8.3)
project(adap_parameter CXX)
add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/externals/sanitizers-cmake/cmake" ${CMAKE_MODULE_PATH})

find_package(Sanitizers)

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
    message_generation
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

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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
  ${catkin_EXPORTED_TARGETS} # standard message/service files
  ${${PROJECT_NAME}_EXPORTED_TARGETS} # this packages message/service files
)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
  -fsanitize=address
)

add_sanitizers(server)

add_executable(
  client
  src/client/main.cpp
)

add_dependencies(
  client
  ${catkin_EXPORTED_TARGETS} # standard message/service files
  ${${PROJECT_NAME}_EXPORTED_TARGETS} # this packages message/service files
)

target_link_libraries(
  client
  ${catkin_LIBRARIES}
)

add_sanitizers(client)
