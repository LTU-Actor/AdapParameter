cmake_minimum_required(VERSION 2.8.3)
project(adap_parameter)
add_definitions(-std=c++11)

find_package(catkin
  REQUIRED
  COMPONENTS
    roscpp
    message_generation
)

add_service_files(
  FILES
    Registration.srv
    Tune.srv
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
  src/main.cpp
)

add_dependencies(
  server
  ${catkin_EXPORTED_TARGETS} # standard message/service files
  ${${PROJECT_NAME}_EXPORTED_TARGETS} # this packages message/service files
)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
)
