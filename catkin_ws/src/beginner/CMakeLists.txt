cmake_minimum_required(VERSION 2.8.3)
project(beginner)

# find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# declare ros messages and services
add_message_files(
  FILES
  message1.msg
)
add_service_files(
  FILES
  service1.srv
)

# generate added messages and service
generate_messages(
  DEPENDENCIES
  std_msgs
)

# declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# build talker
add_executable(
  talker 
  src/talker.cpp
)
target_link_libraries(
  talker 
  ${catkin_LIBRARIES}
)
add_dependencies(
  talker 
  beginner_tutorials_generate_messages_cpp
)
#build listener
add_executable(
  listener 
  src/listener.cpp
)
target_link_libraries(
  listener 
  ${catkin_LIBRARIES}
)
add_dependencies(
  listener 
  beginner_tutorials_generate_messages_cpp
)