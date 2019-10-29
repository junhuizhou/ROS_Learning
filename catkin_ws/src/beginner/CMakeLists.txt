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
  talker1 
  src/talker1.cpp
)
target_link_libraries(
  talker1 
  ${catkin_LIBRARIES}
)
add_dependencies(
  talker1 
  beginner_tutorials_generate_messages_cpp
)
# build listener
add_executable(
  listener1 
  src/listener1.cpp
)
target_link_libraries(
  listener1 
  ${catkin_LIBRARIES}
)
add_dependencies(
  listener1 
  beginner_tutorials_generate_messages_cpp
)
# build server
add_executable(
  server1
  src/server1
)
target_link_libraries(
  server1
  ${catkin_LIBRARIES}
)
add_dependencies(
  server1
  beginner_tutorials_generate_messages_cpp
)
# build client
add_executable(
  client1
  src/client1
)
target_link_libraries(
  client1
  ${catkin_LIBRARIES}
)
add_dependencies(
  client1
  beginner_tutorials_generate_messages_cpp
)