cmake_minimum_required(VERSION 2.8.3)
project(tsensors_mitsui)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation genmsg geometry_msgs)

## Declare ROS messages and services
#add_message_files(FILES msg_.msg)
#add_service_files(FILES srv_.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build follower
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(cpp_name src/cpp_name.cpp)
#target_link_libraries(cpp_name ${catkin_LIBRARIES})
#add_dependencies(cpp_name workspace_generate_messages_cpp)
#add_dependencies(cpp_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
