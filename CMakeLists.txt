cmake_minimum_required(VERSION 2.8.3)
project(tsensors_mitsui)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation genmsg geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build follower
include_directories(include ${catkin_INCLUDE_DIRS})