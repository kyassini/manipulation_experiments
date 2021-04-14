cmake_minimum_required(VERSION 3.0.2)
project(gqcnn_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  moveit_commander
)

catkin_package()
