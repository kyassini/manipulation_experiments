cmake_minimum_required(VERSION 3.0.2)
project(gqcnn_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  moveit_commander
  perception
  autolab_core
  gqcnn
  visualization
)


catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)