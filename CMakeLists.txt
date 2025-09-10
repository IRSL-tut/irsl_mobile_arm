cmake_minimum_required(VERSION 3.0.2)

project(irsl_mobile_arm)

find_package(catkin REQUIRED)

catkin_package()

#install(DIRECTORY node
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#  )
install(DIRECTORY config gazebo_models launch map model world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
catkin_install_python(
  PROGRAMS node/dummy_odom_node.py node/odom_trajectory_to_twist_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
