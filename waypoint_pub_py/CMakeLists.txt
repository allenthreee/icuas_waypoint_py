cmake_minimum_required(VERSION 2.8.3)
project(waypoint_pub)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############
catkin_install_python(PROGRAMS scripts/hello_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/waypoint_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
