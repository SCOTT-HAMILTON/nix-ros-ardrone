cmake_minimum_required(VERSION 3.0.2)
project(nix_ros_ardrone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ardrone_autonomy
  teleop_twist_joy 
  foxglove_bridge
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ardrone_setup
  # CATKIN_DEPENDS roscpp rospy std_msgs ardrone_autonomy hotasx_teleop joy foxglove_bridge
  # DEPENDS system_lib
)

# Si vous avez des exécutables Python ou C++ dans ce package, vous les ajouteriez ici.
# Par exemple, pour un script Python :
# install(PROGRAMS scripts/mon_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(PROGRAMS scripts/ardrone_joystick_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/pid_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/double_pid_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/pos_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/launch_chromium_foxglove.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.launch"
  PATTERN ".git" EXCLUDE
)
