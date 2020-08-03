cmake_minimum_required(VERSION 2.8.3)
project(human_arm)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY config meshes launch scripts urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS 
	scripts/key_control.py
	scripts/a_imu_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
