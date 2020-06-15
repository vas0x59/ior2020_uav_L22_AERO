cmake_minimum_required(VERSION 2.8.3)
project(l22_aero_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  # roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  ColorRectMarker.msg
  ColorRectMarkerArray.msg
)


# catkin_python_setup()


catkin_package(

CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB python_scripts_DIR
    "src/*.py"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
 
catkin_install_python(PROGRAMS ${python_scripts_DIR}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_package()