cmake_minimum_required(VERSION 3.0.2)
project(robot_env_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  moveit_msgs
  visualization_msgs
  rosgraph_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/TimeControl.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_env_publisher
#  CATKIN_DEPENDS moveit_msgs roscpp rospy std_msgs yaml-cpp
#  DEPENDS system_lib
)

############
## Python ##
############
## Add Python executables
catkin_install_python(PROGRAMS 
  scripts/publish_clock.py
  scripts/publish_clock_slidebar.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_env_publisher
  src/obstacle_interface.cpp
)
target_link_libraries(robot_env_publisher PUBLIC
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pinocchio::pinocchio
  yaml-cpp
)
target_compile_features(robot_env_publisher PUBLIC 
    cxx_std_17
)

## Declare C++ executables
add_executable(obstacle_publisher src/obstacle_publisher.cpp)
target_link_libraries(obstacle_publisher PRIVATE
  robot_env_publisher
)

add_executable(file_env_node src/file_env_node.cpp)
target_link_libraries(file_env_node PRIVATE
  robot_env_publisher
)
target_compile_features(file_env_node PUBLIC 
  cxx_std_17
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_env_publisher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
