cmake_minimum_required(VERSION 2.8.3)
project(connectivity_controller_old)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure cmake_modules geometry_msgs nav_msgs roscpp tf)
find_package(Eigen REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Connectivity_oldMsgs.msg
  Control_oldMsgs.msg
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/Control_oldVariables.cfg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#INCLUDE_DIRS include
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp tf
 DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


## Declare a cpp library
# add_library(connectivity_controller_old
#   src/${PROJECT_NAME}/connectivity_controller.cpp
# )

## Declare a cpp executable
add_executable(control_old src/connectivity_control_node.cpp)
add_executable(connectivity_old src/connectivity.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(connectivity_old connectivity_controller_old_generate_messages_cpp)
add_dependencies(control_old connectivity_controller_old_generate_messages_cpp)
add_dependencies(connectivity_old connectivity_controller_old_gencfg)
add_dependencies(control_old connectivity_controller_old_gencfg)

### Spe cify libraries to link a library or executable target against
target_link_libraries(connectivity_old
 ${catkin_LIBRARIES}
)

target_link_libraries(control_old
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS connectivity_controller connectivity_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_connectivity_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
