cmake_minimum_required(VERSION 2.8.3)
project(hasanmay)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure cmake_modules geometry_msgs nav_msgs roscpp tf create_driver)
find_package(Eigen REQUIRED)


catkin_package(
#INCLUDE_DIRS include
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp tf
 DEPENDS eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
# control = executable's name
add_executable(controlhp src/control_node.cpp)

target_link_libraries(controlhp
  ${catkin_LIBRARIES}
  create_vicon_stream
)
