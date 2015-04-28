cmake_minimum_required(VERSION 2.8.3)
project(trajlib)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs 
  message_generation
  cmake_modules
  moveit_msgs
)

find_package(cmake_modules REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetTrajectory.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  moveit_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES right_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(include 
#  ${catkin_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
#  ${catkin_LIBRARY_DIRS}
#)

## Declare a cpp library
# add_library(right_arm
#   src/${PROJECT_NAME}/right_arm.cpp
# )

## Declare a cpp executable
# add_executable(right_arm_node src/right_arm_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(right_arm_node right_arm_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(right_arm_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS right_arm right_arm_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_right_arm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(gripper_srv src/gripper_controller.py)
#target_link_libraries(gripper_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS gripper_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})







