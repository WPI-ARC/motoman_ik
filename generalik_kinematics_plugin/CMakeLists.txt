cmake_minimum_required(VERSION 2.8.3)
project(generalik_kinematics_plugin)

find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_perception
  cmake_modules
  pluginlib
  actionlib
  roscpp
  rosconsole
  dynamic_reconfigure
  message_filters
  srdfdom
  urdf
  tf
  tf_conversions
)
find_package(Eigen REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pluginlib
    moveit_core
    moveit_ros_perception
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
		    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


set(MOVEIT_LIB_NAME generalik_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/generalik_kinematics_plugin.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
    generalik_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
