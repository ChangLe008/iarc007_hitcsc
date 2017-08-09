cmake_minimum_required(VERSION 2.8.3)
project(iarc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  message_generation
  roscpp
  rospy
  std_msgs
  message_runtime
  sensor_msgs
  visualization_msgs
  dji_sdk
)

#catkin_python_setup()#added in 20170521

find_package(OpenCV 2.4.13 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

add_message_files(
   FILES
   location.msg
   path.msg
   target_full.msg
   target.msg
   obstacle.msg
   vehicle_pos.msg
   model_choose.msg
   object.msg
   location.msg
   position.msg
   sona.msg
)


generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs
   actionlib_msgs
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
   INCLUDE_DIRS include
   #LIBRARIES lib
   CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs dji_sdk message_runtime
   #DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/iarc
  #include/pilot
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(iarc
#   src/${PROJECT_NAME}/iarc.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(iarc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(iarc_node src/iarc_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(iarc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(iarc_node
#   ${catkin_LIBRARIES}
# )


add_executable(pilot
  #include/iarc/subscribe.h
  src/pilot/main.cpp src/pilot/leo_math.cpp src/pilot/parameter.cpp src/pilot/control.cpp src/pilot/subscribe.cpp
  src/pilot/pid_controller.cpp
  src/pilot/XU_ESO.cpp
  src/pilot/XU_LADRC.cpp
  src/pilot/target_processor.cpp
)

add_dependencies(pilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(pilot
  ${catkin_LIBRARIES}
)

add_executable(top src/pilot/top.cpp src/pilot/rrt.cpp src/pilot/subscribe.cpp src/pilot/leo_math.cpp)

add_dependencies(top ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(top
  ${catkin_LIBRARIES}
)

add_executable(simu src/pilot/simulation.cpp src/pilot/rrt.cpp src/pilot/subscribe.cpp src/pilot/leo_math.cpp)

add_dependencies(simu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(simu
  ${catkin_LIBRARIES}
)

add_executable(position src/navigation/position.cpp
                                           src/navigation/math.cpp
                                           src/navigation/find_boundary.cpp
                                           src/navigation/common.cpp
                                           src/navigation/line_classified.cpp
                                           src/navigation/line_inside.cpp
                                           src/navigation/threshold_image.cpp
                                           src/navigation/test.cpp)
add_dependencies(position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(position
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(wide_angle_pub  src/navigation/wide_angle_pub.cpp)
add_dependencies(wide_angle_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(wide_angle_pub
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
add_executable(external_parameter  src/navigation/external_parameter.cpp)
add_dependencies(external_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(external_parameter
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
add_executable(new_intrinsic  src/navigation/new_intrinsic.cpp)
add_dependencies(new_intrinsic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(new_intrinsic
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(sona   src/navigation/sona.cpp)
add_dependencies(sona ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(sona
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
add_executable(ultra   src/pilot/sona.cpp)
add_dependencies(ultra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(ultra
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(jrh_pilot
  #include/iarc/subscribe.h
  src/pilot/main.cpp src/pilot/leo_math.cpp src/pilot/parameter.cpp src/pilot/control_jrh.cpp src/pilot/subscribe.cpp
  src/pilot/pid_controller.cpp
  src/pilot/XU_ESO.cpp
  src/pilot/XU_LADRC.cpp
  src/pilot/target_processor.cpp
)

add_executable(cam_get
  src/navigation/cam_get.cpp
)
add_dependencies(cam_get ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(cam_get
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  yaml-cpp
)


add_dependencies(jrh_pilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} iarc_generate_messages_cpp)
target_link_libraries(jrh_pilot
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


###############added in 20170521################
install(PROGRAMS
  scripts/daemon
  scripts/tensorboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###############added in 20170521################


# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS iarc iarc_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_iarc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests


###############added in 20170521################
#catkin_add_nosetests(src/iarc/tests)
###############added in 20170521################


# catkin_add_nosetests(test)
