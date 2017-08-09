cmake_minimum_required(VERSION 2.8.3)
project(iarc007)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  dji_sdk
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)

#find_package(OpenCV REQUIRED)
#set(CMAKE_PREFIX_PATH "/home/hitcsc/thirdparty/opencv2.4.9/")
#set(CMAKE_PREFIX_PATH "/home/hitcsc/thirdparty/opencv2.4.13/")

#find_package(OpenCV 2.4.9 EXACT REQUIRED)
find_package(OpenCV 2.4.13 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_message_files(
   FILES
   object.msg
   obstacle.msg
   vehicle_pos.msg
   jrh_model_choose.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs
   actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/iarc007.cfg
)

catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES lib
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs dji_sdk
   #DEPENDS 
)

include_directories(
  include
  include/iarc007
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# --- Add executable  ----#
add_executable(jrh 
	src/JRH_pilot/pilotmain.cpp
	src/JRH_pilot/control.cpp
	src/JRH_pilot/JRH_math.cpp
	)
 
add_executable(hy_opencv_pub src/HY_target/hy_opencv_pub.cpp)

add_executable(hy_opencv_sub src/HY_target/hy_opencv_sub.cpp 
			     src/HY_target/target.cpp 
			     src/HY_target/Track.cpp 
			     src/HY_target/common.cpp 
			     src/HY_target/aprilTags.cpp
			     src/HY_target/DecHOG.cpp
			     src/HY_target/findcircle.cpp
			     src/HY_target/fun.cpp
			     src/HY_target/Filter_Statistic.cpp
			     src/HY_target/lowtarget.cpp
			     src/HY_target/angleID.cpp
			     src/HY_target/hightarget.cpp
			     src/JRH_pilot/JRH_math.cpp)

add_executable(exter_param src/HY_target/exter_param.cpp)
add_executable(calibration src/HY_target/calibration.cpp)
add_executable(extrinsic src/HY_target/Extrinsic.cpp)
#add_executable(lk_track src/HY_target/lk_track.cpp)
# add_executable(calibration_fisheye src/HY_target/calibfisheye.cpp)

#add_executable(mjh_pos src/MJH_pos/mjh_pos.cpp)
#add_executable(mjh_vision_pos src/MJH_pos/mjh_vision_pos.cpp
#				                      src/MJH_pos/estimater.cpp
#			                        src/MJH_pos/mjh_math.cpp
#                              src/MJH_pos/find_line.cpp
#						                  src/MJH_pos/common.cpp)

#add_executable(mjh_vision_pos src/MJH_pos/mjh_vision_pos.cpp
#						      src/MJH_pos/threshold_image.cpp
#						      src/MJH_pos/mjh_math.cpp
#						      src/MJH_pos/find_line.cpp
#						      src/MJH_pos/common.cpp)
#add_executable(mjh_vision_yaw src/MJH_pos/mjh_vision_yaw.cpp
# 					src/MJH_pos/estimater.cpp
#			      src/JRH_pilot/JRH_math.cpp)

#add_executable(zy_obstacle src/ZY_obstacle/zy_obstacle.cpp)
#add_executable(localization src/localization/robot_local.cpp src/localization/localization.cpp 
#		src/localization/iposInfo.cpp)

# ----  dependencies   ----- #
add_dependencies(jrh iarc007_generate_messages_cpp dji_sdk_generate_messages_cpp)
add_dependencies(jrh ${PROJECT_NAME}_gencfg)

add_dependencies(hy_opencv_pub iarc007_generate_messages_cpp)
add_dependencies(hy_opencv_sub iarc007_generate_messages_cpp dji_sdk_generate_messages_cpp)
add_dependencies(exter_param iarc007_generate_messages_cpp)
add_dependencies(calibration iarc007_generate_messages_cpp)
#add_dependencies(network_client iarc007_generate_messages_cpp)
add_dependencies(extrinsic iarc007_generate_messages_cpp)
#add_dependencies(lk_track iarc007_generate_messages_cpp)

# add_dependencies(calibration_fisheye iarc007_generate_messages_cpp)

#add_dependencies(mjh_pos iarc007_generate_messages_cpp)
#add_dependencies(mjh_vision_pos iarc007_generate_messages_cpp)

#add_dependencies(mjh_vision_yaw iarc007_generate_messages_cpp)
#add_dependencies(zy_obstacle iarc007_generate_messages_cpp)



# ----  target_link  ----#
target_link_libraries(jrh
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(hy_opencv_pub
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
#target_link_libraries(optical_flow_detection
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(hy_opencv_sub
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
#target_link_libraries(optical_flow
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)
target_link_libraries(exter_param
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
target_link_libraries(calibration
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
# target_link_libraries(calibration_fisheye
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )
target_link_libraries(extrinsic
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
#target_link_libraries(lk_track
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(mjh_pos
#  ${OpenCV_LIBS}	
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(mjh_vision_pos
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(mjh_vision_yaw
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(zy_obstacle
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(localization
 # ${catkin_LIBRARIES}
  #${OpenCV_LIBS}  
#)
