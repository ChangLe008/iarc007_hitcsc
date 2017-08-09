cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
  dji_sdk
  rosgraph_msgs
  message_generation
  iarc
  visualization_msgs
)
find_package(OpenCV REQUIRED)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
CATKIN_DEPENDS message_runtime
)

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#add_message_files(
#	obstacle.msg
#)

#generate_messages()
#	DEPENDENCIES
#	rplidarNode


## Declare a C++ executable
add_executable(test_fre
  src/test_fre.cpp
)
add_dependencies(test_fre ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_fre
  ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Declare a C++ executable
add_executable(time
  src/time.cpp
)
add_dependencies(time ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(time
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Declare a C++ executable
add_executable(fre
  src/fre.cpp
)
add_dependencies(fre ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fre
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


## Declare a C++ executable
add_executable(rp_pub
  src/rp_pub.cpp
)
add_dependencies(rp_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_pub
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(leo_pub
  src/leo_pub.cpp
  
)
add_dependencies(leo_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(leo_pub
  ${catkin_LIBRARIES}
    ${OpenCV_LIBS}

)