cmake_minimum_required(VERSION 2.8.3)
project(re_location)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy

  std_msgs
  sensor_msgs
  geometry_msgs

)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)


#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS

)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


add_executable(re_location src/re_location.cpp src/get_feature.cpp)
target_link_libraries(re_location
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES}

                      )
