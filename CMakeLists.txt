cmake_minimum_required(VERSION 2.8.3)
project(kf_tracker_2)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED 
  COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    costmap_converter
    visualization_msgs
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
# )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS pcl_ros roscpp sensor_msgs costmap_converter visualization_msgs
  # DEPENDS system_lib eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include )

add_library(${PROJECT_NAME} 
  src/${PROJECT_NAME}.cpp 
  src/ihgp/Matern32model.cpp 
  src/ihgp/InfiniteHorizonGP.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME} 
    ${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_node 
		${PROJECT_NAME}
		${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

