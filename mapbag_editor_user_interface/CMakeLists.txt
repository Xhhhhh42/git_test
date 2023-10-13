cmake_minimum_required(VERSION 3.5.0)
project(mapbag_editor_user_interface VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_msgs
  hector_math
  hector_rviz_overlay
  hector_world_heightmap
  roscpp
  rviz
  std_msgs
  visualization_msgs
)

find_package(Qt5 COMPONENTS Core REQUIRED)
set(CMAKE_AUTOMOC ON)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES mapbag_editor_user_interface
  CATKIN_DEPENDS geometry_msgs grid_map_msgs hector_math hector_rviz_overlay hector_world_heightmap roscpp rviz std_msgs visualization_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/mapbag_editor_user_interface/editor_user_interface.h
  include/mapbag_editor_user_interface/polygon_tool.h
  src/editor_user_interface.cpp
  src/polygon_tool.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY icons media ui DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
