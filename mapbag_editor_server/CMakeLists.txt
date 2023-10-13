cmake_minimum_required(VERSION 3.5.0)
project(mapbag_editor_server VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  grid_map_msgs
  hector_math
  hector_std_msgs
  hector_world_heightmap
  mapbag_editor_msgs
  interactive_markers
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES mapbag_editor_server
 CATKIN_DEPENDS grid_map_msgs hector_math hector_std_msgs hector_world_heightmap mapbag_editor_msgs roscpp std_msgs interactive_markers
#  DEPENDS system_lib
)

set(LIBRARIES ${catkin_LIBRARIES})
set(INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable( mapbag_editor_server_node src/mapbag_editor_server_node.cpp)

target_link_libraries( mapbag_editor_server_node
  ${LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} mapbag_editor_server_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS mapbag_editor_server_node LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch maps config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

