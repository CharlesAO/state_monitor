cmake_minimum_required(VERSION 2.8.3)
project(state_monitor)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#needed for plotting (mathgl is also needed but doesn't have a cmake find)
find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})

add_definitions(-std=gnu++11 -o3)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/x11_window.cpp
  src/ros_plotters.cpp
  src/node_plotters.cpp
  src/state_monitor.cpp
)

############
# BINARIES #
############
cs_add_executable(state_monitor_node
  src/state_monitor_node.cpp
)
target_link_libraries(state_monitor_node ${PROJECT_NAME} ${X11_LIBRARIES} -lmgl)

##########
# EXPORT #
##########
cs_install()
cs_export()
