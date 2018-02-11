cmake_minimum_required(VERSION 2.8.3)
project(state_monitor)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#needed for plotting (mathgl is also needed but doesn't have a cmake find)
find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})

add_definitions(-std=gnu++11 -o3)

############
# BINARIES #
############
cs_add_executable(state_monitor_node
  src/state_monitor_node.cpp
)
target_link_libraries(state_monitor_node ${X11_LIBRARIES} -lmgl)

##########
# EXPORT #
##########
cs_install()
cs_export()
