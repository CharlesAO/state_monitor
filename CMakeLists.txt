cmake_minimum_required(VERSION 2.8.3)
project(state_monitor)

#needed for plotting (mathgl is also needed but doesn't have a cmake find)
find_package(X11 REQUIRED)

#the optional msf package means we can't use catkin simple
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

find_package(sensor_fusion_comm)
IF(${sensor_fusion_comm_FOUND})
  add_definitions(-DMSF_FOUND)

  include_directories(
    ${sensor_fusion_comm_INCLUDE_DIRS}
  )
ELSE()
  MESSAGE(WARNING "sensor_fusion_comm not found, building without MSF support")
ENDIF()

find_package(mav_disturbance_observer)
IF(${mav_disturbance_observer_FOUND})
  add_definitions(-DMAV_CONTROL_RW_FOUND)

  include_directories(
    ${mav_disturbance_observer_INCLUDE_DIRS}
  )
ELSE()
  MESSAGE(WARNING "mav_disturbance_observer not found, building without mav_control_rw support")
ENDIF()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}
)

add_definitions(-std=gnu++11 -o3 -fext-numeric-literals)

add_library(${PROJECT_NAME}
  src/x11_window.cpp
  src/ros_plotters.cpp
  src/node_plotters.cpp
  src/state_monitor.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(state_monitor_node
  src/state_monitor_node.cpp
)
target_link_libraries(state_monitor_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${X11_LIBRARIES} -lmgl-fltk)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

file(COPY state_monitor
  DESTINATION  ${CATKIN_DEVEL_PREFIX}/bin
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

