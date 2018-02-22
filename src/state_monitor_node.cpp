#include <ros/ros.h>

#include "state_monitor/state_monitor.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "state_monitor_node");


  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  StateMonitor state_monitor(nh, nh_private);

  ros::spin();

  return 0;
}