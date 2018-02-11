#include <ros/ros.h>

#include "state_monitor/node_plotters.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "state_plot_node");


  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PlotManager plot_manager(nh, nh_private);

  ros::spin();

  return 0;
}