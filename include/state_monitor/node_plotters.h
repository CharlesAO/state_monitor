#ifndef NODE_PLOTTERS_STATE_MONITOR_H
#define NODE_PLOTTERS_STATE_MONITOR_H

#include <ros/master.h>
#include <ros/ros.h>

#include "state_monitor/ros_plotters.h"
#include "state_monitor/x11_window.h"

class NodePlotter {
 public:
  NodePlotter(const std::string& topic_base, const ros::NodeHandle& nh);

  void plot();

  virtual void reset() = 0;

 protected:
  const std::string topic_base_;
  ros::NodeHandle nh_;

  std::vector<std::shared_ptr<RosPlotterBase>> plotters_;
};

class SWFPlotter : public NodePlotter {
 public:
  SWFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
             const std::shared_ptr<mglGraph>& gr,
             const double keep_data_for_secs);

  void reset();
};

class RovioPlotter : public NodePlotter {
 public:
  RovioPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
               const std::shared_ptr<mglGraph>& gr,
               const double keep_data_for_secs);

  void reset();
};

class MavrosPlotter : public NodePlotter {
 public:
  MavrosPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
               const std::shared_ptr<mglGraph>& gr,
               const double keep_data_for_secs);

  void reset(){}
};

#ifdef MSF_FOUND
class MSFPlotter : public NodePlotter {
 public:
  MSFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
             const std::shared_ptr<mglGraph>& gr,
             const double keep_data_for_secs);

  void reset();
};
#endif

#endif  // NODE_PLOTTERS_STATE_MONITOR_H