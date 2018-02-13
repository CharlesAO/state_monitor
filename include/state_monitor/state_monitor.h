#ifndef STATE_MONITOR_STATE_MONITOR_H
#define STATE_MONITOR_STATE_MONITOR_H

#include "state_monitor/node_plotters.h"

class StateMonitor {
 public:
  StateMonitor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  void createNodePlotterFromTopicInfo(const ros::master::TopicInfo& topic_info);

  void nodeSearchCallback(const ros::TimerEvent& event);

  void processKeyPress(const KeySym key);

  static void replaceSubString(const std::string& subject,
                               const std::string& search,
                               const std::string& replace, std::string* result);

  void printSidebar();

  void drawCallback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer draw_timer_;
  ros::Timer node_search_timer_;

  X11Window x11_window_;
  double plot_time_length_secs_;
  std::map<std::string, std::shared_ptr<NodePlotter>> node_plotter_map_;
  std::string node_in_focus_;
};

#endif  // STATE_MONITOR_STATE_MONITOR_H
