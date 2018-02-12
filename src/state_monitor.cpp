#include "state_monitor/state_monitor.h"

StateMonitor::StateMonitor(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private) {
  nh_private_.param("plot_time_length_secs", plot_time_length_secs_, 10.0);

  draw_timer_ =
      nh_.createTimer(ros::Duration(0.05), &StateMonitor::drawCallback, this);

  node_search_timer_ = nh_.createTimer(ros::Duration(1.0),
                                       &StateMonitor::nodeSearchCallback, this);
}

void StateMonitor::createNodePlotterFromTopicInfo(
    const ros::master::TopicInfo& topic_info) {
  size_t match;

  // SWF
  match = topic_info.name.find("swf/local_odometry");
  if (match != std::string::npos) {
    const std::string topic_base = topic_info.name.substr(0, match);
    node_plotter_map_.emplace(std::make_pair(
        topic_base,
        std::make_shared<SWFPlotter>(topic_base, nh_, x11_window_.getMGLGraph(),
                                     plot_time_length_secs_)));
    return;
  }

  // Rovio
  match = topic_info.name.find("extrinsics0");
  if (match != std::string::npos) {
    const std::string topic_base = topic_info.name.substr(0, match);
    node_plotter_map_.emplace(std::make_pair(
        topic_base, std::make_shared<RovioPlotter>(topic_base, nh_,
                                                   x11_window_.getMGLGraph(),
                                                   plot_time_length_secs_)));
    return;
  }

// MSF
#ifdef MSF_FOUND
  match = topic_info.name.find("pose_sensor/parameter_updates");
  if ((match != std::string::npos)) {
    const std::string topic_base = topic_info.name.substr(0, match);
    node_plotter_map_.emplace(std::make_pair(
        topic_base,
        std::make_shared<MSFPlotter>(topic_base, nh_, x11_window_.getMGLGraph(),
                                     plot_time_length_secs_)));
    return;
  }
#endif
}

void StateMonitor::nodeSearchCallback(const ros::TimerEvent& event) {
  // try to be fancy and find topic names automatically
  ros::master::V_TopicInfo topic_info_vector;
  ros::master::getTopics(topic_info_vector);

  for (const ros::master::TopicInfo& topic_info : topic_info_vector) {
    createNodePlotterFromTopicInfo(topic_info);
  }
}

void StateMonitor::processKeyPress(const int key) {
  static int prev_key = 0;
  if (key == prev_key) {
    prev_key = key;
    return;
  }
  prev_key = key;

  // ROS_ERROR_STREAM("key " << key);

  constexpr int kUp = 111;
  constexpr int kDown = 116;
  constexpr int kR = 32;

  // cycle state estimator that is in focus
  if (key == kDown) {
    auto it = node_plotter_map_.find(node_in_focus_);
    ++it;
    if (it == node_plotter_map_.end()) {
      it = node_plotter_map_.begin();
    }
    node_in_focus_ = it->first;
  } else if (key == kUp) {
    auto it = node_plotter_map_.find(node_in_focus_);
    if (it == node_plotter_map_.begin()) {
      it = node_plotter_map_.end();
    }
    --it;
    node_in_focus_ = it->first;
  }
  // reset estimator
  else if (key == kR) {
    auto it = node_plotter_map_.find(node_in_focus_);
    it->second->reset();
  }
}

void StateMonitor::replaceSubString(const std::string& subject,
                                    const std::string& search,
                                    const std::string& replace,
                                    std::string* result) {
  *result = subject;
  size_t pos = 0;
  while ((pos = result->find(search, pos)) != std::string::npos) {
    result->replace(pos, search.length(), replace);
    pos += replace.length();
  }
}

void StateMonitor::printSidebar() {
  std::shared_ptr<mglGraph> gr = x11_window_.getMGLGraph();
  gr->SubPlot(4, 1, 0, "");
  gr->SetRanges(0, 1, 0, 1);
  mreal text_location = 1.05;
  gr->Puts(mglPoint(0, text_location), "\\b{State Monitor}", "w:L", 5);
  text_location -= 0.05;
  gr->Puts(mglPoint(0, text_location), "Monitored Nodes:", "w:L", 5);

  for (auto it = node_plotter_map_.begin(); it != node_plotter_map_.end();
       ++it) {
    // replace special characters (currently just _ )
    std::string text_to_print;
    replaceSubString(it->first, "_", "\\_", &text_to_print);

    std::string format;
    if (node_plotter_map_.find(node_in_focus_) == it) {
      format = "y:L";
    } else {
      format = "w:L";
    }

    text_location -= 0.05;
    gr->Puts(mglPoint(0, text_location),
             ("  \\b{" + text_to_print + "}").c_str(), format.c_str(), 5);
  }
}

void StateMonitor::drawCallback(const ros::TimerEvent& event) {
  x11_window_.resizeAndClear();  // note this command destroys any work done by
                                 // calls to plot()

  if (node_in_focus_.empty()) {
    if (node_plotter_map_.empty()) {
      return;
    } else {
      node_in_focus_ = node_plotter_map_.begin()->first;
    }
  }

  const int key = x11_window_.getKeypress();

  processKeyPress(key);

  node_plotter_map_[node_in_focus_]->plot();

  printSidebar();

  x11_window_.render();
}
