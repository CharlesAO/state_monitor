#include "state_monitor/state_monitor.h"

StateMonitor::StateMonitor(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private) {
  nh_private_.param("plot_time_length_secs", plot_time_length_secs_, 10.0);
  double plot_update_rate_hz;
  nh_private_.param("plot_update_rate_hz", plot_update_rate_hz, 20.0);

  draw_timer_ = nh_.createTimer(ros::Duration(1.0 / plot_update_rate_hz),
                                &StateMonitor::drawCallback, this);

  constexpr double kNodeSearchRateHz = 1.0;
  node_search_timer_ = nh_.createTimer(ros::Duration(1.0 / kNodeSearchRateHz),
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

  // Mavros
  match = topic_info.name.find("vfr_hud");
  if (match != std::string::npos) {
    const std::string topic_base = topic_info.name.substr(0, match);
    node_plotter_map_.emplace(std::make_pair(
        topic_base, std::make_shared<MavrosPlotter>(topic_base, nh_,
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

// MAV Control RW
#ifdef MAV_CONTROL_RW_FOUND
  match = topic_info.name.find("KF_observer/observer_state");
  if ((match != std::string::npos)) {
    const std::string topic_base = topic_info.name.substr(0, match);
    node_plotter_map_.emplace(std::make_pair(
        topic_base, std::make_shared<MAVControlRWPlotter>(
                        topic_base, nh_, x11_window_.getMGLGraph(),
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

  constexpr double kTextStartPos = 1.05;
  constexpr double kTextSize = 5.0;
  constexpr double kLineHeight = 0.05;

  constexpr size_t kSideBarIdx = 0;
  constexpr size_t kNumSubplotsWide = 4;
  constexpr size_t kNumSideBarsHigh = 1;

  gr->SubPlot(kNumSubplotsWide, kNumSideBarsHigh, kSideBarIdx, "");
  gr->SetRanges(0, 1, 0, 1);
  mreal text_location = kTextStartPos;
  gr->Puts(mglPoint(0, text_location), "\\b{State Monitor}", "w:L", kTextSize);
  text_location -= kLineHeight;
  gr->Puts(mglPoint(0, text_location), "Monitored Nodes:", "w:L", kTextSize);

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

    text_location -= kLineHeight;
    gr->Puts(mglPoint(0, text_location),
             ("  \\b{" + text_to_print + "}").c_str(), format.c_str(),
             kTextSize);
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
