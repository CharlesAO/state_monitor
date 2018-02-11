#ifndef NODE_PLOTTERS_STATE_MONITOR_H
#define NODE_PLOTTERS_STATE_MONITOR_H

#include <ros/master.h>
#include <ros/ros.h>

#include "state_monitor/ros_plotters.h"
#include "state_monitor/x11_window.h"

class NodePlotter {
 public:
  NodePlotter(const std::string& topic_base) : topic_base_(topic_base) {}

  void plot() {
    for (std::shared_ptr<OdometryPlotter>& plotter : odometry_plotters_) {
      plotter->plot();
    }
    for (std::shared_ptr<PointPlotter>& plotter : point_plotters_) {
      plotter->plot();
    }
    for (std::shared_ptr<TransformPlotter>& plotter : transform_plotters_) {
      plotter->plot();
    }
    for (std::shared_ptr<PosePlotter>& plotter : pose_plotters_) {
      plotter->plot();
    }
    for (std::shared_ptr<ImuPlotter>& plotter : imu_plotters_) {
      plotter->plot();
    }
  }

 protected:
  const std::string topic_base_;

  // this group of every type is crazy ugly, but I couldn't find a clean
  // solution that wasn't insanely complex
  std::vector<std::shared_ptr<OdometryPlotter>> odometry_plotters_;
  std::vector<std::shared_ptr<PointPlotter>> point_plotters_;
  std::vector<std::shared_ptr<TransformPlotter>> transform_plotters_;
  std::vector<std::shared_ptr<PosePlotter>> pose_plotters_;
  std::vector<std::shared_ptr<ImuPlotter>> imu_plotters_;
};

class SWFPlotter : public NodePlotter {
 public:
  SWFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
             const std::shared_ptr<mglGraph>& gr,
             const double keep_data_for_secs)
      : NodePlotter(topic_base) {
    odometry_plotters_.push_back(std::make_shared<OdometryPlotter>(
        nh, gr, keep_data_for_secs, 4, 2, 1, 2, 5, 6));
    odometry_plotters_.back()->setSubscriberTopic(topic_base +
                                                  "swf/local_odometry");

    point_plotters_.push_back(
        std::make_shared<PointPlotter>(nh, gr, keep_data_for_secs, 4, 2,
                                       "Linear Acceleration Bias (m^2/s)", 3));
    point_plotters_.back()->setSubscriberTopic(
        topic_base + "swf/linear_acceleration_biases");

    point_plotters_.push_back(std::make_shared<PointPlotter>(
        nh, gr, keep_data_for_secs, 4, 2, "Angular Velocity Bias (rad/s)", 7));
    point_plotters_.back()->setSubscriberTopic(topic_base +
                                               "swf/angular_velocity_biases");
  }

  static bool isTopicMyKey(const ros::master::TopicInfo& topic_info,
                           std::string* topic_base) {
    size_t match = topic_info.name.find("swf/local_odometry");
    if (match != std::string::npos) {
      *topic_base = topic_info.name.substr(0, match);
      return true;
    } else {
      return false;
    }
  }
};

class PlotManager {
 public:
  PlotManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {
    nh_private_.param("plot_time_length_secs", plot_time_length_secs_, 10.0);

    draw_timer_ =
        nh_.createTimer(ros::Duration(0.1), &PlotManager::drawCallback, this);

    node_search_timer_ = nh_.createTimer(
        ros::Duration(1.0), &PlotManager::nodeSearchCallback, this);
  }

 private:
  void nodeSearchCallback(const ros::TimerEvent& event) {
    // try to be fancy and find topic names automatically
    ros::master::V_TopicInfo topic_info_vector;
    ros::master::getTopics(topic_info_vector);

    for (const ros::master::TopicInfo& topic_info : topic_info_vector) {
      std::string topic_base;
      if (SWFPlotter::isTopicMyKey(topic_info, &topic_base)) {
        if (node_plotter_map_.find(topic_base) == node_plotter_map_.end()) {
          ROS_INFO_STREAM("Attaching to Node output for " << topic_base);
          node_plotter_map_[topic_base] = std::make_shared<SWFPlotter>(
              topic_base, nh_, x11_window_.getMGLGraph(),
              plot_time_length_secs_);
        }
      }
    }
  }

  // cycle state estimator in focus
  void cycleFocus(const int key) {

    static int prev_key = 0;
    if(key == prev_key){
      prev_key = key;
      return;
    }
    prev_key = key;
    
    if (key == 111) {
      auto it = node_plotter_map_.find(node_in_focus_);
      ++it;
      if (it == node_plotter_map_.end()) {
        it = node_plotter_map_.begin();
      }
      node_in_focus_ = it->first;
    } else if (key == 116) {
      auto it = node_plotter_map_.find(node_in_focus_);
      if (it == node_plotter_map_.begin()) {
        it = node_plotter_map_.end();
      }
      --it;
      node_in_focus_ = it->first;
    }
  }

  void printSidebar() {
    std::shared_ptr<mglGraph> gr = x11_window_.getMGLGraph();
    gr->SubPlot(4, 1, 0, "");
    gr->SetRanges(0, 1, 0, 1);
    mreal text_location = 1.05;
    gr->Puts(mglPoint(0, text_location), "\\b{State Monitor}", "w:L", 5);
    text_location -= 0.05;
    gr->Puts(mglPoint(0, text_location), "Monitored Nodes:", "w:L", 5);

    for (auto it = node_plotter_map_.begin(); it != node_plotter_map_.end();
         ++it) {
      std::string format;
      if (node_plotter_map_.find(node_in_focus_) == it) {
        format = "y:L";
      } else {
        format = "w:L";
      }

      text_location -= 0.05;
      gr->Puts(mglPoint(0, text_location),
               ("  \\b{" + it->first + "}").c_str(), format.c_str(), 5);
    }
  }

  void drawCallback(const ros::TimerEvent& event) {
    x11_window_
        .resizeAndClear();  // note this command destroys any work done by
                            // calls to plot()

    if (node_in_focus_.empty()) {
      if (node_plotter_map_.empty()) {
        return;
      } else {
        node_in_focus_ = node_plotter_map_.begin()->first;
      }
    }

    const int key = x11_window_.getKeypress();

    cycleFocus(key);

    node_plotter_map_[node_in_focus_]->plot();

    printSidebar();

    x11_window_.render();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer draw_timer_;
  ros::Timer node_search_timer_;

  X11Window x11_window_;
  double plot_time_length_secs_;
  std::map<std::string, std::shared_ptr<NodePlotter>> node_plotter_map_;
  std::string node_in_focus_;
};

#endif  // NODE_PLOTTERS_STATE_MONITOR_H