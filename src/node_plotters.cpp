#include "state_monitor/node_plotters.h"

NodePlotter::NodePlotter(const std::string& topic_base)
    : topic_base_(topic_base) {}

void NodePlotter::plot() {
  for (std::shared_ptr<RosPlotterBase>& plotter : plotters_) {
    plotter->plot();
  }
}

SWFPlotter::SWFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
                       const std::shared_ptr<mglGraph>& gr,
                       const double keep_data_for_secs)
    : NodePlotter(topic_base) {
  constexpr size_t num_subplots_wide = 4;
  constexpr size_t num_subplots_high = 2;

  plotters_.push_back(std::make_shared<OdometryPlotter>(
      nh, topic_base + "swf/local_odometry", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 1, 2, 5, 6));

  plotters_.push_back(std::make_shared<PointPlotter>(
      nh, topic_base + "swf/linear_acceleration_biases", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, "Linear Acceleration Bias (m^2/s)",
      3));

  plotters_.push_back(std::make_shared<PointPlotter>(
      nh, topic_base + "swf/angular_velocity_biases", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, "Angular Velocity Bias (rad/s)",
      7));
}

#ifdef MSF_FOUND
MSFPlotter::MSFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
                       const std::shared_ptr<mglGraph>& gr,
                       const double keep_data_for_secs)
    : NodePlotter(topic_base) {
  constexpr size_t num_subplots_wide = 4;
  constexpr size_t num_subplots_high = 2;

  plotters_.push_back(std::make_shared<OdometryPlotter>(
      nh, topic_base + "/odometry", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 1, 2, 5, 6));

  plotters_.push_back(std::make_shared<MSFStatePlotter>(
      nh, topic_base + "/state_out", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 3, 7));
}
#endif
