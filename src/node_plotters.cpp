#include <std_srvs/Empty.h>
#ifdef MSF_FOUND
#include <sensor_fusion_comm/InitScale.h>
#endif

#include "state_monitor/node_plotters.h"

NodePlotter::NodePlotter(const std::string& topic_base,
                         const ros::NodeHandle& nh)
    : topic_base_(topic_base), nh_(nh) {}

void NodePlotter::plot() {
  for (std::shared_ptr<RosPlotterBase>& plotter : plotters_) {
    plotter->plot();
  }
}

SWFPlotter::SWFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
                       const std::shared_ptr<mglGraph>& gr,
                       const double keep_data_for_secs)
    : NodePlotter(topic_base, nh) {
  constexpr size_t num_subplots_wide = 4;
  constexpr size_t num_subplots_high = 2;

  plotters_.push_back(std::make_shared<OdometryPlotter>(
      nh_, topic_base + "swf/local_odometry", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 1, 2, 5, 6));

  plotters_.push_back(std::make_shared<PointPlotter>(
      nh_, topic_base + "swf/linear_acceleration_biases", gr,
      keep_data_for_secs, num_subplots_wide, num_subplots_high,
      "Linear Acceleration Bias (m^2/s)", 3));

  plotters_.push_back(std::make_shared<PointPlotter>(
      nh_, topic_base + "swf/angular_velocity_biases", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, "Angular Velocity Bias (rad/s)",
      7));
}

void SWFPlotter::reset() {
  ros::ServiceClient client =
      nh_.serviceClient<std_srvs::Empty>(topic_base_ + "reset");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO_STREAM("Reset SWF on " << topic_base_ + "reset");
  } else {
    ROS_ERROR_STREAM("Failded to reset SWF on " << topic_base_ + "reset");
  }
}

RovioPlotter::RovioPlotter(const std::string& topic_base,
                           const ros::NodeHandle& nh,
                           const std::shared_ptr<mglGraph>& gr,
                           const double keep_data_for_secs)
    : NodePlotter(topic_base, nh) {
  constexpr size_t num_subplots_wide = 4;
  constexpr size_t num_subplots_high = 2;

  plotters_.push_back(std::make_shared<OdometryPlotter>(
      nh_, topic_base + "odometry", gr, keep_data_for_secs, num_subplots_wide,
      num_subplots_high, 1, 2, 5, 6));

  plotters_.push_back(std::make_shared<ImuBiasPlotter>(
      nh_, topic_base + "imu_biases", gr, keep_data_for_secs, num_subplots_wide,
      num_subplots_high, 3, 7));
}

void RovioPlotter::reset() {
  ros::ServiceClient client =
      nh_.serviceClient<std_srvs::Empty>(topic_base_ + "reset");
  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO_STREAM("Reset Rovio on " << topic_base_ + "reset");
  } else {
    ROS_ERROR_STREAM("Failded to reset Rovio on " << topic_base_ + "reset");
  }
}

#ifdef MSF_FOUND
MSFPlotter::MSFPlotter(const std::string& topic_base, const ros::NodeHandle& nh,
                       const std::shared_ptr<mglGraph>& gr,
                       const double keep_data_for_secs)
    : NodePlotter(topic_base, nh) {
  constexpr size_t num_subplots_wide = 4;
  constexpr size_t num_subplots_high = 2;

  // deal with msf's 'interesting' topic naming conventions
  size_t last_char = topic_base.rfind('/', topic_base.size() - 2);
  std::string core_base = topic_base.substr(0, last_char);

  plotters_.push_back(std::make_shared<OdometryPlotter>(
      nh_, core_base + "/msf_core/odometry", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 1, 2, 5, 6));

  plotters_.push_back(std::make_shared<MSFStatePlotter>(
      nh_, core_base + "/msf_core/state_out", gr, keep_data_for_secs,
      num_subplots_wide, num_subplots_high, 3, 7));
}

void MSFPlotter::reset() {
  ros::ServiceClient client = nh_.serviceClient<sensor_fusion_comm::InitScale>(
      topic_base_ + "pose_sensor/initialize_msf_scale");
  sensor_fusion_comm::InitScale srv;
  srv.request.scale = 1.0f;

  if (client.call(srv)) {
    ROS_INFO_STREAM("MSF Response to Reset: " << srv.response.result);
  } else {
    ROS_ERROR_STREAM("Failded to reset MSF on "
                     << topic_base_ + "pose_sensor/initialize_msf_scale");
  }
}
#endif
