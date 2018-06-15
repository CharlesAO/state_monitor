#ifndef ROS_PLOTTERS_STATE_MONITOR_H
#define ROS_PLOTTERS_STATE_MONITOR_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#ifdef MSF_FOUND
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#endif

#ifdef MAV_CONTROL_RW_FOUND
#include <mav_disturbance_observer/ObserverState.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#endif

#ifdef MAVROS_FOUND
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#endif

#include "state_monitor/sub_plotter.h"

// needed so we can use a pointer to a plotter of unknown type to kick off a
// render
class RosPlotterBase {
 public:
  virtual void plot() = 0;
};

template <typename T, size_t num_subplots_, size_t plots_dim_>
class RosPlotter : public RosPlotterBase {
 public:
  RosPlotter(const ros::NodeHandle &nh, const std::string &topic,
             const std::shared_ptr<mglGraph> &gr,
             const double keep_data_for_secs, const size_t num_subplots_wide,
             const size_t num_subplots_high,
             const std::vector<std::string> subplot_titles,
             const std::vector<size_t> subplot_indicies)
      : nh_(nh) {
    for (size_t i = 0; i < num_subplots_; ++i) {
      sub_plots_.emplace_back(gr, keep_data_for_secs, subplot_titles[i],
                              num_subplots_wide, num_subplots_high,
                              subplot_indicies[i]);
    }

    sub_ = nh_.subscribe(topic, kQueueSize, &RosPlotter::callback, this);
  }

  // Copying crashes things (I think its to do with sharing memory with the
  // x11_window but I need to find out why)
  RosPlotter(const RosPlotter &other) = delete;
  RosPlotter &operator=(const RosPlotter &) = delete;

  void plot() {
    for (SubPlotter<plots_dim_> &sub_plot : sub_plots_) {
      sub_plot.plot();
    }
  }

 protected:
  virtual void callback(const T &msg) = 0;

  static constexpr size_t kQueueSize = 10;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::vector<SubPlotter<plots_dim_>> sub_plots_;
};

class OdometryPlotter : public RosPlotter<nav_msgs::OdometryConstPtr, 4, 3> {
 public:
  OdometryPlotter(const ros::NodeHandle &nh, const std::string &topic,
                  const std::shared_ptr<mglGraph> &gr,
                  const double keep_data_for_secs,
                  const size_t num_subplots_wide,
                  const size_t num_subplots_high,
                  const size_t position_subplot_idx,
                  const size_t linear_velocity_subplot_idx,
                  const size_t orientation_subplot_idx,
                  const size_t angular_velocity_subplot_idx);

 private:
  void callback(const nav_msgs::OdometryConstPtr &msg);

  enum PlotOrder { POSITION, LINEAR_VELOCITY, ORIENTATION, ANGULAR_VELOCITY };
};

class PointPlotter
    : public RosPlotter<geometry_msgs::PointStampedConstPtr, 1, 3> {
 public:
  PointPlotter(const ros::NodeHandle &nh, const std::string &topic,
               const std::shared_ptr<mglGraph> &gr,
               const double keep_data_for_secs, const size_t num_subplots_wide,
               const size_t num_subplots_high, const std::string title,
               const size_t subplot_idx);

 private:
  void callback(const geometry_msgs::PointStampedConstPtr &msg);
};

class TransformPlotter
    : public RosPlotter<geometry_msgs::TransformStampedConstPtr, 2, 3> {
 public:
  TransformPlotter(const ros::NodeHandle &nh, const std::string &topic,
                   const std::shared_ptr<mglGraph> &gr,
                   const double keep_data_for_secs,
                   const size_t num_subplots_wide,
                   const size_t num_subplots_high,
                   const size_t position_subplot_idx,
                   const size_t orientation_subplot_idx);

 private:
  void callback(const geometry_msgs::TransformStampedConstPtr &msg);

  enum PlotOrder { POSITION, ORIENTATION };
};

class PosePlotter
    : public RosPlotter<geometry_msgs::PoseStampedConstPtr, 2, 3> {
 public:
  PosePlotter(const ros::NodeHandle &nh, const std::string &topic,
              const std::shared_ptr<mglGraph> &gr,
              const double keep_data_for_secs, const size_t num_subplots_wide,
              const size_t num_subplots_high, const size_t position_subplot_idx,
              const size_t orientation_subplot_idx);

 private:
  void callback(const geometry_msgs::PoseStampedConstPtr &msg);

  enum PlotOrder { POSITION, ORIENTATION };
};

class ImuPlotter : public RosPlotter<sensor_msgs::ImuConstPtr, 2, 3> {
 public:
  ImuPlotter(const ros::NodeHandle &nh, const std::string &topic,
             const std::shared_ptr<mglGraph> &gr,
             const double keep_data_for_secs, const size_t num_subplots_wide,
             const size_t num_subplots_high,
             const size_t linear_acceleration_subplot_idx,
             const size_t angular_velocity_subplot_idx);

 private:
  void callback(const sensor_msgs::ImuConstPtr &msg);

  enum PlotOrder { LINEAR_ACCELERATION, ANGULAR_VELOCITY };
};

class ImuBiasPlotter : public RosPlotter<sensor_msgs::ImuConstPtr, 2, 3> {
 public:
  ImuBiasPlotter(const ros::NodeHandle &nh, const std::string &topic,
                 const std::shared_ptr<mglGraph> &gr,
                 const double keep_data_for_secs,
                 const size_t num_subplots_wide, const size_t num_subplots_high,
                 const size_t linear_acceleration_bias_subplot_idx,
                 const size_t angular_velocity_bias_subplot_idx);

 private:
  void callback(const sensor_msgs::ImuConstPtr &msg);

  enum PlotOrder { LINEAR_ACCELERATION_BIAS, ANGULAR_VELOCITY_BIAS };
};

class JoyPlotter : public RosPlotter<sensor_msgs::JoyConstPtr, 1, 6> {
 public:
  JoyPlotter(const ros::NodeHandle &nh, const std::string &topic,
             const std::shared_ptr<mglGraph> &gr,
             const double keep_data_for_secs, const size_t num_subplots_wide,
             const size_t num_subplots_high, const size_t joy_subplot_idx);

 private:
  void callback(const sensor_msgs::JoyConstPtr &msg);
};

class TrajectoryPlotter
    : public RosPlotter<trajectory_msgs::MultiDOFJointTrajectoryConstPtr, 2,
                        3> {
 public:
  TrajectoryPlotter(const ros::NodeHandle &nh, const std::string &topic,
                    const std::shared_ptr<mglGraph> &gr,
                    const double keep_data_for_secs,
                    const size_t num_subplots_wide,
                    const size_t num_subplots_high,
                    const size_t position_subplot_idx,
                    const size_t orientation_subplot_idx);

 private:
  void callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);

  enum PlotOrder { POSITION, ORIENTATION };
};

#ifdef MSF_FOUND
class MSFStatePlotter
    : public RosPlotter<sensor_fusion_comm::DoubleArrayStampedConstPtr, 2, 3> {
 public:
  MSFStatePlotter(const ros::NodeHandle &nh, const std::string &topic,
                  const std::shared_ptr<mglGraph> &gr,
                  const double keep_data_for_secs,
                  const size_t num_subplots_wide,
                  const size_t num_subplots_high,
                  const size_t linear_acceleration_bias_subplot_idx,
                  const size_t angular_velocity_bias_subplot_idx);

 private:
  void callback(const sensor_fusion_comm::DoubleArrayStampedConstPtr &msg);

  enum PlotOrder { LINEAR_ACCELERATION_BIAS, ANGULAR_VELOCITY_BIAS };
};
#endif

#ifdef MAV_CONTROL_RW_FOUND

// a weird special cose that subscribes to two topics, so they can be displayed
// on one graph
class ObserverStatePlotter
    : public RosPlotter<mav_disturbance_observer::ObserverStateConstPtr, 4, 6> {
 public:
  ObserverStatePlotter(
      const ros::NodeHandle &nh, const std::string &observer_topic,
      const std::string &ref_topic, const std::shared_ptr<mglGraph> &gr,
      const double keep_data_for_secs, const size_t num_subplots_wide,
      const size_t num_subplots_high, const size_t position_subplot_idx,
      const size_t orientation_subplot_idx,
      const size_t external_forces_subplot_idx,
      const size_t external_moments_subplot_idx);

 private:
  void callback(const mav_disturbance_observer::ObserverStateConstPtr &msg);

  void refCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);

  enum PlotOrder { POSITION, ORIENTATION, EXTERNAL_FORCES, EXTERNAL_MOMENTS };

  ros::Subscriber ref_sub_;
  geometry_msgs::Transform last_ref_;
};

class RPYRateThrustPlotter
    : public RosPlotter<mav_msgs::RollPitchYawrateThrustConstPtr, 2, 3> {
 public:
  RPYRateThrustPlotter(const ros::NodeHandle &nh, const std::string &topic,
                       const std::shared_ptr<mglGraph> &gr,
                       const double keep_data_for_secs,
                       const size_t num_subplots_wide,
                       const size_t num_subplots_high,
                       const size_t rpy_rate_subplot_idx,
                       const size_t thrust_idx);

 private:
  void callback(const mav_msgs::RollPitchYawrateThrustConstPtr &msg);

  enum PlotOrder { RPY_RATE, THRUST };
};
#endif

#ifdef MAVROS_FOUND

// a weird special cose that subscribes to three topics
class MAVROSPosePlotter
    : public RosPlotter<geometry_msgs::PoseStampedConstPtr, 2, 6> {
 public:
  MAVROSPosePlotter(const ros::NodeHandle &nh, const std::string &pose_topic,
                       const std::string &attitude_setpoint_topic,
                       const std::string &position_setpoint_topic,
                       const std::shared_ptr<mglGraph> &gr,
                       const double keep_data_for_secs,
                       const size_t num_subplots_wide,
                       const size_t num_subplots_high,
                       const size_t position_subplot_idx,
                       const size_t orientation_subplot_idx);

 private:
  void callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void attitudeSetpointCallback(const mavros_msgs::AttitudeTargetConstPtr &msg);

  void positionSetpointCallback(const mavros_msgs::PositionTargetConstPtr &msg);

  enum PlotOrder {
    POSITION,
    ORIENTATION,
  };

  ros::Subscriber attitude_setpoint_sub_;
  ros::Subscriber position_setpoint_sub_;
  geometry_msgs::Pose last_setpoint_;
};
#endif

#endif  // ROS_PLOTTERS_STATE_MONITOR_H