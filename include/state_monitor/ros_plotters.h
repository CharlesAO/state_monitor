#ifndef ROS_PLOTTERS_STATE_MONITOR_H
#define ROS_PLOTTERS_STATE_MONITOR_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "state_monitor/sub_plotter.h"

template <typename T, size_t num_subplots_, size_t plots_dim_>
class RosPlotter {
 public:
  RosPlotter(const ros::NodeHandle &nh, const std::shared_ptr<mglGraph> &gr,
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
  }

  // Copying crashes things (I think its to do with sharing memory with the
  // x11_window but I need to find out why)
  RosPlotter(const RosPlotter &other) = delete;
  RosPlotter &operator=(const RosPlotter &) = delete;

  void setSubscriberTopic(const std::string &topic) {
    sub_ = nh_.subscribe(topic, kQueueSize, &RosPlotter::callback, this);
  }

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
  OdometryPlotter(const ros::NodeHandle &nh,
                  const std::shared_ptr<mglGraph> &gr,
                  const double keep_data_for_secs,
                  const size_t num_subplots_wide,
                  const size_t num_subplots_high,
                  const size_t position_subplot_idx,
                  const size_t linear_velocity_subplot_idx,
                  const size_t orientation_subplot_idx,
                  const size_t angular_velocity_subplot_idx)
      : RosPlotter(nh, gr, keep_data_for_secs, num_subplots_wide,
                   num_subplots_high,
                   {"Position (m)", "Linear Velocity (m/s)",
                    "Orientation (rads)", "Angular Velocity (rads/s)"},
                   {position_subplot_idx, linear_velocity_subplot_idx,
                    orientation_subplot_idx, angular_velocity_subplot_idx}) {}

 private:
  void callback(const nav_msgs::OdometryConstPtr &msg) {
    const double t = msg->header.stamp.toSec();

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z,
                                 msg->pose.pose.orientation.w))
        .getRPY(roll, pitch, yaw);

    sub_plots_[POSITION].addDataPoint(t, msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sub_plots_[LINEAR_VELOCITY].addDataPoint(t, msg->twist.twist.linear.x,
                                             msg->twist.twist.linear.y,
                                             msg->twist.twist.linear.z);
    sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
    sub_plots_[ANGULAR_VELOCITY].addDataPoint(t, msg->twist.twist.angular.x,
                                              msg->twist.twist.angular.y,
                                              msg->twist.twist.angular.z);
  }

  enum PlotOrder { POSITION, LINEAR_VELOCITY, ORIENTATION, ANGULAR_VELOCITY };
};

class PointPlotter
    : public RosPlotter<geometry_msgs::PointStampedConstPtr, 1, 3> {
 public:
  PointPlotter(const ros::NodeHandle &nh, const std::shared_ptr<mglGraph> &gr,
               const double keep_data_for_secs, const size_t num_subplots_wide,
               const size_t num_subplots_high, const std::string title,
               const size_t subplot_idx)
      : RosPlotter(nh, gr, keep_data_for_secs, num_subplots_wide,
                   num_subplots_high, {title}, {subplot_idx}) {}

 private:
  void callback(const geometry_msgs::PointStampedConstPtr &msg) {
    const double t = msg->header.stamp.toSec();

    sub_plots_.front().addDataPoint(t, msg->point.x, msg->point.y,
                                    msg->point.z);
  }
};

class TransformPlotter
    : public RosPlotter<geometry_msgs::TransformStampedConstPtr, 2, 3> {
 public:
  TransformPlotter(const ros::NodeHandle &nh,
                   const std::shared_ptr<mglGraph> &gr,
                   const double keep_data_for_secs,
                   const size_t num_subplots_wide,
                   const size_t num_subplots_high,
                   const size_t position_subplot_idx,
                   const size_t orientation_subplot_idx)
      : RosPlotter(nh, gr, keep_data_for_secs, num_subplots_wide,
                   num_subplots_high, {"Position (m)", "Orientation (rads)"},
                   {position_subplot_idx, orientation_subplot_idx}) {}

 private:
  void callback(const geometry_msgs::TransformStampedConstPtr &msg) {
    const double t = msg->header.stamp.toSec();

    double roll, pitch, yaw;
    tf::Matrix3x3(
        tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                       msg->transform.rotation.z, msg->transform.rotation.w))
        .getRPY(roll, pitch, yaw);

    sub_plots_[POSITION].addDataPoint(t, msg->transform.translation.x,
                                      msg->transform.translation.y,
                                      msg->transform.translation.z);
    sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
  }

  enum PlotOrder { POSITION, ORIENTATION };
};

class PosePlotter
    : public RosPlotter<geometry_msgs::PoseStampedConstPtr, 2, 3> {
 public:
  PosePlotter(const ros::NodeHandle &nh, const std::shared_ptr<mglGraph> &gr,
              const double keep_data_for_secs, const size_t num_subplots_wide,
              const size_t num_subplots_high, const size_t position_subplot_idx,
              const size_t orientation_subplot_idx)
      : RosPlotter(nh, gr, keep_data_for_secs, num_subplots_wide,
                   num_subplots_high, {"Position (m)", "Orientation (rads)"},
                   {position_subplot_idx, orientation_subplot_idx}) {}

 private:
  void callback(const geometry_msgs::PoseStampedConstPtr &msg) {
    const double t = msg->header.stamp.toSec();

    double roll, pitch, yaw;
    tf::Matrix3x3(
        tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                       msg->pose.orientation.z, msg->pose.orientation.w))
        .getRPY(roll, pitch, yaw);

    sub_plots_[POSITION].addDataPoint(
        t, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
  }

  enum PlotOrder { POSITION, ORIENTATION };
};

class ImuPlotter : public RosPlotter<sensor_msgs::ImuConstPtr, 3, 3> {
 public:
  ImuPlotter(const ros::NodeHandle &nh, const std::shared_ptr<mglGraph> &gr,
             const double keep_data_for_secs, const size_t num_subplots_wide,
             const size_t num_subplots_high,
             const size_t linear_acceleration_subplot_idx,
             const size_t orientation_subplot_idx,
             const size_t angular_velocity_subplot_idx)
      : RosPlotter(nh, gr, keep_data_for_secs, num_subplots_wide,
                   num_subplots_high,
                   {"Linear Acceleration (m^2/s)", "Orientation (rads)",
                    "Angular Velocity (rads/s)"},
                   {linear_acceleration_subplot_idx, orientation_subplot_idx,
                    angular_velocity_subplot_idx}) {}

 private:
  void callback(const sensor_msgs::ImuConstPtr &msg) {
    const double t = msg->header.stamp.toSec();

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->orientation.x, msg->orientation.y,
                                 msg->orientation.z, msg->orientation.w))
        .getRPY(roll, pitch, yaw);

    sub_plots_[LINEAR_ACCELERATION].addDataPoint(t, msg->linear_acceleration.x,
                                                 msg->linear_acceleration.y,
                                                 msg->linear_acceleration.z);
    sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
    sub_plots_[ANGULAR_VELOCITY].addDataPoint(t, msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              msg->angular_velocity.z);
  }

  enum PlotOrder { LINEAR_ACCELERATION, ORIENTATION, ANGULAR_VELOCITY };
};

#endif  // ROS_PLOTTERS_STATE_MONITOR_H