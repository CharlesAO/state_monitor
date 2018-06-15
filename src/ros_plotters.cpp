#include "state_monitor/ros_plotters.h"

OdometryPlotter::OdometryPlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t position_subplot_idx, const size_t linear_velocity_subplot_idx,
    const size_t orientation_subplot_idx,
    const size_t angular_velocity_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Position (m)", "Linear Velocity (m/s)", "Orientation (rads)",
                  "Angular Velocity (rads/s)"},
                 {position_subplot_idx, linear_velocity_subplot_idx,
                  orientation_subplot_idx, angular_velocity_subplot_idx}) {}

void OdometryPlotter::callback(const nav_msgs::OdometryConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(
                    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w))
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

PointPlotter::PointPlotter(const ros::NodeHandle &nh, const std::string &topic,
                           const std::shared_ptr<mglGraph> &gr,
                           const double keep_data_for_secs,
                           const size_t num_subplots_wide,
                           const size_t num_subplots_high,
                           const std::string title, const size_t subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high, {title}, {subplot_idx}) {}

void PointPlotter::callback(const geometry_msgs::PointStampedConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_.front().addDataPoint(t, msg->point.x, msg->point.y, msg->point.z);
}

TransformPlotter::TransformPlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t position_subplot_idx, const size_t orientation_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high, {"Position (m)", "Orientation (rads)"},
                 {position_subplot_idx, orientation_subplot_idx}) {}

void TransformPlotter::callback(
    const geometry_msgs::TransformStampedConstPtr &msg) {
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

PosePlotter::PosePlotter(const ros::NodeHandle &nh, const std::string &topic,
                         const std::shared_ptr<mglGraph> &gr,
                         const double keep_data_for_secs,
                         const size_t num_subplots_wide,
                         const size_t num_subplots_high,
                         const size_t position_subplot_idx,
                         const size_t orientation_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high, {"Position (m)", "Orientation (rads)"},
                 {position_subplot_idx, orientation_subplot_idx}) {}

void PosePlotter::callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w))
      .getRPY(roll, pitch, yaw);

  sub_plots_[POSITION].addDataPoint(t, msg->pose.position.x,
                                    msg->pose.position.y, msg->pose.position.z);
  sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
}

ImuPlotter::ImuPlotter(const ros::NodeHandle &nh, const std::string &topic,
                       const std::shared_ptr<mglGraph> &gr,
                       const double keep_data_for_secs,
                       const size_t num_subplots_wide,
                       const size_t num_subplots_high,
                       const size_t linear_acceleration_subplot_idx,
                       const size_t angular_velocity_subplot_idx)
    : RosPlotter(
          nh, topic, gr, keep_data_for_secs, num_subplots_wide,
          num_subplots_high,
          {"Linear Acceleration (m^2/s)", "Angular Velocity (rads/s)"},
          {linear_acceleration_subplot_idx, angular_velocity_subplot_idx}) {}

void ImuPlotter::callback(const sensor_msgs::ImuConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[LINEAR_ACCELERATION].addDataPoint(t, msg->linear_acceleration.x,
                                               msg->linear_acceleration.y,
                                               msg->linear_acceleration.z);
  sub_plots_[ANGULAR_VELOCITY].addDataPoint(t, msg->angular_velocity.x,
                                            msg->angular_velocity.y,
                                            msg->angular_velocity.z);
}

ImuBiasPlotter::ImuBiasPlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t linear_acceleration_bias_subplot_idx,
    const size_t angular_velocity_bias_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Linear Acceleration Bias (m^2/s)",
                  "Angular Velocity Bias (rads/s)"},
                 {linear_acceleration_bias_subplot_idx,
                  angular_velocity_bias_subplot_idx}) {}

void ImuBiasPlotter::callback(const sensor_msgs::ImuConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[LINEAR_ACCELERATION_BIAS].addDataPoint(
      t, msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z);
  sub_plots_[ANGULAR_VELOCITY_BIAS].addDataPoint(t, msg->angular_velocity.x,
                                                 msg->angular_velocity.y,
                                                 msg->angular_velocity.z);
}

JoyPlotter::JoyPlotter(const ros::NodeHandle &nh, const std::string &topic,
                       const std::shared_ptr<mglGraph> &gr,
                       const double keep_data_for_secs,
                       const size_t num_subplots_wide,
                       const size_t num_subplots_high,
                       const size_t joy_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high, {"Joystick inputs"}, {joy_subplot_idx}) {}

void JoyPlotter::callback(const sensor_msgs::JoyConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_.back().addDataPoint(t, msg->axes[0], msg->axes[1], msg->axes[2],
                                 msg->axes[3], msg->axes[4], msg->axes[5]);
}

TrajectoryPlotter::TrajectoryPlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t position_subplot_idx, const size_t orientation_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Reference Position (m)", "Reference Orientation (rads)"},
                 {position_subplot_idx, orientation_subplot_idx}) {}

void TrajectoryPlotter::callback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[POSITION].addDataPoint(
      t, msg->points.front().transforms.front().translation.x,
      msg->points.front().transforms.front().translation.y,
      msg->points.front().transforms.front().translation.z);

  double roll, pitch, yaw;
  tf::Matrix3x3(
      tf::Quaternion(msg->points.front().transforms.front().rotation.x,
                     msg->points.front().transforms.front().rotation.y,
                     msg->points.front().transforms.front().rotation.z,
                     msg->points.front().transforms.front().rotation.w))
      .getRPY(roll, pitch, yaw);

  sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw);
}

#ifdef MSF_FOUND
MSFStatePlotter::MSFStatePlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t linear_acceleration_bias_subplot_idx,
    const size_t angular_velocity_bias_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Linear Acceleration Bias (m^2/s)",
                  "Angular Velocity Bias (rads/s)"},
                 {linear_acceleration_bias_subplot_idx,
                  angular_velocity_bias_subplot_idx}) {}

void MSFStatePlotter::callback(
    const sensor_fusion_comm::DoubleArrayStampedConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  constexpr size_t kAngularVelocityXBiasIdx = 10;
  constexpr size_t kAngularVelocityYBiasIdx = 11;
  constexpr size_t kAngularVelocityZBiasIdx = 12;
  constexpr size_t kLinearAccelerationXBiasIdx = 13;
  constexpr size_t kLinearAccelerationYBiasIdx = 14;
  constexpr size_t kLinearAccelerationZBiasIdx = 15;

  sub_plots_[ANGULAR_VELOCITY_BIAS].addDataPoint(
      t, msg->data[kAngularVelocityXBiasIdx],
      msg->data[kAngularVelocityYBiasIdx], msg->data[kAngularVelocityZBiasIdx]);
  sub_plots_[LINEAR_ACCELERATION_BIAS].addDataPoint(
      t, msg->data[kLinearAccelerationXBiasIdx],
      msg->data[kLinearAccelerationYBiasIdx],
      msg->data[kLinearAccelerationZBiasIdx]);
}
#endif

#ifdef MAV_CONTROL_RW_FOUND
ObserverStatePlotter::ObserverStatePlotter(
    const ros::NodeHandle &nh, const std::string &observer_topic,
    const std::string &ref_topic, const std::shared_ptr<mglGraph> &gr,
    const double keep_data_for_secs, const size_t num_subplots_wide,
    const size_t num_subplots_high, const size_t position_subplot_idx,
    const size_t orientation_subplot_idx,
    const size_t external_forces_subplot_idx,
    const size_t external_moments_subplot_idx)
    : RosPlotter(nh, observer_topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Position (m)", "Orientation (rads)", "External Forces (N)",
                  "External Moments (Nm)"},
                 {position_subplot_idx, orientation_subplot_idx,
                  external_forces_subplot_idx, external_moments_subplot_idx}) {
  ref_sub_ = nh_.subscribe(ref_topic, kQueueSize,
                           &ObserverStatePlotter::refCallback, this);
}

void ObserverStatePlotter::callback(
    const mav_disturbance_observer::ObserverStateConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[POSITION].addDataPoint(t, msg->position[0], msg->position[1],
                                    msg->position[2], last_ref_.translation.x,
                                    last_ref_.translation.y,
                                    last_ref_.translation.z);

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(last_ref_.rotation.x, last_ref_.rotation.y,
                               last_ref_.rotation.z, last_ref_.rotation.w))
      .getRPY(roll, pitch, yaw);

  sub_plots_[ORIENTATION].addDataPoint(t, msg->attitude[0], msg->attitude[1],
                                       msg->attitude[2], roll, pitch, yaw);

  sub_plots_[EXTERNAL_FORCES].addDataPoint(
      t, msg->external_forces[0], msg->external_forces[1],
      msg->external_forces[2], NAN, NAN, NAN);
  sub_plots_[EXTERNAL_MOMENTS].addDataPoint(
      t, msg->external_moments[0], msg->external_moments[1],
      msg->external_moments[2], NAN, NAN, NAN);
}

void ObserverStatePlotter::refCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {
  last_ref_ = msg->points[0].transforms[0];
}

RPYRateThrustPlotter::RPYRateThrustPlotter(
    const ros::NodeHandle &nh, const std::string &topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t rpy_rate_subplot_idx, const size_t thrust_subplot_idx)
    : RosPlotter(nh, topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high,
                 {"Roll Pitch Yaw rate (rads)", "Thrust (N)"},
                 {rpy_rate_subplot_idx, thrust_subplot_idx}) {}

void RPYRateThrustPlotter::callback(
    const mav_msgs::RollPitchYawrateThrustConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[RPY_RATE].addDataPoint(t, msg->roll, msg->pitch, msg->yaw_rate);
  sub_plots_[THRUST].addDataPoint(t, msg->thrust.x, msg->thrust.y,
                                  msg->thrust.z);
}
#endif

#ifdef MAVROS_FOUND
MAVROSPosePlotter::MAVROSPosePlotter(
    const ros::NodeHandle &nh, const std::string &pose_topic,
    const std::string &attitude_setpoint_topic,
    const std::string &position_setpoint_topic,
    const std::shared_ptr<mglGraph> &gr, const double keep_data_for_secs,
    const size_t num_subplots_wide, const size_t num_subplots_high,
    const size_t position_subplot_idx, const size_t orientation_subplot_idx)
    : RosPlotter(nh, pose_topic, gr, keep_data_for_secs, num_subplots_wide,
                 num_subplots_high, {"Position (m)", "Orientation (rads)"},
                 {position_subplot_idx, orientation_subplot_idx}) {
  attitude_setpoint_sub_ =
      nh_.subscribe(attitude_setpoint_topic, kQueueSize,
                    &MAVROSPosePlotter::attitudeSetpointCallback, this);
  position_setpoint_sub_ =
      nh_.subscribe(position_setpoint_topic, kQueueSize,
                    &MAVROSPosePlotter::positionSetpointCallback, this);
}

void MAVROSPosePlotter::callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  const double t = msg->header.stamp.toSec();

  sub_plots_[POSITION].addDataPoint(
      t, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      last_setpoint_.position.x, last_setpoint_.position.y,
      last_setpoint_.position.z);

  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w))
      .getRPY(roll, pitch, yaw);

  double setpoint_roll, setpoint_pitch, setpoint_yaw;
  tf::Matrix3x3(tf::Quaternion(
                    last_setpoint_.orientation.x, last_setpoint_.orientation.y,
                    last_setpoint_.orientation.z, last_setpoint_.orientation.w))
      .getRPY(setpoint_roll, setpoint_pitch, setpoint_yaw);

  sub_plots_[ORIENTATION].addDataPoint(t, roll, pitch, yaw, setpoint_roll,
                                       setpoint_pitch, setpoint_yaw);
}

void MAVROSPosePlotter::attitudeSetpointCallback(
    const mavros_msgs::AttitudeTargetConstPtr &msg) {
  last_setpoint_.orientation = msg->orientation;
}

void MAVROSPosePlotter::positionSetpointCallback(
    const mavros_msgs::PositionTargetConstPtr &msg) {
  last_setpoint_.position = msg->position;
}
#endif
