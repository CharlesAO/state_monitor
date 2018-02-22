# State Monitor
A simple GUI for monitoring the state of a range of ASL estimators

![pretty_gif](https://i.imgur.com/4sv2Ofa.gif)

# Dependencies

State Monitor depends on mathgl and X11. These can be installed via `sudo apt install libmgl-dev libx11-dev`

# Usage

To start State Monitor simply type `state_monitor` in a terminal. It will automatically search all advertised topics and subscribe to the nodes automatically. These nodes may be renamed or in groups, but should not have their outputs remapped to different topic names. Remapping inputs however, does not effect the subscription.

The list of subscribeable nodes is shown on the left side of the screen, the currently displayed node is shown in cyan. The displayed node can be changed by using the up/down arrow keys.
A reset service call to the currently selected node can be performed by pressing the 'r' key.

## Supported Nodes

* Rovio
* MSF
* SWF
* Mav Control RW
* Mavros

## Supported Ros Topics

* geometry_msgs/PointStamped
* nav_msgs/Odometry
* sensor_msgs/Imu
* sensor_msgs/Joy
* trajectory_msgs/MultiDOFJointTrajectory
* sensor_fusion_comm/DoubleArrayStamped
* mav_disturbance_observer/ObserverState
* mav_msgs/RollPitchYawrateThrust

## Extending

Additional ros topics can be supported by adding to those defined in `ros_plotters.h`.
Additional nodes can be supported by adding them to `node_plotters.h` and adding a topic that uniquely identifies the node in the `createNodePlotterFromTopicInfo` function in `state_monitor.cpp`
