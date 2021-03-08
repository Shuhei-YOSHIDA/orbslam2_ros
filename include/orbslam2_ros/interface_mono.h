/**
 * @file interface_mono.h
 */

#ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_MONO_H
#define INCLUDE_ORBSLAM2_ROS_INTERFACE_MONO_H

#include "orbslam2_ros/interface.h"

namespace orbslam2_ros
{

class ORBSLAM2InterfaceMono : public ORBSLAM2Interface
{
 public:
   ORBSLAM2InterfaceMono(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private,
                         const bool visualization);

protected:
  void subscribeToTopics();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber _image_sub;
};

} // namespace orbslam2_ros

#endif /* ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_MONO_H */
