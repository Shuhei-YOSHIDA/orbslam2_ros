/**
 * @file interface_rgbd.h
 */

#ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_RGBD
#define INCLUDE_ORBSLAM2_ROS_INTERFACE_RGBD

#include "orbslam2_ros/interface.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

namespace orbslam2_ros
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
  rgbd_sync_pol;

class ORBSLAM2InterfaceRGBD : public ORBSLAM2Interface
{
public:
  ORBSLAM2InterfaceRGBD(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private,
                        const bool visualization);

protected:
  void subscribeToTopics();

  void rgbdImageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                         const sensor_msgs::ImageConstPtr& depth_msg);

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> _rgb_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> _depth_sub;
  std::shared_ptr<message_filters::Synchronizer<rgbd_sync_pol>> _sync;

};

} // namespace orbslam2_ros

#endif /* ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_RGBD */
