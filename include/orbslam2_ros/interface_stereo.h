/**
 * @file interface_stereo.h
 */

#ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_STEREO
#define INCLUDE_ORBSLAM2_ROS_INTERFACE_STEREO

#include "orbslam2_ros/interface.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

namespace orbslam2_ros
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
  stereo_sync_pol;

class ORBSLAM2InterfaceStereo : public ORBSLAM2Interface
{
public:
  ORBSLAM2InterfaceStereo(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private,
                          const bool visualization);

protected:
  void subscribeToTopics();

  void stereoImageCallback(const sensor_msgs::ImageConstPtr& msg_left,
                           const sensor_msgs::ImageConstPtr& msg_right);

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> _left_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> _right_sub;
  std::shared_ptr<message_filters::Synchronizer<stereo_sync_pol>> _sync;

  // Undistortion in this class
  bool _do_rectify;
  cv::Mat _M1l, _M2l;
  cv::Mat _M1r, _M2r;
};

} // namespace orbslam2_ros

#endif /* ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_STEREO */
