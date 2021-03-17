/**
 * @file interface_rgbd.cpp
 * @node Should rgb and depth image's coord system be written to setting_file
 *       especially when the systems are different?
 */

#include "orbslam2_ros/interface_rgbd.h"
#include "orbslam2_ros/interface.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace orbslam2_ros
{

ORBSLAM2InterfaceRGBD::ORBSLAM2InterfaceRGBD(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const bool visualization)
  : ORBSLAM2Interface(nh, nh_private)
{
  subscribeToTopics();
  ROS_INFO("Wait for ORB_SLAM2:System to wake up...");
  _slam_system = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(_vocabulary_file_path, _setting_file_path,
                            ORB_SLAM2::System::RGBD, visualization));
}

void ORBSLAM2InterfaceRGBD::subscribeToTopics()
{
  _rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
      _nh, "camera/rgb/image_raw", 1);
  _depth_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
      _nh, "camera/depth_registered/image_raw", 1);
  _sync = std::make_shared<message_filters::Synchronizer<rgbd_sync_pol>>(
      rgbd_sync_pol(10), *_rgb_sub, *_depth_sub);

  // Registering the synchronized image callback
  _sync->registerCallback(boost::bind(&ORBSLAM2InterfaceRGBD::rgbdImageCallback, this, _1, _2));
}

void ORBSLAM2InterfaceRGBD::rgbdImageCallback(
    const sensor_msgs::ImageConstPtr& rgb_msg,
    const sensor_msgs::ImageConstPtr& depth_msg)
{
  auto cv_load = [](cv_bridge::CvImageConstPtr& cv_ptr, const sensor_msgs::ImageConstPtr& msg) {
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
    return true;
  };
  cv_bridge::CvImageConstPtr cv_ptr_rgb;
  if (!cv_load(cv_ptr_rgb, rgb_msg)) return;
  cv_bridge::CvImageConstPtr cv_ptr_depth;
  if (!cv_load(cv_ptr_depth, depth_msg)) return;

  // Handling the image to ORB_SLAM for tracking(T_C_W is camera pose from world(initial pose) coord)
  cv::Mat T_C_W_opencv;

  T_C_W_opencv = _slam_system->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image,
                                         cv_ptr_rgb->header.stamp.toSec());
  // If Tracking is successful, update camera's pose
  if (!T_C_W_opencv.empty())
  {
    ROS_DEBUG("ORBSLAM2 rgbd image is tracked");
    Eigen::Affine3d T_C_W, T_W_C;
    convertORBSLAMPoseToEigen(T_C_W_opencv, T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, rgb_msg->header); ///@note from camera to world? rgb or depth coord?
    _camera_T_world = T_W_C;
  }

}

} // namespace orgbslam2_ros
