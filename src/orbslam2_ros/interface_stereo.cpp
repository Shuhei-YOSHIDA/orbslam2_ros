/**
 * @file interface_stereo.cpp
 */

#include "orbslam2_ros/interface_stereo.h"
#include "orbslam2_ros/interface.h"
#include <cv_bridge/cv_bridge.h>

namespace orbslam2_ros
{

ORBSLAM2InterfaceStereo::ORBSLAM2InterfaceStereo(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const bool visualization)
  : ORBSLAM2Interface(nh, nh_private)
{
  subscribeToTopics();
  _slam_system = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(_vocabulary_file_path, _setting_file_path,
                            ORB_SLAM2::System::STEREO, visualization));
}

void ORBSLAM2InterfaceStereo::subscribeToTopics()
{
  _left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
      _nh, "camera/left/image_raw", 1);
  _right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
      _nh, "camera/right/image_raw", 1);
  _sync = std::make_shared<message_filters::Synchronizer<stereo_sync_pol>>(
      stereo_sync_pol(10), *_left_sub, *_right_sub);

  // Registering the synchronized image callback
  _sync->registerCallback(boost::bind(&ORBSLAM2InterfaceStereo::stereoImageCallback, this, _1, _2));
}

void ORBSLAM2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right)
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
  cv_bridge::CvImageConstPtr cv_ptr_left;
  if (!cv_load(cv_ptr_left, msg_left)) return;
  cv_bridge::CvImageConstPtr cv_ptr_right;
  if (!cv_load(cv_ptr_right, msg_right)) return;

  // Handling the image to ORB_SLAM for tracking(T_C_W is camera pose from world(initial pose) coord)
   cv::Mat T_C_W_opencv = _slam_system->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                                    cv_ptr_left->header.stamp.toSec());
  // If Tracking is successful, update camera's pose
  if (!T_C_W_opencv.empty())
  {
    ROS_DEBUG("ORBSLAM2 stereo image is tracked");
    Eigen::Affine3d T_C_W, T_W_C;
    convertORBSLAMPoseToEigen(T_C_W_opencv, T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, msg_left->header); ///@note from camera to world?
    _camera_T_world = T_W_C;
  }

}

} // namespace orgbslam2_ros
