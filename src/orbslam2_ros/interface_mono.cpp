/**
 * @file interface_mono.cpp
 */

#include "orbslam2_ros/interface_mono.h"
#include "orbslam2_ros/interface.h"
#include "ros/node_handle.h"
#include <cv_bridge/cv_bridge.h>

namespace orbslam2_ros
{

ORBSLAM2InterfaceMono::ORBSLAM2InterfaceMono(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const bool visualization)
  : ORBSLAM2Interface(nh, nh_private)
{
  subscribeToTopics();
  _slam_system = std::shared_ptr<ORB_SLAM2::System>(
        new ORB_SLAM2::System(_vocabulary_file_path, _setting_file_path,
                              ORB_SLAM2::System::MONOCULAR, visualization));
}

void ORBSLAM2InterfaceMono::subscribeToTopics()
{
  _image_sub = _nh.subscribe("camera/image_raw", 1, &ORBSLAM2InterfaceMono::imageCallback, this);
}

void ORBSLAM2InterfaceMono::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Handling the iamge to ORB_SLAM for tracking
  cv::Mat T_C_W_opencv = _slam_system->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
  // If Tracking is successful, update camera's pose
  if (!T_C_W_opencv.empty())
  {
  //void convertORBSLAMPoseToEigen(const cv::Mat& T_cv, Eigen::Affine3d& T);
    Eigen::Affine3d T_C_W, T_W_C;
    convertORBSLAMPoseToEigen(T_C_W_opencv, T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, msg->header); ///@note from camera to world?
    _world_T_camera = T_W_C; ///@todo inversed?
  }
}

} // namespace orbslam2_ros