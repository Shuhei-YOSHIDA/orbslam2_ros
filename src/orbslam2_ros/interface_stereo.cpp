/**
 * @file interface_stereo.cpp
 */

#include "orbslam2_ros/interface_stereo.h"
#include "orbslam2_ros/interface.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace orbslam2_ros
{

ORBSLAM2InterfaceStereo::ORBSLAM2InterfaceStereo(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private,
    const bool visualization)
  : ORBSLAM2Interface(nh, nh_private, ORB_SLAM2::System::STEREO)
{
  subscribeToTopics();
  ROS_INFO("Wait for ORB_SLAM2:System to wake up...");
  _slam_system = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(_vocabulary_file_path, _setting_file_path,
                            _sensor_type, visualization));

  // Load parameter ~do_rectify, and set undistortion-info if it is true
  _do_rectify = false;
  if (nh_private.getParam("do_rectify", _do_rectify))
  {
    if (_do_rectify)
    {
      ROS_INFO("rectify setting in slam node");
      // Load setting related to stereo calibration
      cv::FileStorage fs_settings(_setting_file_path, cv::FileStorage::READ);
      if (!fs_settings.isOpened())
      {
        ROS_FATAL("Wrong setting_file_path for stereo, exit(-1)");
        exit(-1);
      }

      cv::Mat K_l, P_l, R_l, D_l;
      cv::Mat K_r, P_r, R_r, D_r;

      fs_settings["LEFT.K"] >> K_l;
      fs_settings["RIGHT.K"] >> K_r;

      fs_settings["LEFT.P"] >> P_l;
      fs_settings["RIGHT.P"] >> P_r;

      fs_settings["LEFT.R"] >> R_l;
      fs_settings["RIGHT.R"] >> R_r;

      fs_settings["LEFT.D"] >> D_l;
      fs_settings["RIGHT.D"] >> D_r;

      int rows_l = fs_settings["LEFT.height"];
      int cols_l = fs_settings["LEFT.width"];
      int rows_r = fs_settings["RIGHT.height"];
      int cols_r = fs_settings["RIGHT.width"];

      if (K_l.empty() || P_l.empty() || R_l.empty() || D_l.empty() ||
          K_r.empty() || P_r.empty() || R_r.empty() || D_r.empty() ||
          rows_l == 0 || cols_l == 0 ||
          rows_r == 0 || cols_r == 0)
      {
        ROS_FATAL("Could not interplet rectify-parameters, exit(-1)");
        exit(-1);
      }

      cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
          cv::Size(cols_l, rows_l), CV_32F, _M1l, _M2l);
      cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
          cv::Size(cols_r, rows_r), CV_32F, _M1r, _M2r);

      ROS_INFO("Images are rectified in stereo ORB_SLAM2 node");
    }
    else ROS_INFO("Images are not rectified, please publish rectified image");
  }
  else
  {
    ROS_WARN("Could not read parameter ~do_rectify, Images are not rectified");
  }
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
  cv::Mat T_C_W_opencv;
  if (_do_rectify)
  {
    cv::Mat im_left, im_right;
    cv::remap(cv_ptr_left->image, im_left, _M1l, _M2l, cv::INTER_LINEAR);
    cv::remap(cv_ptr_right->image, im_right, _M1r, _M2r, cv::INTER_LINEAR);
    T_C_W_opencv = _slam_system->TrackStereo(im_left, im_right,
                                             cv_ptr_left->header.stamp.toSec());
  }
  else
  {
    T_C_W_opencv = _slam_system->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                             cv_ptr_left->header.stamp.toSec());
  }
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

  publishState();
}

} // namespace orgbslam2_ros
