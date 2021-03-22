/**
 * @file interface.cpp
 */

#include "orbslam2_ros/interface.h"
#include <ORB_SLAM2/System.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace orbslam2_ros
{

ORBSLAM2Interface::ORBSLAM2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private,
                                     ORB_SLAM2::System::eSensor sensor_type)
  : _nh(nh), _nh_private(nh_private), _sensor_type(sensor_type)
{
  // initialize
  advertiseTopics();
  advertiseServices();
  getParametersFromROS();
}

void ORBSLAM2Interface::advertiseTopics()
{
  // Advetising topics
  _trans_pub = _nh_private.advertise<geometry_msgs::TransformStamped>("transform_cam", 1);
  // Creating a callback time for TF publisher
  _tf_timer = _nh.createTimer(ros::Duration(0.01),
                              &ORBSLAM2Interface::publishCurrentPoseAsTF, this);

  // Add extra topics rather than ethz-asl/orb_slam_2_ros
  _states_pub = _nh_private.advertise<orbslam2_ros_msgs::ORBSLAM2State>("state", 1);
}

void ORBSLAM2Interface::advertiseServices()
{
  _reset_srv = _nh_private.advertiseService("reset", &ORBSLAM2Interface::resetService, this);
  _switch_mode_srv = _nh_private.advertiseService(
      "switch_mode", &ORBSLAM2Interface::switchModeService, this);
  _save_trajectory_srv = _nh_private.advertiseService(
      "save_trajectory_or_kf", &ORBSLAM2Interface::saveTrajectoryService, this);
}

void ORBSLAM2Interface::getParametersFromROS()
{
  bool is_voc_prm = _nh_private.getParam("vocabulary_file_path", _vocabulary_file_path);
  bool is_set_prm = _nh_private.getParam("setting_file_path", _setting_file_path);
  if (!is_voc_prm || !is_set_prm)
  {
    ROS_FATAL("Please provide the ~vocabulary_file_path and ~setting_file_path as ROS parameters");
    exit(-1);
  }

  _frame_id = "cam0";
  _child_frame_id = "world";
  if (!_nh_private.getParam("frame_id", _frame_id))
    ROS_WARN("Could not read parameter ~frame_id");
  if (!_nh_private.getParam("child_frame_id", _child_frame_id))
    ROS_WARN("Could not read parameter ~child_frame_id");

  ROS_INFO_STREAM("Parameters for ORB_SLAM2:" << endl
      << "vocabulary_file_path:" << _vocabulary_file_path << endl
      << "setting_file_path:" << _setting_file_path << endl
      << "frame_id:" << _frame_id << endl
      << "child_frame_id:" << _child_frame_id);
}

//void ORBSLAM2Interface::imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{

//}

void ORBSLAM2Interface::publishState()
{
  orbslam2_ros_msgs::ORBSLAM2State msg;
  msg.header.stamp = ros::Time::now();

  msg.is_changed = _slam_system->MapChanged();
  msg.tracking_state = (int8_t)_slam_system->GetTrackingState();

  _states_pub.publish(msg);
}

void ORBSLAM2Interface::publishCurrentPose(const Eigen::Affine3d& T,
                                           const std_msgs::Header& header)
{
  // Publish pose from camera's frame_id to world(Camera is parents link of world in TF)
  geometry_msgs::TransformStamped msg;
  msg.header = header;
  msg.child_frame_id = _child_frame_id;

  tf::transformEigenToMsg(T, msg.transform);

  _trans_pub.publish(msg);
}

void ORBSLAM2Interface::publishCurrentPoseAsTF(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = _frame_id;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = _child_frame_id;
  tf::transformEigenToMsg(_camera_T_world, msg.transform);

  _tf_broadcaster.sendTransform(msg);
}

void ORBSLAM2Interface::convertORBSLAMPoseToEigen(const cv::Mat& T_cv,
                                                  Eigen::Affine3d& T)
{
  Eigen::Matrix4f T_eigen_f;
  cv::cv2eigen(T_cv, T_eigen_f);
  // Eigen matrix (float) to Eigen matrix (double)
  Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();

  Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
  Eigen::AngleAxisd aa(R_unnormalized);
  Eigen::Matrix3d R = aa.toRotationMatrix();
  //Eigen::Quaterniond q(R);
  Eigen::Vector3d t(T_eigen_d.block<3, 1>(0, 3));

  Eigen::Translation<double, 3> trans(t);
  Eigen::Affine3d iso = trans * R;
  T = iso;
}

bool ORBSLAM2Interface::resetService(orbslam2_ros_msgs::Reset::Request &req,
                                     orbslam2_ros_msgs::Reset::Response &res)
{
  _slam_system->Reset();
  ROS_INFO("ORB_SLAM2 Reset function is called, and the map is cleared.");
  return true;
}

bool ORBSLAM2Interface::switchModeService(orbslam2_ros_msgs::SwitchMode::Request &req,
                                          orbslam2_ros_msgs::SwitchMode::Response &res)
{
  if (req.is_localization)
  {
    _slam_system->ActivateLocalizationMode();
    ROS_INFO("ORB_SLAM2 ActivateLocalizationMode function is called, and localization mode starts");
  }
  else
  {
    _slam_system->DeactivateLocalizationMode();
    ROS_INFO("ORB_SLAM2 DeactivateLocalizationMode function is called, and mapping mode starts");
  }
  return true;
}

bool ORBSLAM2Interface::saveTrajectoryService(orbslam2_ros_msgs::SaveTrajectory::Request &req,
                                              orbslam2_ros_msgs::SaveTrajectory::Response &res)
{
  ROS_INFO("ORB_SLAM2 shutdown function is called, for saving trajectorys");
  _slam_system->Shutdown();

  auto stamp = ros::WallTime::now();
  string filename_postfix = to_string(stamp.toNSec());
  if (_sensor_type == ORB_SLAM2::System::eSensor::MONOCULAR)
  {
    res.trajectory_file_path = ""; // empty
    res.keyframe_file_path = req.directory_path + "/kf_" + filename_postfix;
    _slam_system->SaveKeyFrameTrajectoryTUM(res.keyframe_file_path);
    ROS_INFO("Sensor type is monocular, only SaveKeyFrameTrajectoryTUM is called");
  }
  else
  {
    res.trajectory_file_path = req.directory_path + "/traj_" + filename_postfix;
    res.keyframe_file_path = req.directory_path + "/kf_" + filename_postfix;
    _slam_system->SaveTrajectoryTUM(res.trajectory_file_path);
    _slam_system->SaveKeyFrameTrajectoryTUM(res.keyframe_file_path);
    ROS_INFO("Sensor type is no t monocular, both SaveTrajectoryTUM and SaveKeyFrameTrajectoryTUM are called");
  }
  ROS_INFO_STREAM("saved filepath is:" << res);

  /// @todo _slam_system should be respawn?

  return true;
}

} // namespace orbslam2_ros
