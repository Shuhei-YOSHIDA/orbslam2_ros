/**
 * @file interface.cpp
 */

#include "orbslam2_ros/interface.h"
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace orbslam2_ros
{

ORBSLAM2Interface::ORBSLAM2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
  : _nh(nh), _nh_private(nh_private)
{
  // initialize
  advertiseTopics();
  getParametersFromROS();
}

void ORBSLAM2Interface::advertiseTopics()
{
  // Advetising topics
  _trans_pub = _nh_private.advertise<geometry_msgs::TransformStamped>("transform_cam", 1);
  // Creating a callback time for TF publisher
  _tf_timer = _nh.createTimer(ros::Duration(0.01),
                              &ORBSLAM2Interface::publishCurrentPoseAsTF, this);
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
  if (_nh_private.getParam("frame_id", _frame_id))
    ROS_WARN("Could not read parameter ~frame_id");
  if (_nh_private.getParam("child_frame_id", _child_frame_id))
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

} // namespace orbslam2_ros
