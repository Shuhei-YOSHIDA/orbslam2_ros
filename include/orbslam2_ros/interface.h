/**
 * @file interface.h
 */

#ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_H
#define INCLUDE_ORBSLAM2_ROS_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ORB_SLAM2/System.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "orbslam2_ros_msgs/Reset.h"
#include "orbslam2_ros_msgs/SwitchMode.h"
#include "orbslam2_ros_msgs/SaveTrajectory.h"


namespace orbslam2_ros
{

class ORBSLAM2Interface
{
public:
  ORBSLAM2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private,
                    ORB_SLAM2::System::eSensor sensor_type);

protected:
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromROS();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void publishCurrentPose(const Eigen::Affine3d& T, const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);

  void convertORBSLAMPoseToEigen(const cv::Mat& T_cv, Eigen::Affine3d& T);

  bool resetService(orbslam2_ros_msgs::Reset::Request &req,
                    orbslam2_ros_msgs::Reset::Response &res);

  bool switchModeService(orbslam2_ros_msgs::SwitchMode::Request &req,
                         orbslam2_ros_msgs::SwitchMode::Response &res);

  bool saveTrajectoryService(orbslam2_ros_msgs::SaveTrajectory::Request &req,
                             orbslam2_ros_msgs::SaveTrajectory::Response &res);

  ros::NodeHandle _nh;
  ros::NodeHandle _nh_private;

  ros::Publisher _trans_pub;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  ros::Timer _tf_timer;

  Eigen::Affine3d _camera_T_world;

  // Services
  ros::ServiceServer _reset_srv;
  ros::ServiceServer _switch_mode_srv;
  ros::ServiceServer _save_trajectory_srv;

  // Parameters
  std::string _vocabulary_file_path;
  std::string _setting_file_path;

  std::string _frame_id;
  std::string _child_frame_id;

  // The ORB_SLAM2 system
  std::shared_ptr<ORB_SLAM2::System> _slam_system;

  const ORB_SLAM2::System::eSensor _sensor_type;
};

}

#endif /* ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_H */
