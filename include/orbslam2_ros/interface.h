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


namespace orbslam2_ros
{

class ORBSLAM2Interface
{
public:
  ORBSLAM2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

protected:
  void advertiseTopics();
  void getParametersFromROS();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void publishCurrentPose(const Eigen::Affine3d& T, const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);

  //void convertORBSLAMPoseToEigen(const cv::Mat& T_cv, Eigen::Isometry3d& T);
  void convertORBSLAMPoseToEigen(const cv::Mat& T_cv, Eigen::Affine3d& T);

  ros::NodeHandle _nh;
  ros::NodeHandle _nh_private;

  ros::Publisher _trans_pub;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  ros::Timer _tf_timer;

  //geometry_msgs::TransformStamped _world_T_camera;
  //Eigen::Matrix4d _world_T_camera;
  //Eigen::Isometry3d _world_T_camera;
  Eigen::Affine3d _world_T_camera;

  // Parameters
  std::string _vocabulary_file_path;
  std::string _setting_file_path;

  std::string _frame_id;
  std::string _child_frame_id;

  // The ORB_SLAM2 system
  std::shared_ptr<ORB_SLAM2::System> _slam_system;
};

}

#endif /* ifndef INCLUDE_ORBSLAM2_ROS_INTERFACE_H */
