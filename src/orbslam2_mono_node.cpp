/**
 * @file orbslam_node.cpp
 * @brief almost same as ros_mono.cc
 */

#include <ros/ros.h>
#include <ORB_SLAM2/System.h>
#include <cv_bridge/cv_bridge.h>

class ImageGrabber
{
public:
  ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){};

  void grabImage(const sensor_msgs::ImageConstPtr& msg)
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

    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
  }

  ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orbslam_node");
  ros::start();

  if (argc != 3)
  {
    std::cerr << std::endl << "Usage: rosrun orbslam2_ros orbslam2_node path_to_vocabulary path_to_settings" << std::endl;
    ros::shutdown();
    return 1;
  }

  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  ImageGrabber igb(&SLAM);

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, &ImageGrabber::grabImage, &igb);

  ros::spin();

  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_rosnode.txt");

  ros::shutdown();
  return 0;
}
