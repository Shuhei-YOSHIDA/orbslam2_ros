/**
 * @file orbslam2_ros_node.cpp
 */

#include "orbslam2_ros/interface.h"
#include "orbslam2_ros/interface_mono.h"

// A factory method for creating an interface
std::unique_ptr<orbslam2_ros::ORBSLAM2Interface>
create_interface(std::string interface_type, const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private, const bool visualization) {
  // Creating the aligner object subclasss dependent on the argument
  std::unique_ptr<orbslam2_ros::ORBSLAM2Interface> interface;
  if (interface_type == "mono")
  {
    interface = std::unique_ptr<orbslam2_ros::ORBSLAM2Interface>(
        new orbslam2_ros::ORBSLAM2InterfaceMono(nh, nh_private, visualization));
  }
  else if (interface_type == "stereo")
  {
    ROS_FATAL("interface type[stereo] is not implemented now. Must be mono");
    ros::shutdown();
    exit(1);
  }
  else
  {
    ROS_FATAL("interface type is not recognized. Must be mono or stereo");
    ros::shutdown();
    exit(1);
  }

  return interface;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orbslam2_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Set parameters
  std::string interface_type = "mono";
  nh_private.getParam("interface_type", interface_type);
  bool visualization = true;
  nh_private.getParam("visualization", visualization);

  // Creating the interface object to do the work
  std::unique_ptr<orbslam2_ros::ORBSLAM2Interface> interface =
    create_interface(interface_type, nh, nh_private, visualization);

  // Spinning
  ros::AsyncSpinner spinner(0); // 0:All cpu thread
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
