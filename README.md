orbslam2_ros
====

Use [Forked Repository](https://github.com/Shuhei-YOSHIDA/ORB_SLAM2) of raulmur/ORB_SLAM2,
and execute `make install` in `ORB_SLAM2/build/`, `ORB_SLAM2/ThirdParty/DBoW2/build/`, and `ORB_SLAM2/ThirdParty/g2o/build`.

In addition, set environment variable by adding a code into your `.bashrc`.
```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/ORB_SLAM2
```

And this package depends on [orbslam2_ros_msgs](https:github.com/Shuhei-YOSHIDA/orbslam2_ros_msgs) package.

## ROS API
### Topics published
* `~/transform_cam` : [`geometry_msgs/TransformStamped`]
  - Transform from camera(frame_id) to map-origin(child_frame_id).
* `~/state` : [`orbslam2_ros_msgs/ORBSLAM2State`]
  - State of SLAM Algorithm.

### Topics subscribed
Image topics should be subcribed for SLAM.

For monocular,
* `~/camera/image_raw` : [`sensor_msgs/Image`]

For stereo,
* `~/camera/left/image_raw` : [`sensor_msgs/Image`]
* `~/camera/right/image_raw` : [`sensor_msgs/Image`]

For RGBD,
* `~/camera/rgbd/image_raw` : [`sensor_msgs/Image`]
* `~/camera/depth_registered/image_raw` : [`sensor_msgs/Image`]

### Services advertised
* `reset` : [`orbslam2_ros_msgs/Reset`]
  - Generated map is reset.
* `switch_mode` : [`orbslam2_ros_msgs/SwitchMode`]
  - Mode of algorithm is switched to mapping mode or localization mode.
* `save_trajectory_and_kf` : [`orbslam2_ros_msgs/SaveTrajectory`]
  - Save camera trajectory and keyframe trajectory into specified directory path.

## Sample for mono-camera
You can try mono-camera SLAM of this package by using a launch file;`freiburg1_xyz.launch`.
* Download rosbag of TUM dataset 'fr1/xyz' from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#)
* Fix `vocaburaly_file_path` and `setting_file_path` in `params/param_freiburg1_xyz_example.yaml`
* Migrate rosbag file. (The contents of rosbag for fr1/xyz may be too old to use in your environment)
* `rosbag play --clock --pause rgbd_dataset_freiburg1_xyz.bag #Use bag file which is migrated`

### Migration of rosbag file
```bash
$ rosbag check rgbd_dataset_freiburg1_xyz.bag -g rule.bmr
$ rosbag fix rgbd_dataset_freiburg1_xyz.bag new.bag rule.bmr
```

## Sample for stereo-camera
You can also try stereo-camera SLAM of this package by using a launch file;`V1_01_easy.launch`.
* Download rosbag of EuRoc dataset 'Vicon room 1 01' from [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
* Fix `vocaburaly_file_path` and `setting_file_path` in `params/param_V1_01_easy_example.yaml`
* `rosbag play --clock --pause V1_01_easy.bag`

## Sample for RGB-D camera
You can also try RGB-D camera SLAM of this package by using a launch file;`freiburg_xyz_rgbd.launch`.
* Use rosbag of TUM dataset 'fr1/xyz' from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#), which is same as the one of mono-camera example.
* Fix `vocaburaly_file_path` and `setting_file_path` in `params/param_freiburg1_xyz_example.yaml`
* Migrate rosbag file. (The contents of rosbag for fr1/xyz may be too old to use in your environment)
* `rosbag play --clock --pause rgbd_dataset_freiburg1_xyz.bag #Use bag file which is migrated`

In original `ORB_SLAM2/Example/RGB-D/TUM1.yaml`, value of DepthMapFactor: 5000.0 may be invalid when it is used with rosbag file.
Please keep in mind that the example uses a fixed file; `rgbd_TUM1.yaml`.

## License
The source code is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt) as is the underlying library [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).

And this repository is based on [ethz-asl/orb_slam_2_ros](https://github.com/ethz-asl/orb_slam_2_ros).
