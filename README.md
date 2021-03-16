orbslam2_ros
====

Use [Forked Repository](https://github.com/Shuhei-YOSHIDA/ORB_SLAM2) of raulmur/ORB_SLAM2,
and execute `make install` in `ORB_SLAM2/build/`, `ORB_SLAM2/ThirdParty/DBoW2/build/`, and `ORB_SLAM2/ThirdParty/g2o/build`.

## Sample for mono-camera
You can try mono-camera SLAM of this package by using a launch file;`freiburg1_xyz.launch`.
* Download rosbag of TUM dataset 'fr1/xyz' from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#)
* Fix `vocaburaly_file_path` and `setting_file_path` in `params/param_freiburg1_xyz_example.yaml`
* Migrate rosbag file. (Rosbag for fr1/xyz may be too old to use in your environment)
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

## License
The source code is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt) as is the underlying library [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).

And this repository is based on [ethz-asl/orb_slam_2_ros](https://github.com/ethz-asl/orb_slam_2_ros).
