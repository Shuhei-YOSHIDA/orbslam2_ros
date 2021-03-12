orbslam2_ros
====

Use [Forked Repository](https://github.com/Shuhei-YOSHIDA/ORB_SLAM2) of raulmur/ORB_SLAM2,
and execute `make install`

You can try this package by using a launch file;`freiburg1_xyz.launch`.
* Download rosbag of TUM dataset 'fr1/xyz' from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#)
* Fix `vocaburaly_file_path` and ` setting_file_path` in `params/param_example.yaml`
* Migrate rosbag file. (Rosbag for fr1/xyz may be too old to use in your environment)

## Migration of rosbag file
```bash
$ rosbag check rgbd_dataset_freiburg1_xyz.bag -g rule.bmr
$ rosbag fix rgbd_dataset_freiburg1_xyz.bag new.bag rule.bmr
```
