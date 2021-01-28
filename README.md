# plane_fit_ground_filter

点云分割论文2017 Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications


## plane_fit_ground_filter

> 参考：https://github.com/AbangLZU/plane_fit_ground_filter


## Run_based_segmentation

> 参考：https://github.com/VincentCheungM/Run_based_segmentation



## Requirement

* [PCL](https://github.com/PointCloudLibrary/pcl)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [ROS Velodyne_driver](https://github.com/ros-drivers/velodyne)

安装`velodyne_pointcloud`  官网链接：[http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16](https://links.jianshu.com/go?to=http%3A%2F%2Fwiki.ros.org%2Fvelodyne%2FTutorials%2FGetting%2520Started%2520with%2520the%2520Velodyne%2520VLP16)

```shell
# melodic
sudo apt-get install ros-melodic-velodyne
# kinetic
sudo apt-get install ros-kinetic-velodyne
```


### Run

```bash
$ catkin_make
$ rosrun points_preprocessor_usi groundplanfit
$ rosrun points_preprocessor_usi scanlinerun
```

And cluster point cloud will be published as `cluster` with different label.


## License

Copyright (c) [双愚](https://github.com/HuangCongQing/). All rights reserved.

Licensed under the [BSD 3-Clause License](./LICENSE) License.
