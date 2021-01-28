## plane_fit_ground_filter

点云分割论文2017 Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications

## Introduction




## Dataset bag

数据集已处理好，放在百度网盘上，需要自己下载

* kitti_2011_09_26_drive_0005_synced.bag
* 链接: https://pan.baidu.com/s/1sYWHzF11RpyEW25cQ_iNGA  密码: b6pd

## plane_fit_ground_filter

> 参考：https://github.com/AbangLZU/plane_fit_ground_filter



### Run(Terminal)

```
# Terminal1
roscore

# Terminal2  注意修改bag路径
rosbag play ~/data/KittiRawdata/2011_09_26_drive_0005_sync/kitti_2011_09_26_drive_0005_synced.bag --loop

# Terminal3
roslaunch plane_ground_filter plane_ground_filter.launch
```




### Result


## Run_based_segmentation

> 参考：https://github.com/VincentCheungM/Run_based_segmentation



### Requirement

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


**修改输入Topic**

> Run_based_segmentation/nodes/ground_filter/groundplanfit.cpp

```
    node_handle_.param<std::string>("point_topic", point_topic_, " /kitti/velo/pointcloud");  // 输入topoc   /velodyne_points   OR  /kitti/velo/pointcloud

```

### Run(Terminal)

```
catkin_make # 编译

# Terminal1  注意修改bag路径
rosrun points_preprocessor_usi groundplanfit

# Terminal2
rosrun points_preprocessor_usi scanlinerun
```


And cluster point cloud will be published as `cluster` with different label.

### Result

![图片](https://cdn.nlark.com/yuque/0/2021/png/232596/1611823927608-e23ab8dd-cc9e-470a-8ef6-efad1fd086a6.png)


## License

Copyright (c) [双愚](https://github.com/HuangCongQing/). All rights reserved.

Licensed under the [BSD 3-Clause License](./LICENSE) License.
