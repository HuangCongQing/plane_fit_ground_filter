## plane_fit_ground_filter

点云分割论文2017 Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications

@[双愚](https://github.com/HuangCongQing/) , 若fork或star请注明来源

```
@inproceedings{Zermas2017Fast,
  title={Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2017},
}
```

**相关算法（带中文详细注解）：https://github.com/HuangCongQing/linefit_ground_segmentation_details**

## Introduction

笔记：https://www.yuque.com/huangzhongqing/ngixrc/lum14t

## Dataset bag

数据集已处理好，放在百度网盘上，需要自己下载

* kitti_2011_09_26_drive_0005_synced.bag
* 链接: https://pan.baidu.com/s/1sYWHzF11RpyEW25cQ_iNGA  密码: b6pd

## 编译

将本仓库下的2个文件夹`plane_fit_ground_filter&Run_based_segmentation`移动到catkin_wp/src下，然后执行下面操作

```shell
// 创建环境变量 src中运行
mkdir -p catkin_wp/src
cd catkin_wp/src
catkin_init_workspace

// 编译（需要回到工作空间catkin_wp）
cd ..
catkin_make  // 产生build和devel文件夹


//设置环境变量，找到src里的功能包(每个新的shell窗口都要执行以下source devel/setup.bash)
source devel/setup.bash  // 不同shell，不同哦.sh  .zsh           通过设置gedit ~/.zshrc，不用每次都source
```

详情可参考：https://www.yuque.com/docs/share/e59d5c91-b46d-426a-9957-cd262f5fc241?# 《09.创建工作空间与功能包※※※》

## plane_fit_ground_filter

> 参考：https://github.com/AbangLZU/plane_fit_ground_filter

### 修改配置文件


举例：修改输入topic,需要修改两处

```bash
cd plane_fit_ground_filter/src/plane_ground_filter_core.cpp
# 16行  需要修改 "/kitti/velo/pointcloud"
sub_point_cloud_ = nh.subscribe("/kitti/velo/pointcloud", 10, &PlaneGroundFilter::point_cb, this)

cd plane_fit_ground_filter/plane_ground_filter.launch

#第2行 修改 value="/kitti/velo/pointcloud"  修改你的雷达点云话题
<arg name="input_topic" default="/kitti/velo/pointcloud" />     <!-- 输入topic   原始 default="/velodyne_points"    OR /kitti/velo/pointcloud-->   

```


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

![result](https://cdn.nlark.com/yuque/0/2021/png/232596/1611824743464-99d29a4e-e336-492d-8ebf-ae99ec28a89e.png)

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

### 修改配置文件

举例：修改输入topic

```bash
cd Run_based_segmentation/nodes/ground_filter/groundplanfit.cpp

#第129行 修改  node_handle_.param<std::string>("point_topic", point_topic_, "/kitti/velo/pointcloud");  
node_handle_.param<std::string>("point_topic", point_topic_, "/kitti/velo/pointcloud");  // 输入topoc   /velodyne_points   OR  /kitti/velo/pointcloud

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
