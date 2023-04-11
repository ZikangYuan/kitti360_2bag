# kitti-360_2bag

**This sourced code is for converting data of [KITTI-360 dataset](https://www.cvlibs.net/datasets/kitti-360/) to ROS bag format.**

## Guide

1. Installation dependency

> GCC >= 5.4.0
>
> Cmake >= 3.0.2
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2.8
>
> [PCL](https://pointclouds.org/downloads/) == 1.7 for Ubuntu 16.04, and == 1.8 for Ubuntu 18.04
>
> [OpenCV](https://opencv.org/releases/) == 2.4.9 for Ubuntu 16.04
>
> [ROS](http://wiki.ros.org/ROS/Installation)

2. Create a new ROS workspace

```
mkdir /kitti-360_2bag/src
cd /kitti-360_2bag/src
```

3. Clone and build this repository

```
git clone https://github.com/ZikangYuan/kitti-360_2bag.git
cd ..
catkin_make
```

4. Adjust directory structure

Please make sure that the directory format of KITTI-360 is as follow:

```
KITTI-360
├─────────data_2d_raw  
          ├──────────2013_05_28_drive_0000_sync
                     ├─────────────────────────image_00
                                               ├───────data_rect
                                                       ├────────0000000000.png
                                                       ├────────0000000001.png
                                                       ├────────......
                                                       └────────0000011517.png
                                               └───────timestamps.txt
                     ├─────────────────────────image_01
                     ├─────────────────────────image_02
                     └─────────────────────────image_03
          ├──────────2013_05_28_drive_0002_sync
          ├──────────2013_05_28_drive_0003_sync
          ├──────────2013_05_28_drive_0004_sync
          ├──────────2013_05_28_drive_0005_sync
          ├──────────2013_05_28_drive_0006_sync
          ├──────────2013_05_28_drive_0007_sync
          ├──────────2013_05_28_drive_0009_sync
          └──────────2013_05_28_drive_0010_sync
├─────────data_3d_raw
          ├──────────2013_05_28_drive_0000_sync
                     ├─────────────────────────velodyne_points
                                               ├──────────────data
                                                              ├───0000000000.bin
                                                              ├───0000000001.bin
                                                              ├───......
                                                              └───0000011517.bin
                                               └──────────────timestamps.txt
          ├──────────2013_05_28_drive_0002_sync
          ├──────────......
          └──────────2013_05_28_drive_0010_sync
└─────────dara_poses
          ├──────────2013_05_28_drive_0000_sync
                     ├─────────────────────────cam0_ti_world.txt
                     └─────────────────────────poses.txt
          ├──────────2013_05_28_drive_0002_sync
          ├──────────......
          └──────────2013_05_28_drive_0010_sync
```
