# LVI-SAM

This repository contains code that use LVI-SAM to build the map and save the map. The map include visual information and Lidar information. The map structure is:
- **your folder**
  - **camera**
    - **pose_graph**      (similar to vins-mono's pose graph)
  - **lidar**
    - **Corners**
    - **Surfs**
    - **Scans**
    - **SCDs**
    - cloudGlobal.pcd
    - cloudCorner.pcd
    - cloudSurfs.pcd
    - trajectory.pcd
    - transformations.pcd

---

## Dependency
The dependency of this repo is same as the official [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM). So if you occur a compile problem, we recommend you to compile the official LVI-SAM firstly. Right now we have only tested on Ubuntu 18.04 + ROS melodic environment.

---

## Compile
You can use the following commands to download and compile the package.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/x-Yabao/lvi_sam_savemap
cd ..
catkin_make
```

---

## Run the package on different datasets
1. [M2DGR dataset](https://github.com/SJTU-ViSYS/M2DGR)
- Change the config files.

- Run the launch file:
```
roslaunch lvi_sam run_m2dgr.launch
```
- Play existing bag files, e.g. gate_01.bag:
```
rosbag play gate_01.bag 
```
- After playing the rosbag, input **s** and **Enter** in the terminal to save the Visual Map, then press **Ctrl + C** to save the Lidar Map.
    
2. [KITTI raw dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php)
- Change the config files.

- Run the launch file:
 ```
 roslaunch lvi_sam run_kitti_xx_xx.launch
 ```
- Play existing bag files. Please note that you must use **KITTI raw dataset** rather than KITTI Odometry dataset, because the latter's IMU frequency is too low. If you want to use KITTI raw dataset for LVI-SAM, you need to get rosbag files firstly. You can get it refer to [LIO-SAM/config/doc/kitti2bag](https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag). 
```
rosbag play kitti_xxx.bag  
```
- After playing the rosbag, input **s** and **Enter** in the terminal to save the Visual Map, then press **Ctrl + C** to save the Lidar Map.

---

## Acknowledgement
  - [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
  - [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused)
