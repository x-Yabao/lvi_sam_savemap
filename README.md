# LVI-SAM

This repository contains code that use LVI-SAM to build the map and save the map.




---

## Dependency
The dependency of this repo is same as the official [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM). So if you occur a compile problem, we recommend you to compile the official LVI-SAM firstly. Right now we have only tested on Ubuntu 18.04 + ROS melodic environment.

---

## Compile

You can use the following commands to download and compile the package.

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Cc19245/LVI-SAM-Easyused
cd ..
catkin_make
```

---

## Run the package on different datasets
1. [M2DGR dataset](https://github.com/SJTU-ViSYS/M2DGR)
   - Run the launch file:
     ```
     roslaunch lvi_sam run_m2dgr.launch
     ```
   - Play existing bag files, e.g. gate_01.bag:
     ```
     rosbag play gate_01.bag 
     ```
2. [KITTI dataset](https://github.com/SJTU-ViSYS/M2DGR)
   - Run the launch file:
     ```
     roslaunch lvi_sam run_kitti_09_30.launch
     ```
   - Play existing bag files, e.g. gate_01.bag:
     ```
     rosbag play gate_01.bag 
     ```

---

## Acknowledgement
  - [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
  - [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused)
