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

## Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
roslaunch lvi_sam run.launch
```

3. Play existing bag files:
```
rosbag play handheld.bag 
```


---

## Acknowledgement
  - [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
  - [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused)
