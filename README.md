# TOF-VIO

TOF Camera Visual Initial Odometry

**Video:**

<a href="https://www.youtube.com/embed/IqfIqArsWXA" target="_blank"><img src="http://img.youtube.com/vi/IqfIqArsWXA/0.jpg" 
alt="cla" width="360" height="230" border="10" /></a>

Perception in the Dark

<img src="files/indark.gif" width="300">

Test in Lab on a Moving Platform

<img src="files/fj005.gif" width="300"> 

**Related Papers:**
### Introduction: 

### Prerequisites
Ubuntu + ROS We have tested in the following environment:<br />
Ubuntu 16.04 + [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)<br />
Ubuntu 18.04 + [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)<br />

### Build and Run
Clone the repository to the catkin work space eg. /catkin_ws/src
```
cd ~/catkin_ws/src
git clone https://github.com/HKPolyU-UAV/TOF-VIO.git
```
Compile
```
cd ~/catkin_ws
catkin_make
```
### Verify using Dataset
Using our recorded rosbag:

->[Link1](https://drive.google.com/open?id=1-mdz7wl5JyhxFYr9SoeClK4WtimJQYd_) Hand held test 

->[Link2](https://drive.google.com/open?id=1MgEL9vWcRwh5zFwe1Vh7nNjQmszu9h3I) Lab test(Changing of environment lighting condition)

->[Link3](https://drive.google.com/open?id=1eQtt0zhSFPT5nYd5PYAoZZP8JioHqfxa) UAV fly in corridor

The rosbag is [compressed](http://wiki.ros.org/rosbag/Commandline#compress), [depressed](http://wiki.ros.org/rosbag/Commandline#decompress) it before estimation.

Data Format of the rosbag

|   Topic Name  |             Content            | Frequency |
|:-------------:|:------------------------------:|:---------:|
| /image\_depth | Depth image (u,v,z)            |     15    |
| /image\_nir   | NIR image (u,v,i)              |     15    |
| /points       | Organized point cloud          |     15    |
| /imu          | IMU data                       |    250    |
| /gt           | Ground truth captured by Vicon |     50    |

Camera matrix and distortion coeffs of the Depth/NIR image

| camera matrix |           | distortion coeffs |         |
|---------------|-----------|-------------------|---------|
| fx            | 211.95335 | k1                | 0.57858 |
| fy            | 211.95335 | k2                | -5.7317 |
| cx            | 115.6517  | p1                | 0       |
| cy            | 87.125724 | p2                | 0       |
|               |           | p3                | 10.0098 |

Place the .bag file into bag folder then modify the bag.launch file 
```
<node pkg="rosbag" type="play" name="rosbag" args="$(find vio)/bag/nameofthebag.bag"/>
```
Run: <br />
```
roslaunch vio rviz.launch
```
```
roslaunch vio bag.launch
```

### Evaluation 
TUM scripts can be used to evaluate the result, the following are the exported of rosbag result:

Handheld test

<img src="files/HH.png" width="300">

UAV test

<img src="files/UAV.png" width="300">


### Tips for making own experiment platform
We can use light-weight PMD Flexx TOF-CAMERA: get [drivers](https://pmdtec.com/picofamily/software/) from the PMD website and install [ros wrapper](https://github.com/code-iai/pico_flexx_driver)
You can use a pixhawk as an IMU with [mavros](http://wiki.ros.org/mavros)

### Maintainer:
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />


