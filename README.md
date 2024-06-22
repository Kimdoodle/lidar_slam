## Improving accuracy of Hector SLAM using DBSCAN 
DBSCAN을 활용하여 Hector SLAM과정에서의 정확도 향상 연구

### Hardware Info
OS: Ubuntu 20.04(ROS1 Noetic)\
HW: RPLiDAR A3 

### Installation
0. Project folder
```bash
cd ~/catkin_ws
mkdir -p src
cd src
```
1. RPLiDAR A3 SDK
```bash
git clone https://github.com/Slamtec/rplidar_ros.git
```
2. Hector SLAM
```bash
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```
modify `/hector_mapping/launch/mapping_default.launch` line 5, 6 into:
```xml
<arg name="base_frame" default="base_link"/>
<arg name="odom_frame" default="base_link"/>
```
3. This package
```bash
git clone https://github.com/Kimdoodle/lidar_slam.git
```
4. build
```bash
cd ~/catkin_ws
catkin_make
```

### Run
Option 1. Run both lidar and slam
```bash
roslaunch lidar_slam hector_lidar.launch
```

Option 2. Run seperately
```bash
roslaunch rplidar_ros rplidar_a3.launch
roslaunch lidar_slam hector_only.launch
```

### Parameters
Change params in `/launch/hectormapping.launch`
```xml
<param name="eps_ratio" value="5"/>
<param name="minpts" value="15"/>
<param name="remains" value="0.5"/>
```
1. `eps_ratio`(0~100) stands for the eps value in DBSCAN, calculated as n% of the Euclidean distance in each scan.
2. `minpts` stands for the minimum number of points required to form a cluster.
3. `remains`(0.0 ~ 1.0) stands for the remaining clusters, which sorts all clusters by the number of points and retains only n% of them. Other clusters' labels are turned into -1 (outliers).
