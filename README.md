# 2023년 개별연구 - LiDAR 센서 다루기

SDK: https://github.com/slamtec/rplidar_ros

## 빌드 방법

   1) Clone, move into catkin_ws and build
   ```bash
   $ cd ~/.../catkin_ws
   $ catkin_make
   ```
   2) Set ROS Workspace
   ```bash
   source .../catkin_ws/devel/setup.bash
   ``````

## 실행 방법
### I. rviz를 사용한 그래픽 인터페이스

```bash
$ roslaunch rplidar_ros view_rplidar_a3.launch
```

### II. Server/Client방식
   1) Server
   ```bash
   $ roslaunch rplidar_ros rplidar_a3.launch
   ```
   2) Client
   ```bash
   $ rosrun rplidar_ros rplidarNodeClient
   ```
