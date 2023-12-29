# 2023년 개별연구 - LiDAR 센서 다루기

https://github.com/slamtec/rplidar_ros


## 빌드 방법

   1) Clone this project to your catkin's workspace src folder
   2) Running catkin_make to build rplidarNode and rplidarNodeClient

## 실행 방법
### I. rviz를 사용한 그래픽 인터페이스

```bash
roslaunch rplidar_ros view_rplidar_a3.launch
```

### II. Server/Client방식

```bash
roslaunch rplidar_ros rplidar_a3.launch
```

## RPLidar frame

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png
