catkin_make  
source devel/setup.bash

# apriltag 
## 安装 apriltag
git clone https://github.com/AprilRobotics/apriltag.git  
cd PATH_TO_APRILTAG  
mkdir build  
cd build   
make ..  
cd ..  

## 使用
1. 运行：
```
source devel/setup.bash
roslaunch fsm iris_realsense_camera_px4_mavros_vo.launch
```
2. 运行脚本：
```
./shell_all.sh
```

