# human_arm
A ROS+Gazebo simulation package to control a human-like arm via:
- Keyboard
- IMU

This package uses scripts developed in [Cube-Visualization](1). Sensor data for IMU control is transmitted via the [AndyIMU-2](2) android application.

![drawing of the assem](link)
![4 view bot](link)

### Commands
To view the robot in RViz
```
roslaunch human_arm display-rviz.launch
```
To control the bot via keyboard in Gazebo:
```
roslaunch human_arm human_arm_gazebo.launch
rosrun human_arm key_control.py
```
> For help, press *h*

To control the bot via android IMU in Gazebo:
```
roslaunch human_arm human_arm_gazebo.launch
rosrun human_arm a_imu_control.py
```
Custom sensor fusion algorithm can be written in the file *d_sensorFusion.py*. For more instructions, check [this](3).

[1]:https://github.com/meetm473/Cube-Visualization
[2]:https://github.com/meetm473/AndyIMU
[3]:https://github.com/meetm473/Cube-Visualization#adding-custom-sensor-fusion-algorithm
