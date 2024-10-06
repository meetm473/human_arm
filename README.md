# human_arm
A ROS+Gazebo simulation package to control a human-like arm via:
- Keyboard
- IMU

This package uses scripts developed in [Cube-Visualization](https://github.com/meetm473/Cube-Visualization). Sensor data for IMU control is transmitted via the [AndyIMU-2](https://github.com/meetm473/AndyIMU) android application.

![drawing of the assem](https://github.com/meetm473/human_arm/blob/master/multimedia/Assem_drawing.PNG)
![4 view bot](https://github.com/meetm473/human_arm/blob/master/multimedia/sw.png)

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
Custom sensor fusion algorithm can be written in the file *d_sensorFusion.py*. For more instructions, check [this](https://github.com/meetm473/Cube-Visualization#adding-custom-sensor-fusion-algorithm).
