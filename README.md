# IMcoders
The IMcoders project is meant to offer to the robotic community an easy to mount, cheap and reliable device capable of substitute the wheel encoders in an already existing system, or to produce accurate odometry data for a wheeled device/robot without previous odometry support.  

## Dependencies
### ROS Kinetic
Link to official site:  
http://wiki.ros.org/kinetic/Installation/Ubuntu  
**NOTE:** The recomended version is _Desktop-Full Install_

### ROS IMU tools package
`sudo apt install ros-kinetic-imu-tools`  

### (Optional) RTIMULib2
Install this package if you want to use the `imcoder_reader` for using the hardware 
```
git clone -b NoQt https://github.com/Pablo-Leyva/RTIMULib2 ~/imcoders_ws/third-party/RTIMULib2
mkdir -p ~/imcoders_ws/third-party/RTIMULib2/RTHost/build && cd "$_"
cmake ..
sudo make install
sudo ldconfig
```

## How-to
1. Create a new ROS workspace or use your an existing one  
`mkdir -p ~/imcoders_ws/src`

1. Clone this repo  
`git clone --recursive -b devel https://github.com/solosito/IMcoders.git ~/imcoders_ws/src/.`  

    If you installed [RTIMULib2](https://github.com/solosito/IMcoders#optional-rtimulib2), remove the `CATKIN_IGNORE` file from `imcoder_reader` in order to also compile that package  
    `rm ~/imcoders_ws/src/imcoder_reader/CATKIN_IGNORE`

1. Build the code  
Go to the workspace folder  
`cd ~/imcoders_ws`  
then  
`catkin_make`  
or (in case you have the [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/installing.html))  
`catkin build`  

1. Source the workspace  
`source ~/imcoders_ws/devel/setup.bash`  

    (Optional) Source the workspace in your .bashrc so you don't have to source the workspace every time you open a new terminal  
    `echo "source ~/imcoders_ws/devel/setup.bash" >> ~/.bashrc`  

## Running simulation
### Box
1. Launch simulator  
`roslaunch imcoders_gazebo box_robot_gazebo.launch`  

1. Launch teleop from a new terminal (node you need to source the workspace as before if you did not modified your .bashrc)  
`roslaunch imcoders_control keyboard_teleop.launch`  

1. Launch Rviz for visualization  
`roslaunch imcoders_rviz_launchers view_box.launch`  

![](https://github.com/solosito/IMcoders/blob/devel/doc/images/box_robot_gazebo.png)  

![](https://github.com/solosito/IMcoders/blob/devel/doc/images/box_gz-rotation.gif)

### Differential wheeled robot with IMcoders  
1. Launch simulator  
`roslaunch imcoders_gazebo diff_wheeled_gazebo_IMcoders.launch`  

1. Launch teleop from a new terminal (node you need to source the workspace as before if you did not modified your .bashrc)  
`roslaunch imcoders_control keyboard_teleop.launch`  

1. Launch Rviz for visualization  
`roslaunch imcoders_rviz_launchers view_diff.launch`  

![](https://github.com/solosito/IMcoders/blob/devel/doc/images/diff_robot_gazebo.png)

## Running with hardware
1. Source the third party packages  
`source ~/imcoders_ws/third-party/RTIMULib2/RTHost/build/devel/setup.bash`  

1. Launch node for sensors  
`roslaunch imcoder_reader imcoders_all_timed.launch`  

1. Launch visualization  
`roslaunch imcoders_rviz_launchers view_imcoders_boxes.launch`  

![](https://github.com/solosito/IMcoders/blob/devel/doc/images/imcoders_rviz.png)

Example video:

[![Rviz visualization boxes](https://img.youtube.com/vi/ohp5S3b75mg/0.jpg)](https://www.youtube.com/watch?v=ohp5S3b75mg)

## Troubleshooting
### Missing dependencies while building
If you are missing dependencies for any package in this repo, source the workspace and install the missing dependencies for the packages not building:
`source ~/imcoders_ws/devel/setup.bash`
`rosdep install <package_name>`  

Example:  
`rosdep install imcoders_control`  
