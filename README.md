# IMcoders

## Dependencies
### ROS Kinetic
Link to official site:  
http://wiki.ros.org/kinetic/Installation/Ubuntu  
**NOTE:** The recomended version is _Desktop-Full Install_

### ROS IMU tools package
`sudo apt install ros-kinetic-imu-tools`  

### (Optional) RTIMULib2
Install this package if you want to use the `imcoder_reader` 
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
`git clone https://github.com/solosito/IMcoders.git ~/imcoders_ws/src/.`

1. Build the code  
Go to the workspace folder  
`cd ~/imcoders_ws`  
then  
`catkin_make`  
or  
`catkin build`  

1. Source the workspace  
`source ~/imcoders_ws/devel/setup.bash`  
(Optional) Source the workspace in your .bashrc so you don't have to source the workspace every time you open a new terminal  
`echo "source ~/imcoders_ws/devel/setup.bash" >> ~/.bashrc`

1. Launch simulation  
`roslaunch imcoders_gazebo box_robot_gazebo.launch`  

1. Launch teleop from a new terminal (node you need to source the workspace as before if you did not modified your .bashrc)  
`roslaunch imcoders_control keyboard_teleop.launch`

1. (Optional) Launch Rviz for visualization  
`roslaunch imcoders_rviz_launchers view_diff.launch`  
`roslaunch imcoders_rviz_launchers view_box.launch`  

## Troubleshooting
### Missing dependencies while building
If you are missing dependencies for any package in this repo, source the workspace and install the missing dependencies for the packages not building:
`source ~/imcoders_ws/devel/setup.bash`
`rosdep install <package_name>`  

Example:  
`rosdep install imcoders_control`  
