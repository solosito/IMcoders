# IMcoders

## Dependencies
### ROS
`sudo apt install ros-kinetic-full-desktop`

### Gazebo (just for simulation)
`sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-control`

## How-to
1. Create a new ROS workspace or use your an existing one
`mkdir -p ~/imcoders_ws/src`

1. Clone this repo:
`git clone https://github.com/solosito/IMcoders.git ~/imcoders_ws/src/.`

1. Build the code
`cd ~/imcoders_ws`

then:

`catkin build`

or

`catkin_make`

1. Launch simulation
`roslaunch imcoders_control keyboard_teleop.launch`

1. Launch teleop
`roslaunch imcoders_gazebo box_robot_gazebo.launch`

1. Launch Rviz (optional)
`rviz`

## Troubleshooting
### Missing dependencies
If you are missing dependencies for the in this repo, from the workspace try:
`rosdep install <package_name>`
e.g.,
`rosdep install <imcoders_control>`
