# Ridgeback + UR5 simulation

Simple repo merging ridgeback packages with ur_description to control the ridgeback using Rviz.

Use either in sim or with real robot, must be able to communicate with robot before launching rviz.
## docker

    # Build the Docker containers
    docker compose build

    # Start the Docker containers in detached mode
    docker compose up -d

    # Access the Docker container's shell
    docker exec -it ros_docker-ros-1 bash


## Commands 

### Gazebo sim

Office simulation: 
```console
roslaunch cpr_office_gazebo office_world.launch platform:=ridgeback
```

Racetrack simulation: 
```console
roslaunch ridgeback_gazebo ridgeback_world.launch
```

### RViz 

Basic Rviz: 
```console
roslaunch ridgeback_viz view_robot.launch
```
To use interactive markers, use this topic relay: 
```console
rosrun topic_tools relay /ridgeback_velocity_controller/cmd_vel /cmd_vel
```

Can also use keyboard for basic control: 
```console
rosrun ridgeback_control teleop_keyboard.py 
```
#### Navigation and mapping 
More info in this [tutorial](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/indoor_robots/ridgeback/tutorials_ridgeback/#navigating-ridgeback)

##### Navigation
```console
roslaunch ridgeback_navigation odom_navigation_demo.launch
roslaunch ridgeback_viz view_robot.launch config:=navigation
```
##### Mapping
```console
roslaunch ridgeback_navigation gmapping_demo.launch
roslaunch ridgeback_viz view_robot.launch config:=gmapping
```
### Setup 

in ridgeback_ur5_sim run :
```console
rosdep install -y --from-paths . --ignore-src --rosdistro noetic 
catkin_make
source devel/setup.bash
```
### TODO 

Make map of robot room and save in here, use in sim !

