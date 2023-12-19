# Ridgeback + UR5 simulation

Simple repo merging ridgeback packages with ur_description to control the ridgeback using Rviz.

Use either in sim or with real robot, must be able to communicate with robot before launching rviz.

## Commands 

### Gazebo sim

roslaunch cpr_office_gazebo office_world.launch platform:=ridgeback


### RViz 

roslaunch ridgeback_viz view_robot.launch


### Setup 

in ridgeback_ur5_sim run :

rosdep install -y --from-paths . --ignore-src --rosdistro noetic 
catkin_make
source devel/setup.bash
