# multinav_turtlebot3

 Multi-navigation system for Turtlebot3
 
## Introduction

This package provides basic operations and guides for multi-turtlebot3 navigation system, using Gazebo and RViZ.

## Requirement / Environment

- Ubuntu 16.04
- ros Melodic

-[Optional] Ros Development studio (for online computing, instead of local. visit: https://rds.theconstructsim.com/)

## Pre-install guide

Please install all the necessary packages for turtlebot3. 
For more information, follow exactly the guide from here: https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup

In case you don't want to hover over the link(only recommended for the pros), you can just simply follow these codes.
(Or using the RDS online)

`cd ~/catkin_ws/src/`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git`

`git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git`

`git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`cd ~/catkin_ws && catkin_make`

## Installation

The method of installation is simply doing catkin_make in your catkin_ws. Follow these codes.

`cd ~/catkin_ws/src/`

`git clone https://github.com/syahmi001/multinav_turtlebot3.git`

`cd ~/catkin_ws && catkin_make`

If no error found, congratulation! You have successfullt installed the package.

## Running the package

Copy and paste these lines in your respective terminal.

- Terminal 1 : Running ros node master

`roscore`

- Terminal 2 : Initiating and Running Gazebo environment

`export TURTLEBOT3_MODEL=burger`

`roslaunch multinav_turtlebot3 turtlebot3_world_multi.launch`

- Terminal 3 : Running multiple turtlebot3 navigation stacks

`export TURTLEBOT3_MODEL=burger`

`roslaunch multinav_turtlebot3 turtlebot3_navigation_multi.launch`

- Terminal 4 : Running simple script by sending different goals to different turtlebot3

(you might want to run "chmod +x simple_navigation_goals.py" in the src folder first to make sure your machine recognize the script.)

`cd ~/catkin_ws/src/multinav_turtlebot3/src`

`chmod +x simple_navigation_goals.py`

`export TURTLEBOT3_MODEL=burger`

`rosrun multinav_turtlebot3 simple_navigation_goals.py`

## Results

If everything runs smoothly, the expected output should be as below:

**In Gazebo**


![Scene1](../master/media/tt1.gif)




**In RViz**

![Scene1](../master/media/tt2.gif)

