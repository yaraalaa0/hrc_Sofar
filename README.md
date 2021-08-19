# human_baxter_collaboration

This project is composed of the design and implementation of a software architecture in ROS to control the double arm Baxter robot in cube-collecting game scenario on Unity. The main objective is to perceive the Unity simulation environment in real time and control Baxter to collect all blue cubes from their original locations to inside the blue box. In the meanwhile, Baxter must avoid collisions with the human operator as well as with the table and Baxter’s other arm. 

To perceive the locations of objects in the environment, baxter has access to the transformations of each object published by Unity. The movement of each arm is achieved by publishing a 4-trajectory message to Unity. The four trajectories in each message correspond to the following movements, respectively: 
- go above cube’s location (the arm’s eef opens)
- go down to the cube’s location (the eef closes and picks the cube)
- go up
- go to the target location (the eef releases the cube). 

The final objective is to move all blue cubes in the environment to the blue box in the fastest time possible and without collisions. 

----------------------------------------------------------------------------------------

The overall system architecture is described in the UML diagram:

![alt text](https://github.com/yaraalaa0/hrc_sofar/blob/main/HRC8_UML_FINAL.png?raw=true)

-----------------------------------------------------------------------------------------

Installing the dependencies:

- Download the package baxter_common from: https://github.com/RethinkRobotics/baxter_common and extract it in the src folder of your ROS workspace.
- Download the package moveit_robots from: https://github.com/ros-planning/moveit_robots and extract it in the src folder of your ROS workspace.

For launching the system:

- First, clone all the packages of this repository in the src folder of your ROS workspace.

- Then, in the root of your workspace, run:
~~~
source devel/setup.bash
~~~

- Then, build the workspace:
~~~
catkin_make
~~~

## There are 4 ways to launch the system:

### 1- Fast cube collection using one arm at a time (Recommended for Simulation)

The moveit controller uses normal planning to pose targets for all the four trajectories. In some rare cases, it leads to non-straight motions of the eef in the 2nd and 3rd trajectories. The task manager only uses one arm at a time to move the cubes.

- Start the ROS-Unity communication and the task manager node:

~~~
roslaunch human_baxter_collaboration human_baxter_one_hand.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers:

~~~
roslaunch human_baxter_servers servers.launch
~~~

### 2- Slow cube collection using one arm at a time (Recommended for Hardware)

The slow cube collection behaviour comes from using computeCartesian() function for planning the second and third trajectories. This is done to force the baxter's eef to go in straight line while going down to pick the cube and going up after picking it. 

- Start the ROS-Unity communication and the task manager node:

~~~
roslaunch human_baxter_collaboration human_baxter_one_hand.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers:

~~~
roslaunch human_baxter_servers servers_cartesian.launch
~~~

### 3- Fast cube collection using two arms simultaneously (needs improvement)

The task manager uses the two arms simultaneously to move the cubes. This leads to very fast behaviour. However, sometimes, the two arms collide with each other in the middle placement. A form of arm collision avoidance needs to be developed.

- Start the ROS-Unity communication and the task manager node:

~~~
roslaunch human_baxter_collaboration human_baxter_both_hands.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers:

~~~
roslaunch human_baxter_servers servers.launch
~~~

### 4- Slow cube collection using two arms simultaneously (needs improvement)

- Start the ROS-Unity communication and the task manager node:
~~~
roslaunch human_baxter_collaboration human_baxter_both_hands.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers: 
~~~
roslaunch human_baxter_servers servers_cartesian.launch
~~~
