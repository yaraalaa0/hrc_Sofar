# human_baxter_collaboration

This project is composed of the design and implementation of a software architecture in ROS to control the double arm Baxter robot in cube-collecting scenario on Unity. The main objective of this project is to perceive the Unity simulation environment in real time and control Baxter to collect all blue cubes from their original locations to inside the blue box. In the meanwhile, Baxter must avoid collisions with the human operator as well as with the table and Baxter’s other arm. To perceive the locations of objects in the environment, baxter has access to the transformations of each object published by Unity. The movement of each arm is achieved by publishing a 4-trajectory message to Unity. The four trajectories in each message correspond to the following movements, respectively: go above cube’s location (the arm’s eef opens), go down to the cube’s location (the eef closes and picks the cube), go up, then, go to the target location (the eef releases the cube). The final objective is to move all blue cubes in the environment to the blue box in the fastest time possible and without collisions. 

-----------------------------------------------------------------------------------------

For launching the system:

- First, clone all the packages of this repository in the src folder of your ROS workspace.

- Then, in the root of your workspace, run:
~~~
source devel/setup.bash
~~~

- build the workspace:
~~~
catkin_make
~~~

## There are 4 ways to launch the system:

### 1- Fast cube collection using one arm at a time (Recommended)

The moveit controller uses normal planning to pose target for all the four trajectories (rarely, it leads to non-straight motions in the 2nd and 3rd trajectories). The task manager only uses one arm at a time to move the cubes.

- Start the ROS-Unity communication and the task manager node:

~~~
roslaunch human_baxter_collaboration human_baxter_one_hand.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers:

~~~
roslaunch human_baxter_servers servers.launch
~~~

### 2- Slow cube collection using one arm at a time

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

The moveit controller uses normal planning to pose target for all the four trajectories (rarely, it leads to non-straight motions in the 2nd and 3rd trajectories). The task manager only uses the two arms simultaneously to move the cubes. This leads to very fast behaviour. However, sometimes, the two arms collide with each othe in the middle placement. A form of arm collision avoidance needs to be developed.

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

The slow cube collection behaviour comes from using computeCartesian() function for planning the second and third trajectories. The task manager only uses the two arms simultaneously to move the cubes.

- Start the ROS-Unity communication and the task manager node:
~~~
roslaunch human_baxter_collaboration human_baxter_both_hands.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers: 
~~~
roslaunch human_baxter_servers servers_cartesian.launch
~~~
