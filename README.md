# human_baxter_collaboration



For launching the system:

First, clone all the packages of this repository in the src folder of your ROS workspace.

Then, in the root of your workspace, run:
~~~
source devel/setup.bash
~~~

Then,
~~~
catkin_make
~~~

There are 4 ways to launch system:

## Fast cube collection using one arm at a time (Recommended)

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

## Slow cube collection using one arm at a time

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

## Fast cube collection using two arms simultaneously (needs improvement)

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

## Slow cube collection using two arms simultaneously (needs improvement)

The slow cube collection behaviour comes from using computeCartesian() function for planning the second and third trajectories. The task manager only uses the two arms simultaneously to move the cubes.

- Start the ROS-Unity communication and the task manager node:
- 
~~~
roslaunch human_baxter_collaboration human_baxter_both_hands.launch
~~~

- Start the simulation of Unity

- On another terminal, launch the servers:
- 
~~~
roslaunch human_baxter_servers servers_cartesian.launch
~~~
