# Checkpoint 6. ROS2 Basic C++

## Task 1

In this section, you will create a ROS Service that tells the robot what to do. This will replace the obstace avoidance algorithm you created in Checkpoint 5.

The goal in this section is to create a ROS Service that, when called:

- It receives the laser data captured by the sensor
- It analyzes the laser data received
Based on this data, it tells the robot what to do next
To accomplish this, you will have to do 3 things:

Create a service server node that, when called, will analyze the laser data and tell the robot where to move next
Modify the program from Checkpoint 5 (patrol.cpp) so that it includes now a service client that calls the service server before sending velocities to the robot
Create a new launch file to launch both nodes: the service server node and the patrol node.

## Task 2

Create an action server to send position goals to the robot¶
The goal of this section is to create an action server that allows you to send the robot to a certain position, so we can later use it to manually navigate the the robot around the environment.

To achieve that, you will need to create another node that contains an action server in charge of doing the necessary operations.

/odom

x: x coordinate (in meters)
y: y coordinate (in meters)
theta: goal orientation (in degrees)



# Checkpoint 5 ROS2 Basic C++
## Current Status
Done 2 real robot sessions.
Using broadest band policy provides the best obstracle avoding for robot in simulation, fully successful.
Real robot, however, move quite different than the simulation, and thus I can see it tried to avoid the obstracle,
but yet to be fully successful.

## Implement the following algorithm inside the laser callback, to identify the safest direction to move next:

## You need to get the rays corresponding to those 180º
- Identify the largest distance ray (which is not an inf) of those 180º, and get its corresponding angle from the front X axis
  Note: that angle must be between -pi/2
 and pi/2
That angle indicates the direction to which the robot must rotate in order to move to the most opened (safest) area.
Store the angle into a class variable named direction_

## Move the robot to the safest area
In order to move the robot to the safe direction, you need to publish to the velocity topic of the robot the proper values:

Inside the patrol.cpp, create a publisher to the /cmd_vel topic (the one that controls the wheels)
- Create a control loop of 10 Hz
- At every step of the control loop, you need to publish the proper velocity command on that topic, based on the values detected by the laser.
- The linear velocity in x must always be 0.1 m/s
- Take the value of the var direction_ and use it to compute the proper angular velocity in z:
- angular velocity in z = direction_ / 2
- Create a launch file named start_patrolling.launch.py that starts your program.