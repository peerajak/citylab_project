# Checkpoint 5 ROS2 Basic C++

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