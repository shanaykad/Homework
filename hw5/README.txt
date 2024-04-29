Problem 1:
See the mp4 files titled savingmap and localizationdemo

Bonus: SLAM standards for Simultaneous Localization and Mapping. The idea behind the process is if I drop a robot into an environment it knows nothing about, like a residential home, it should be able to map its environment while also determining where it is in that environment. To do this it must utilize two kinds of sensors. The first is some sort of environmental scanning to get information on the robots surroundings. This can be in the form of visual data, lidar, etc. The robot also must be able to get odometry data aka information about how it is moving. This is often done through the use of encoders on the wheels to track the rotational velocity data which can then be used to approximate the relative position of the robot as it moves through space. The general process to build the map and relative location is you iteratively move the robot a known distance and angle through the space taking environmental readings periodically. By doing this and knowing the relative distance between each environmental scan we can build up the entire environment by stitching the data together where it overlaps. Additionally, because we were constantly collecting data about the position of the robot we can determine exactly where the robot is in this stitched map. 
Unfortunately though the environmental data rarely overlaps due to a mixture of noisy lidar and noisy encoders. To fix this we introduce constraints between the relative positions of each scan with a graph. In this graph nodes are represented by the approximated locations of the bot from the odometry and the edges are constraints we apply between the positions. These edges are weighted based upon the confidence we have in the relative position between the two nodes. For example, with perfect odometry sensors our confidence would be absolute (1) but with really bad odometry sensors it would be ignorable (0). The trick to this way of solving comes when the robot makes two scans it believes are the same. By doing this we can create loops of constraints which “tug” on each other to optimize how far we deviate from each constraint. Because we deviate from the calculated relative positions the relative positions of the scans are also shifted and with enough scans and accuracy the scans will begin to overlap creating a map of the surrounding area.

Problem 2:
Navigate to ENAE450_ws/src/hw5/hw5 and run
chmod u+x insidewalls

Navigate back to ENAE450_ws and run
colcon build --symlink install

Then run
source install/setup.bash

Run in a terminal
ros2 launch hw5 tb3_4walls_inside.launch.py

Bonus: Both parameters have been made can be set from within the launch file, the direction parameter can be set to 1 for CCW and -1 for CW.

Problem 3:
Give permissions and rebuild/source if necessary then run
ros2 launch hw5 tb3_4walls_outside.launch.py



Contributions: Note that only Shanay had working gazebo
1: Shanay/Trevor
video files

1 bonus: Trevor

2:
Functional test of turtlebot3 tracking wall code - Trevor
Initial forward, turning, and scanning code in Gazebo - Shanay
Logic to self correct, change state, scan and average distance from wall in Gazebo - Trevor and Shanay

2 bonus: Shanay

3: Shanay