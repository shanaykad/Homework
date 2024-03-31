Problem 1:
Navigate to ENAE450_ws/src/hw4_1/hw4_1 and run
chmod u+x singleturtle.py

Navigate back to ENAE450_ws and run
colcon build --symlink install

Then run
source install/setup.bash

Start tmux with at least two panes and run the following commands in seperate windows
ros2 run turtlesim turtlesim_node
ros2 run hw4_1 singleturtle



Problem 2:
Navigate to ENAE450_ws/src/hw4_2/hw4_2 and run
chmod u+x trajectory

Navigate back to ENAE450_ws and run
colcon build --symlink install

Then run
source install/setup.bash

Start tmux with at least two panes and run the following commands in seperate windows
ros2 run turtlesim turtlesim_node
ros2 run hw4_2 trajectory

PLEASE NOTE: For some reason when running trajectory it will spam the terminal with a coordinate out of range error, 
I don't know why this happens, and its not consistent with anything but if it does please redo the last step until it works.

Bonus: I made k and w parameters, they can be set either in trajectory.py in lines 48 and 49 or with these commands respectively (1.0 can be replaced with any float)
ros2 param set /draw_function k 1.0
ros2 param set /draw_function w 1.0



Problem 3:
Navigate to ENAE450_ws and run
colcon build --symlink install

Then run
source install/setup.bash

Then run
ros2 launch hw4_3 turtlefollowinglaunch.py

Bonus: I added the respawn when touching walls feature, to activate it use the command below, setting the parameter to anything but 0 will activate it
ros2 param set /turtle_control avoidwalls 1