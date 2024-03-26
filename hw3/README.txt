Problem 1: (Bonus iii was done)
Navigate to ENAE450_ws/src/hw3_1/hw3_1 and run 
chmod u+x sortingpub.py sortingsub.py

Navigate back to ENAE450_ws and run
colcon build --symlink-install

Then run
source install/setup.bash

Start tmux with at least two windows and to run the publisher and subscriber respectively, run
ros2 run hw3_1 py_sortingpub
ros2 run hw3_1 py_sortingsub

Problem 2: 
Navigate to ENAE450_ws/src/hw3_2/hw3_2 and run 
chmod u+x numsearchpub.py numsearchsub.py

Navigate back to ENAE450_ws and run
colcon build --symlink-install

Then run
source install/setup.bash

Start tmux with at least two windows and to run the publisher and subscriber respectively, run
ros2 run hw3_2 py_numsearchpub
ros2 run hw3_2 py_numsearchsub