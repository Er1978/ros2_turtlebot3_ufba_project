rm -rf log install build
colcon build --packages-select turtlebot3_ufba
source install/setup.bash

