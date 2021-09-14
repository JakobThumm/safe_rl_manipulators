conda activate robot-rl
catkin_make clean
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Debug
source devel/setup.bash
rospack profile
