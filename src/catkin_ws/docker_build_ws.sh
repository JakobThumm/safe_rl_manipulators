# Execute all of these commands manually inside the docker container.
export PYTHONPATH="${PYTHONPATH}:$ROBOT_RL_SIM_ROOT/src/spinningup/"
sudo rm -r devel/ build/
conda activate robot-rl
pip3 install -r requirements.txt --upgrade
catkin_make clean
mkdir $ROBOT_RL_SIM_ROOT/src/modrob_workstation/workstation/build
cd $ROBOT_RL_SIM_ROOT/src/modrob_workstation/workstation/build
cmake ..
make
cd $ROBOT_RL_SIM_ROOT
catkin_make
source devel/setup.bash
rospack profile
