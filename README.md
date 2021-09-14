# Robot RL

This project covers everything to train verifiably safe manipulators in human environments using reinforcement learning (RL).
The simulation environment is based on Gazebo.
In the future we are planning to also incorporate real-world training.


## Installation
We work on Ubuntu 20.04.
If you run Ubuntu 20.04, you can install our project locally with the following steps.
Otherwise, you can use a Docker container as described below.
A Nvidia graphics card is required if you want to display the simulation GUI.

1. Install ROS noetic http://wiki.ros.org/noetic/Installation/Ubuntu
2. Go into catkin workspace root folder `cd src/catkin_ws`
3. Copy your PATH_TO_CATKIN_WS:
    ```
    pwd
    ```
4. Add to your `~/.bashrc`:
    ```
    source /opt/ros/noetic/setup.bash
    export ROBOT_RL_SIM_ROOT=PATH_TO_CATKIN_WS
    alias kill_gazebo='$ROBOT_RL_SIM_ROOT/kill_all_processes.sh'
    ```
    Make sure to replace PATH_TO_CATKIN_WS with the copied path.
    The `kill_gazebo` command can be used if not all ros and gazebo tasks are closed.

5. Install anaconda https://docs.anaconda.com/anaconda/install/linux/) and create a conda environment with Py 3.6
    ```
    conda create -n robot-rl python=3.6
    conda activate robot-rl
    ```
6. Go into catkin workspace root folder `cd src/catkin_ws`
7. Install ros dependencies.
   Do this now and when checking out a new branch or pulling big changes:
    ```
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
8. Install pip requirenments
    ```
    pip3 install -r requirements.txt --upgrade
    ```
9. (Optional): Install spinup.
    ```
    cd src/spinningup/
    sudo apt-get update && sudo apt-get install libopenmpi-dev
    pip install -e .
    ```
    Unfortunately, the spinningup repo has a bug regarding tf1 and pytorch. There is already a ticket on their github and I commented a simple solution (https://github.com/openai/spinningup/issues/333).
10. Compile the modrob workstation library
    To compile the workstation library, execute the following steps:
    1. create a build directory in the workstation folder and cd into it
    ```
    mkdir $ROBOT_RL_SIM_ROOT/src/modrob_workstation/workstation/build
    cd $ROBOT_RL_SIM_ROOT/src/modrob_workstation/workstation/build
    ```
    2. generate a make file with **cmake**
    ```
    cmake ..
    ```
    3. compile the library with the generated **make file**
    ```
    make
    ```
11. Build project
    In catkin workspace root folder:
    Either
    ```
    ./debug_build.sh
    ```
    for debug and development, or
    ```
    ./clean_build.sh
    ```
    for deployment and training on server.
    IMPORTANT:
    ```
    source devel/setup.bash
    ```
    This needs to be repeated for every new bash.

## Modular robot simulation
Run the training
```
roslaunch example_modrob_human_training start_training_safe_human_her_sac_modrob1.launch
```

## Important configuration files
Set the parameters of the RL algorithm in
```
src/catkin_ws/src/example_modrob_human_training/config/my_modrob_safe_human_params_sac_her_modrob1.yaml
```
Set the parameters of the robot task (like goal position, accuracy, time in between movements, etc.) in
```
src/catkin_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs/human_modrob/config/modrob_safe_human_random_modrob1.yaml
```
You can use the recommended parameters for the random goal experiment in
```
src/catkin_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs/human_modrob/config/modrob_safe_human_random_modrob1_all_rand.yaml
```
or for the evading human experiment in
```
src/catkin_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs/human_modrob/config/modrob_safe_human_random_modrob1_single_goal.yaml
```

To disable the safety shield, go to the file
```
src/catkin_ws/src/openai_ros/openai_ros/src/openai_ros/robot_envs/modrob_env_path_following.py
```
and change the line 110 from
```
launch_file_name="init_modrob.launch",
```
to
```
launch_file_name="init_modrob_unsafe.launch",
```

The results will be saved in
```
src/catkin_ws/src/example_modrob_human_training/training_results
```

## Test pretrained models
We provide the training results of the safe and unsafe agent (baseline) for two
experiments: random goal and evading human.
These experiments are located in
```
src/catkin_ws/src/example_modrob_human_training/training_results
```

To test these models, you will need to adapt the file
```
src/catkin_ws/src/example_modrob_human_training/config/my_modrob_safe_human_params_sac_her_modrob1_load.yaml
```
Choose the correct `outdir` and choose the `load_epoch` (195 for fully trained model).

## Docker setup


### Without GUI (for training on server)
#### Install docker
Install docker according to https://docs.docker.com/engine/install/ubuntu/
#### Build the docker image
Build the docker container (You only need to rebuild if you change something in the package.xml files or in the spinningup folder.)
```
cd src/catkin_ws
./build_docker.sh
```
#### Run the docker image without graphical output
`./run_docker_no_gui.sh`
#### Build the ros project (see below)

### With GUI (for local machine)
Since the simulation needs the graphical representation of gazebo,
we need to run a nvidia-docker2 container.
A nvidia graphics card with CUDA support is required.

#### Install nvidia cuda toolkit
`sudo apt-get install nvidia-cuda-toolkit`
This is not the deep learning support. Check cuDNN for this.
#### Install nvidia docker
```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd
```
### Build the docker image
Build the docker container (You only need to rebuild if you change something in the package.xml files or in the spinningup folder.)
```
cd src/catkin_ws
./build_docker.sh
```
#### Run the docker image with graphic output
`./run_docker_gui.sh`


### Build the ROS project
Execute every command in `./docker_build_ws.sh` manually:
```
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
```
