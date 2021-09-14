# Currently, pip install requirements inside the docker only works if you are
# running the docker container as root.
# Consider further tuning of the Docker build process to allow non-root users
# with: --user=1000 \
docker run -it \
    --gpus=all \
    --volume="$ROBOT_RL_SIM_ROOT:/home/$USER/catkin_ws" \
    --net=host \
    --privileged \
    ros-noetic-spinningup/$USER:v1 \
    bash
