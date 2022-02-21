export ARGS="--headless"
docker run --gpus=all --privileged --net host -e "DISPLAY=$DISPLAY" -e ARGS -v /tmp/.X11-unix:/tmp/.X11-unix -v "$(pwd)"/../../../data_headless:/home/ubb/src/simple_pybullet_ros2/sim_recorder/data pybullet
docker run --gpus=all --privileged --net host -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v "$(pwd)"/../../../data:/home/ubb/src/simple_pybullet_ros2/sim_recorder/data pybullet

