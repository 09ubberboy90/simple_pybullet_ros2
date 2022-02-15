ARG VARIANT="focal"
ARG BASE_IMAGE=nvidia/cudagl:11.0-devel-ubuntu20.04

FROM ${BASE_IMAGE} AS base

ENV USER_NAME ubb
ENV USER_PASSWORD docker
ENV DEBIAN_FRONTEND=noninteractive
ENV ARGS " "

RUN apt-get update && \
  apt-get install -y sudo \
  software-properties-common \
  curl \
  git-core \
  gnupg \
  zsh \
  wget \
  nano \
  xvfb \
  fonts-powerline \
  gnupg2 \
  lsb-release \
  # add a user (--disabled-password: the user won't be able to use the account until the password is set)
  && adduser --quiet --disabled-password --shell /bin/zsh --home /home/$USER_NAME --gecos "User" $USER_NAME \
  # update the password
  && echo "${USER_NAME}:${USER_PASSWORD}" | chpasswd && usermod -aG sudo $USER_NAME

FROM base AS ros2

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


RUN apt-get update
RUN apt-get install -y ros-foxy-desktop \
  ros-foxy-moveit \
  ros-foxy-moveit-common \
  ros-foxy-gripper-controllers \
  ros-foxy-ros2-controllers \
  ros-foxy-ros2-control 

RUN apt-get install -y python3-rosdep python3-pip python3-colcon-common-extensions python3-colcon-mixin 
RUN pip3 install -U pytest pyquaternion

FROM ros2 AS pybullet

RUN pip3 install -U pybullet

FROM pybullet AS sim_comp

RUN sudo rosdep init
RUN rosdep update

RUN cd /home/${USER_NAME}/ \
    && mkdir -p /home/${USER_NAME}/src \
    && chown ${USER_NAME}:${USER_NAME} -R /home/${USER_NAME} \
    && cd /home/${USER_NAME}/src \
    && git clone https://github.com/09ubberboy90/simple_pybullet_ros2.git \
    && rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

RUN cd /home/${USER_NAME}\
    && . /opt/ros/foxy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && chown ${USER_NAME}:${USER_NAME} -R /home/${USER_NAME}

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh 
ENTRYPOINT ["/entrypoint.sh"]

CMD ["sh", "-c", "ros2 run sim_recorder run_recording -i 20 ${ARGS}"] 
