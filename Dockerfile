FROM ros:humble-ros-base

ARG USERNAME=cvdoc
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

RUN rm -f /etc/apt/sources.list && curl -sSL http://mirrors.pku.edu.cn/repoconfig/ubuntu22.04/sources.list -o /etc/apt/sources.list
RUN curl -sSL https://ghproxy.cn/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim unzip -y && \
    pip install xmacro gdown

# Install small_gicp
RUN apt install -y libeigen3-dev libomp-dev && \
    mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

RUN echo 'export PATH=$PATH:/home/ws/.script' >> /home/$USERNAME/.bashrc
RUN echo 'alias wsi="source /opt/ros/humble/setup.bash"' >> /home/$USERNAME/.bashrc
RUN echo 'alias ini="source install/setup.bash"' >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml && rosdep update || true

RUN --mount=type=bind,target=/home/ws,source=.,readonly=false cd /home/ws \
    && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y || true

RUN --mount=type=bind,target=/home/ws,source=.,readonly=false cd /home/ws \
    && sudo gdown https://drive.google.com/uc\?id\=1kAxdOU-mi1TcLssyKmDR0pz0ojF96jQ4 -O /tmp/simulation_pcd.zip && \
    sudo unzip /tmp/simulation_pcd.zip -d /home/ws/src/pb2025_nav_bringup/pcd/simulation/ && \
    sudo rm /tmp/simulation_pcd.zip


# setup .zshrc
# RUN echo 'export TERM=xterm-256color\n\
#     source ~/ros_ws/install/setup.zsh\n\
#     eval "$(register-python-argcomplete3 ros2)"\n\
#     eval "$(register-python-argcomplete3 colcon)"\n'\
#     >> /root/.zshrc

# source entrypoint setup
# RUN sed --in-place --expression \
#       '$isource "/root/ros_ws/install/setup.bash"' \
#       /ros_entrypoint.sh

# RUN rm -rf /var/lib/apt/lists/*
