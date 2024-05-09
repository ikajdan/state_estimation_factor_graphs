FROM ros:humble

ENV DEBIAN_FRONTEND noninteractive

RUN ln -snf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    echo Europe/Warsaw > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata && \
    sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' /root/.bashrc && \
    echo '! shopt -oq posix && source /etc/bash_completion' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=222' >> /root/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'export PS1="🐢 $PS1"' >> /root/.bashrc && \
    mkdir -p /root/ws/src

COPY ./src/ /root/ws/src/

WORKDIR /root/ws

RUN apt-get update -q && \
    apt-get install -y -q --no-install-recommends \
        bash-completion \
        command-not-found \
        curl \
        iputils-ping \
        net-tools \
        tzdata \
        unzip \
        software-properties-common \
        build-essential \
        cmake \
        python3 \
        python3-pip \
        wget \
        ros-humble-rviz2 \
        ros-humble-rviz-imu-plugin \
        ros-humble-joint-state-publisher-gui \
        ros-humble-teleop-twist-keyboard && \
    rosdep update --rosdistro=humble -q && \
    rosdep install --from-paths src --ignore-src -y -q && \
    rm -rf /root/.ros/ && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
