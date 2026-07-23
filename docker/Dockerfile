FROM ultralytics/ultralytics:latest-jetson-jetpack6
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
      curl gnupg lsb-release software-properties-common locales \
      usbutils iproute2 net-tools nano v4l-utils udev \
    && locale-gen en_US en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y \
      ros-humble-ros-base ros-humble-cv-bridge ros-humble-vision-opencv \
      ros-humble-image-transport ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
      python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
      "depthai<3" pymavlink MAVProxy pyserial future pyyaml

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/robotx_ws
CMD ["bash"]
