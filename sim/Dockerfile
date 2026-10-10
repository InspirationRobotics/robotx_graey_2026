# Graey software-in-the-loop: ArduSub 4.5.7 SITL (the version on Graey's Cube)
# plus the ROS 2 Humble environment Graey's Jetson code runs in. The repo itself
# is mounted at run time (sim/run.sh), so code edits need no rebuild.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git g++ make python3-pip python3-dev python3-numpy python3-opencv \
      python3-lxml python3-serial python3-future libxml2-dev libxslt1-dev \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir "empy==3.3.4" pexpect pymavlink MAVProxy

RUN git clone --depth 1 --branch ArduSub-4.5.7 https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
 && cd /opt/ardupilot && git submodule update --init --recursive --depth 1
WORKDIR /opt/ardupilot
RUN ./waf configure --board sitl && ./waf sub

WORKDIR /root/robotx_ws/src/robotx_graey_2026
CMD ["bash"]
