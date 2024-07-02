FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# update the system
RUN apt-get update && apt-get upgrade -y

# install python and pip
RUN apt-get install -y python3 python3-pip iproute2

# install python packages
RUN pip3 install numpy python-can

# install can-utils
RUN apt-get install -y can-utils git
# enable the module
# RUN modprobe can
# enable the interface
# RUN ip link set can0 type can bitrate 1000000
# bring up the interface
# RUN ip link set up can0

RUN mkdir -p /ros2_ws/src

RUN chmod 1777 /tmp

# install lart_msgs
WORKDIR /ros2_ws/src
RUN echo "hello"
RUN git clone -b dev https://github.com/FSLART/lart_msgs.git
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 6"

# copy the source code
COPY . /ros2_ws/src/t24e_can_bridge

# build the workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 6"

# start the node
CMD ["/bin/bash"]
