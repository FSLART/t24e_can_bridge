FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# update the system
RUN apt-get update && apt-get upgrade 

# install python and pip
RUN apt-get install -y python3 python3-pip

# install python packages
RUN pip3 install numpy python-can

# install can-utils
RUN apt-get install -y can-utils
# enable the module
RUN modprobe can
# enable the interface
RUN ip link set can0 type can bitrate 1000000
# bring up the interface
RUN ip link set up can0


