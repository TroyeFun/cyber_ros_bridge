FROM osrf/ros:melodic-desktop-full 


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

    
RUN ["/bin/bash", "-c", "apt-get update && apt-get install software-properties-common -y"]
RUN ["/bin/bash", "-c", "add-apt-repository ppa:deadsnakes/ppa && apt install python3.7 -y && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1"]
RUN ["/bin/bash", "-c", "apt-get update && apt-get install -y apt-utils"]
RUN ["/bin/bash", "-c", "apt-get update && apt-get install python3-pip python3-yaml -y"]
RUN ["/bin/bash", "-c", "pip3 install --upgrade pip setuptools"]
RUN ["/bin/bash", "-c", "pip3 install rospkg catkin_pkg pycyber numpy"]
RUN ["/bin/bash", "-c", "apt-get install python-catkin-tools python3.7-dev -y "]
RUN ["/bin/bash", "-c", "pip3 install ipdb protobuf==3.20.1"]
RUN ["/bin/bash", "-c", "pip3 install cython"]
# pip3 install protobuf==3.20.1






