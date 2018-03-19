# Full ROS installation on Ubuntu 16.04
FROM ros:lunar-perception-xenial

MAINTAINER Lukas Lao Beyer "llb@mit.edu"

# Nvidia
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    ssh \
    vim \
    nano \
    python-pip \
    clang-format-3.8

RUN apt-get update && apt-get install -y \
    libblas-dev liblapack-dev \
    ros-lunar-robot-model \
    ros-lunar-rqt \
    ros-lunar-rqt-common-plugins \
    ros-lunar-rqt-robot-plugins \
    #ros-lunar-moveit \
    #ros-lunar-moveit-kinematics \
    #ros-lunar-moveit-visual-tools \
    ros-lunar-robot-state-publisher \
    ros-lunar-joint-state-publisher \
    ros-lunar-controller-manager \
    ros-lunar-trac-ik \
    ros-lunar-trac-ik-kinematics-plugin \
    python-catkin-tools \
    python-wstool

RUN pip install numpy dxfgrabber

# Init MoveIt workspace (for installing from source)
RUN mkdir -p /ws_moveit/src && \
    cd /ws_moveit/src && \
    wstool init . 

# Add custom MoveIt wstool config
ADD config/moveit.rosinstall /ws_moveit/moveit.rosinstall

# Install MoveIt
RUN cd /ws_moveit/src && \
    wstool merge /ws_moveit/moveit.rosinstall && \
    wstool update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro lunar && \
    cd .. && \
    catkin config --extend /opt/ros/lunar --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build

# Init main workspace
RUN mkdir -p /ws/src

# Install OpenRAVE
# RUN cd /root && \
#     git clone --branch latest_stable https://github.com/rdiankov/openrave.git && \
#     cd openrave && mkdir build && cd build && \
#     cmake .. && \
#     make -j4 && \
#     make install

# Install Descartes
# RUN cd /ws/src && \
#     source /ws_moveit/devel/setup.bash && \
#     git clone https://github.com/ros-industrial-consortium/descartes.git && \
#     cd /ws && \
#     rosdep install -r -y --from-paths src --ignore-src

# Build ROS workspace (for Descartes only)
# RUN cd /ws && \
#     source /ros_entrypoint.sh && \
#     source /ws_moveit/devel/setup.bash && \
#     catkin build

# Catkin prebuild (for speed when rebuilding)
RUN cd /ws && \
    source /ros_entrypoint.sh && \
    source /ws_moveit/devel/setup.bash && \
    catkin build

# ROS packages
ADD src /ws/src

# Launch scripts, etc.
ADD scripts/* /ws/

# RViz config
ADD config/rviz/* /root/.rviz/

# Add examle drawing
ADD config/drawing.dxf /root/drawing.dxf

# IKFast generated file
ADD config/ikfast61_redhawk.cpp /root/ikfast61_redhawk.cpp

# .bashrc
ADD config/.bashrc /root/.bashrc

# Build ROS workspace
RUN cd /ws && \
    source /ros_entrypoint.sh && \
    source /ws_moveit/devel/setup.bash && \
    catkin build

# Build IKFast MoveIt! package
# RUN cd /ws && \
#     source /ros_entrypoint.sh && \
#     source devel/setup.bash && \
#     cd src && \
#     catkin_create_pkg redhawk_ikfast && \
#     cd /ws && \
#     catkin build && \
#     source devel/setup.bash && \ 
#     rosrun moveit_kinematics create_ikfast_moveit_plugin.py redhawk robot redhawk_ikfast /root/ikfast61_redhawk.cpp && \
#     cd /ws && \
#     catkin build


ENTRYPOINT /ws/launch.bash
