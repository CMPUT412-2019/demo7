FROM osrf/ros:kinetic-desktop-full

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Install a few utilities
RUN apt-get update
RUN apt-get install -y wget tmux
RUN mkdir /nvim
WORKDIR /nvim
RUN wget https://github.com/neovim/neovim/releases/download/v0.4.3/nvim.appimage
RUN chmod u+x nvim.appimage
RUN ./nvim.appimage --appimage-extract
RUN echo "PATH=$PATH:/nvim/squashfs-root/usr/bin" >> ~/.bashrc

# Install prereqs for realsense
# RUN echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' || sudo tee /etc/apt/sources.list.d/realsense-public.list
# RUN sh -c 'echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" > /etc/apt/sources.list.d/realsense-public.list'
# RUN apt-get update -qq
# RUN apt-get install librealsense2-dkms --allow-unauthenticated -y
# RUN apt-get install librealsense2-dev --allow-unauthenticated -y
# RUN apt-get install ros-kinetic-cv-bridge -y
# RUN apt-get install ros-kinetic-image-transport
# RUN apt-get install ros-kinetic-tf -y
# RUN apt-get install ros-kinetic-diagnostic-updater -y
# RUN apt-get install ros-kinetic-ddynamic-reconfigure -y

# # And install realsense from source.
# RUN mkdir -p /ros/src
# WORKDIR /ros/src
# RUN git clone -v --progress https://github.com/doronhi/realsense.git
# RUN source /opt/ros/kinetic/setup.bash && catkin_init_workspace
# WORKDIR /ros
# RUN source /opt/ros/kinetic/setup.bash && catkin_make clean
# RUN source /opt/ros/kinetic/setup.bash && catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
# RUN source /opt/ros/kinetic/setup.bash && catkin_make install

# Install a dummy package for "realsense-camera" so that apt doesn't try to install it again.
RUN apt-get install -y equivs
WORKDIR /
RUN echo $'Section: misc \n\
Priority: optional \n\
Standards-Version: 2.3.3 \n\
Package: ros-kinetic-realsense-camera \n\
Version: 1.8.1 \n\
Section: mail \n\
Maintainer: Mitchell Epp \n\
Provides: mail-transport-agent \n\
Architecture: all \n\
Description: Dummy realsense-camera package' >> ros-kinetic-realsense-camera
RUN equivs-build ros-kinetic-realsense-camera
RUN dpkg -i ros-kinetic-realsense-camera_1.8.1_all.deb

# Once this is done, the turtlebot packages won't throw errors due to librealsense. So, install them.
RUN apt-get install -y ros-kinetic-turtlebot
RUN apt-get install -y ros-kinetic-turtlebot-apps
RUN apt-get install -y ros-kinetic-turtlebot-interactions
RUN apt-get install -y ros-kinetic-turtlebot-simulator
RUN apt-get install -y ros-kinetic-kobuki
RUN apt-get install -y ros-kinetic-kobuki-core
RUN apt-get install -y ros-kinetic-openni2-camera
RUN apt-get install -y ros-kinetic-openni2-launch

WORKDIR /ros