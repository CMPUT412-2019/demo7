FROM cmput412_ros

RUN apt-get install -y ros-kinetic-rtabmap-ros ros-kinetic-ar-track-alvar ros-kinetic-ros-numpy

WORKDIR /ros/src
RUN ln -s /source/src/calibration
RUN ln -s /source/src/models
RUN ln -s /source/src/navigation
RUN ln -s /source/src/demo7

WORKDIR /source

COPY build.bash /tmp/build.bash
RUN source /opt/ros/kinetic/setup.bash && bash /tmp/build.bash
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "source /ros/devel/setup.bash" >> ~/.bashrc

# CMD ["bash", "-c", "./build.bash"]

CMD ["bash"]
