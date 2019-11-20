FROM cmput412_ros

WORKDIR /ros/src
RUN ln -s /source/src/calibration
RUN ln -s /source/src/models

WORKDIR /source

COPY build.bash /tmp/build.bash
RUN source /opt/ros/kinetic/setup.bash && bash /tmp/build.bash
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "source /ros/devel/setup.bash" >> ~/.bashrc

# CMD ["bash", "-c", "./build.bash"]

CMD ["bash"]
